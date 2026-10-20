"""
YOLO buoy detection node for Sailbot.

This module provides a ROS2 node that runs TensorRT-backed YOLO buoy detection and
publishes geographic buoy locations in the same format/channels used by the original
blob-detection pipeline.
"""

import os
import time
from typing import List

import cv2
import numpy as np
import pyzed.sl as sl
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from scipy.optimize import linear_sum_assignment
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from std_msgs.msg import Header

from sailbot.buoy_detection import (
    CAMERA_RESOLUTION,
    CX,
    CY,
    FX,
    FY,
    Track,
    calculate_offset_position,
    enu_to_geodetic,
    geodetic_to_enu,
)
from sailbot_msgs.msg import AnnotatedImage, BuoyDetectionStamped, BuoyTypeInfo, CVParameters


# -----------------------------------------------------------------------------
# Structural Integration Notes (comments only, non-functional)
# -----------------------------------------------------------------------------
# This file intentionally preserves the previous architecture's public outputs
# while replacing the internals of detection and frame acquisition:
# - The old node used ROS image/depth topics + blob/HSV segmentation.
# - This node uses direct ZED SDK capture (`pyzed.sl`) + YOLO TensorRT inference.
# - Output channels are kept compatible (`buoy_position`, `cv_mask`,
#   `initial_cv_parameters`, and `cv_parameters` handling) to stay plug-and-play
#   with existing consumers.
#
# Known working command sequence used during integration/testing:
# 1) cd /home/sailbot/sailbot24-25
# 2) source .venv/bin/activate
# 3) source /opt/ros/humble/setup.bash
# 4) source /home/sailbot/sailbot24-25/sailbot_ws/install/setup.bash
# 5) (if TensorRT Python bindings are from system dist-packages) [dont do this]
#    export PYTHONPATH="/usr/lib/python3/dist-packages:/usr/lib/python3.10/dist-packages:$PYTHONPATH"
# 6) python /home/sailbot/sailbot24-25/sailbot_ws/src/sailbot/sailbot/buoy_detection_yolo.py \      --ros-args --params-file /home/sailbot/sailbot24-25/sailbot_ws/src/sailbot/config/config.yaml
# 7) In a second sourced terminal:
#    ros2 topic hz /cv_mask
#    ros2 topic echo /buoy_position
# -----------------------------------------------------------------------------


class BuoyDetectionYOLO(Node):
    """
    A ROS2 node responsible for detecting buoys using a YOLO TensorRT engine and
    publishing geographic buoy detections.

    This node preserves the external ROS interfaces from the original buoy
    detector where practical, while changing the detection backend from
    blob/HSV logic to neural network inference.

    :ivar current_x_scaling_factor: Scaling factor to adjust the pixel coordinates based on image width.
    :ivar current_y_scaling_factor: Scaling factor to adjust the pixel coordinates based on image height.
    :ivar latitude: Current latitude of the vessel.
    :ivar longitude: Current longitude of the vessel.
    :ivar heading: Current heading of the vessel in degrees.
    :ivar tracks: List of active tracking objects representing detected buoys.
    :ivar buoy_types: Configured buoy metadata used for compatibility and diameter lookup.

    **Methods**:
    - **publish_tracks**: Publishes tracked buoy positions on `buoy_position`.
    - **associate_detections_to_tracks**: Matches frame detections to existing tracks.
    - **capture_and_process**: Captures ZED frames, runs YOLO, projects detections, updates tracks, and publishes output.
    - **pixel_to_world**: Converts pixel coordinates to world coordinates based on camera intrinsics and depth.

    **Usage**:
    - Node can be launched as a drop-in replacement for buoy detection in ROS graph integration.

    **Notes**:
    - This node uses direct ZED SDK frame capture and does not require ROS image topics.
    """

    current_x_scaling_factor = 1.0
    current_y_scaling_factor = 1.0
    latitude, longitude = 42.84456, -70.97622
    heading = 0.0
    tracks: List[Track] = []
    buoy_types: List[BuoyTypeInfo] = []

    def __init__(self):
        super().__init__("object_detection_node")
        self.bridge = CvBridge()

        self._init_default_buoy_types()
        self.set_parameters()
        self.get_parameters()
        self._load_yolo_model()
        self._open_zed()

        self.airmar_position_subscription = self.create_subscription(
            NavSatFix,
            "/airmar_data/lat_long",
            self.airmar_position_callback,
            10,
        )
        self.airmar_heading_subscription = self.create_subscription(
            Float64,
            "heading",
            self.airmar_heading_callback,
            10,
        )
        self.cv_parameters_subscription = self.create_subscription(
            CVParameters,
            "cv_parameters",
            self.cv_parameters_callback,
            10,
        )

        self.mask_publisher = self.create_publisher(AnnotatedImage, "cv_mask", 10)
        self.buoy_position_publisher = self.create_publisher(BuoyDetectionStamped, "buoy_position", 10)
        self.initial_cv_parameters_publisher = self.create_publisher(CVParameters, "initial_cv_parameters", 10)

        # self.publish_initial_cv_parameters()
        self.process_timer = self.create_timer(self.capture_period_seconds, self.capture_and_process)
        self.get_logger().info("YOLO buoy detection setup complete")

    def _init_default_buoy_types(self) -> None:
        orange_type = BuoyTypeInfo()
        orange_type.buoy_diameter = 0.5
        orange_type.name = "orange"
        self.buoy_types.append(orange_type)

        green_type = BuoyTypeInfo()
        green_type.buoy_diameter = 0.5
        green_type.name = "green"
        self.buoy_types.append(green_type)

        white_type = BuoyTypeInfo()
        white_type.buoy_diameter = 0.5
        white_type.name = "white"
        self.buoy_types.append(white_type)

    def set_parameters(self) -> None:
        self.declare_parameter("sailbot.cv.buoy_circularity_threshold", 0.6)
        self.declare_parameter("sailbot.cv.depth_error_threshold_meters", 3.0)
        self.declare_parameter("sailbot.cv.buoy_detection_lifetime_seconds", 3.0)

        self.declare_parameter(
            "sailbot.cv.yolo.engine_path",
            "/home/sailbot/Downloads/yolobuoyV2-engine-validation/my_model_fp16.engine",
        )
        self.declare_parameter("sailbot.cv.yolo.conf_threshold", 0.25)
        self.declare_parameter("sailbot.cv.yolo.iou_threshold", 0.45)
        self.declare_parameter("sailbot.cv.yolo.imgsz", 640)
        self.declare_parameter("sailbot.cv.yolo.device", "0")
        self.declare_parameter("sailbot.cv.yolo.max_detection_distance_meters", 120.0)
        self.declare_parameter("sailbot.cv.yolo.max_association_distance_meters", 3.0)
        self.declare_parameter("sailbot.cv.yolo.capture_period_seconds", 0.05)
        self.declare_parameter("sailbot.cv.yolo.zed_resolution", "VGA")
        self.declare_parameter("sailbot.cv.yolo.zed_depth_mode", "NEURAL")

    def get_parameters(self) -> None:
        self.buoy_circularity_threshold = (
            self.get_parameter("sailbot.cv.buoy_circularity_threshold").get_parameter_value().double_value
        )
        self.depth_error_threshold_meters = (
            self.get_parameter("sailbot.cv.depth_error_threshold_meters").get_parameter_value().double_value
        )
        self.buoy_detection_lifetime_seconds = (
            self.get_parameter("sailbot.cv.buoy_detection_lifetime_seconds").get_parameter_value().double_value
        )

        self.yolo_engine_path = self.get_parameter("sailbot.cv.yolo.engine_path").get_parameter_value().string_value
        self.yolo_conf_threshold = self.get_parameter("sailbot.cv.yolo.conf_threshold").get_parameter_value().double_value
        self.yolo_iou_threshold = self.get_parameter("sailbot.cv.yolo.iou_threshold").get_parameter_value().double_value
        self.yolo_imgsz = self.get_parameter("sailbot.cv.yolo.imgsz").get_parameter_value().integer_value
        self.yolo_device = self.get_parameter("sailbot.cv.yolo.device").get_parameter_value().string_value
        self.max_detection_distance_meters = (
            self.get_parameter("sailbot.cv.yolo.max_detection_distance_meters").get_parameter_value().double_value
        )
        self.max_association_distance_meters = (
            self.get_parameter("sailbot.cv.yolo.max_association_distance_meters").get_parameter_value().double_value
        )
        self.capture_period_seconds = (
            self.get_parameter("sailbot.cv.yolo.capture_period_seconds").get_parameter_value().double_value
        )
        self.zed_resolution_name = self.get_parameter("sailbot.cv.yolo.zed_resolution").get_parameter_value().string_value
        self.zed_depth_mode_name = self.get_parameter("sailbot.cv.yolo.zed_depth_mode").get_parameter_value().string_value

    def _load_yolo_model(self) -> None:
        if not os.path.exists(self.yolo_engine_path):
            raise FileNotFoundError(
                f"YOLO engine not found at '{self.yolo_engine_path}'. "
                "Set sailbot.cv.yolo.engine_path to a valid TensorRT engine."
            )
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is required for buoy_detection_yolo. "
                "Install ultralytics in the runtime environment."
            ) from exc
        self.model = YOLO(self.yolo_engine_path, task="detect")
        self.get_logger().info(f"Loaded YOLO engine: {self.yolo_engine_path}")

    def _open_zed(self) -> None:
        resolution_map = {
            "VGA": sl.RESOLUTION.VGA,
            "HD720": sl.RESOLUTION.HD720,
            "HD1080": sl.RESOLUTION.HD1080,
            "HD2K": sl.RESOLUTION.HD2K,
        }
        depth_map = {
            "PERFORMANCE": sl.DEPTH_MODE.PERFORMANCE,
            "QUALITY": sl.DEPTH_MODE.QUALITY,
            "ULTRA": sl.DEPTH_MODE.ULTRA,
            "NEURAL": sl.DEPTH_MODE.NEURAL,
        }
        resolution = resolution_map.get(self.zed_resolution_name.upper(), sl.RESOLUTION.VGA)
        depth_mode = depth_map.get(self.zed_depth_mode_name.upper(), sl.DEPTH_MODE.NEURAL)

        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = resolution
        init_params.depth_mode = depth_mode
        init_params.coordinate_units = sl.UNIT.METER

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED open failed: {status}")

        self.runtime_params = sl.RuntimeParameters()
        self.runtime_params.enable_depth = True
        self.image_zed = sl.Mat()
        self.depth_zed = sl.Mat()
        self.get_logger().info(
            f"Opened ZED with resolution={self.zed_resolution_name}, depth_mode={self.zed_depth_mode_name}"
        )

    def publish_initial_cv_parameters(self) -> None:
        initial_parameters = CVParameters()
        initial_parameters.buoy_types.extend(self.buoy_types)
        initial_parameters.circularity_threshold = self.buoy_circularity_threshold

        deadline = time.time() + 10.0
        while self.initial_cv_parameters_publisher.get_subscription_count() < 1 and time.time() < deadline:
            self.get_logger().info("waiting for cv parameters subscriber...")
            time.sleep(1)

        self.initial_cv_parameters_publisher.publish(initial_parameters)
        self.get_logger().info(
            f"Published initial cv parameters with {self.initial_cv_parameters_publisher.get_subscription_count()} subscribers"
        )

    def airmar_position_callback(self, msg: NavSatFix) -> None:
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def airmar_heading_callback(self, msg: Float64) -> None:
        self.heading = msg.data

    def cv_parameters_callback(self, msg: CVParameters) -> None:
        self.buoy_types = msg.buoy_types
        self.buoy_circularity_threshold = msg.circularity_threshold

    def remove_stale_tracks(self) -> None:
        current_time = time.time()
        self.tracks = [
            track for track in self.tracks if current_time - track.last_update_time <= self.buoy_detection_lifetime_seconds
        ]

    def publish_tracks(self, header: Header) -> None:
        for track in self.tracks:
            if track.time_since_update != 0:
                continue
            enu_position = track.get_position()
            lat, lon = enu_to_geodetic(enu_position[0], enu_position[1])
            detection = BuoyDetectionStamped()
            detection.header = header
            detection.position.latitude = lat
            detection.position.longitude = lon
            detection.id = track.id
            self.buoy_position_publisher.publish(detection)

    def associate_detections_to_tracks(self, tracks, detections_enu, max_distance: float):
        if len(tracks) == 0:
            return [], set(range(len(detections_enu)))
        if len(detections_enu) == 0:
            return [], []

        cost_matrix = np.zeros((len(tracks), len(detections_enu)), dtype=np.float32)
        for t, track in enumerate(tracks):
            predicted_enu = track.predict()
            for d, detection_enu in enumerate(detections_enu):
                cost_matrix[t, d] = np.linalg.norm(np.array(predicted_enu) - np.array(detection_enu))

        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        matches = []
        unmatched_detections = set(range(len(detections_enu)))
        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r, c] < max_distance:
                matches.append((r, c))
                unmatched_detections.remove(c)
        return matches, list(unmatched_detections)

    def _lookup_class_name(self, class_id: int) -> str:
        names = self.model.names
        if isinstance(names, dict):
            return str(names.get(class_id, "buoy"))
        if isinstance(names, list) and 0 <= class_id < len(names):
            return str(names[class_id])
        return "buoy"

    def _lookup_diameter(self, class_name: str) -> float:
        target = class_name.lower()
        for buoy_type in self.buoy_types:
            if buoy_type.name.lower() == target:
                return float(buoy_type.buoy_diameter)
        return 0.5

    def _get_valid_depth(self, cx: int, cy: int) -> float | None:
        depth_err, depth = self.depth_zed.get_value(cx, cy)
        if depth_err != sl.ERROR_CODE.SUCCESS:
            return None
        depth_value = float(depth)
        if np.isnan(depth_value) or depth_value <= 0.0 or not np.isfinite(depth_value):
            return None
        if depth_value > self.max_detection_distance_meters:
            return None
        return depth_value

    def pixel_to_world(self, x_pixel: int, y_pixel: int, depth: float, f_x: float, f_y: float, c_x: float, c_y: float):
        x_norm = (x_pixel - c_x) / f_x
        y_norm = (y_pixel - c_y) / f_y
        x_world = x_norm * depth
        y_world = y_norm * depth
        return x_world, y_world, depth

    def capture_and_process(self) -> None:
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return

        self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT, sl.MEM.CPU)
        
        self.zed.retrieve_measure(self.depth_zed, sl.MEASURE.DEPTH, sl.MEM.CPU)

        frame = self.image_zed.get_data()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        self.current_x_scaling_factor = frame_bgr.shape[1] / CAMERA_RESOLUTION[0]
        self.current_y_scaling_factor = frame_bgr.shape[0] / CAMERA_RESOLUTION[1]

        results = self.model.predict(
            frame_bgr,
            conf=self.yolo_conf_threshold,
            iou=self.yolo_iou_threshold,
            imgsz=self.yolo_imgsz,
            device=self.yolo_device,
            verbose=False,
        )
        if not results:
            return

        detections_enu = []
        annotated = frame_bgr.copy()
        boxes = results[0].boxes

        if boxes is not None and boxes.xyxy is not None and len(boxes.xyxy) > 0:
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy() if boxes.conf is not None else np.ones((len(xyxy),), dtype=np.float32)
            classes = boxes.cls.cpu().numpy().astype(int) if boxes.cls is not None else np.zeros((len(xyxy),), dtype=int)

            for bbox, conf, class_id in zip(xyxy, confs, classes):
                x1, y1, x2, y2 = bbox.tolist()
                cx = int((x1 + x2) / 2.0)
                cy = int((y1 + y2) / 2.0)
                depth = self._get_valid_depth(cx, cy)
                if depth is None:
                    continue

                class_name = self._lookup_class_name(int(class_id))
                diameter = self._lookup_diameter(class_name)
                world_coords = self.pixel_to_world(
                    cx,
                    cy,
                    depth,
                    FX * self.current_x_scaling_factor,
                    FY * self.current_y_scaling_factor,
                    CX * self.current_x_scaling_factor,
                    CY * self.current_y_scaling_factor,
                )
                latlon = calculate_offset_position(self.latitude, self.longitude, self.heading, world_coords[2], world_coords[0])
                detections_enu.append(geodetic_to_enu(latlon.latitude, latlon.longitude))

                cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                label = f"{class_name} {conf:.2f} {depth:.1f}m d={diameter:.2f}"
                cv2.putText(
                    annotated,
                    label,
                    (int(x1), max(0, int(y1) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )

        matches, unmatched = self.associate_detections_to_tracks(
            self.tracks, detections_enu, self.max_association_distance_meters
        )
        for track_idx, det_idx in matches:
            self.tracks[track_idx].update(detections_enu[det_idx])
        for det_idx in unmatched:
            enu = detections_enu[det_idx]
            self.tracks.append(Track(enu[0], enu[1]))

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "zed_left_camera"
        self.publish_tracks(header)
        self.remove_stale_tracks()

        annotated_rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
        img_msg = self.bridge.cv2_to_imgmsg(annotated_rgb, encoding="rgb8")
        self.mask_publisher.publish(AnnotatedImage(image=img_msg, source="yolo"))

    def destroy_node(self):
        if hasattr(self, "zed") and self.zed is not None:
            self.zed.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BuoyDetectionYOLO()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()