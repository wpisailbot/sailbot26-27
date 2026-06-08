#!/usr/bin/env python3

import rclpy
from typing import Optional
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.subscription import Subscription
from time import time as get_time

from std_msgs.msg import Int8, Int16, Empty, Float32, Float64, String, Bool
from sailbot_msgs.msg import Wind, AutonomousMode, GeoPath, TrimState, BuoyDetectionStamped
from sailbot_msgs.srv import RestartNode

import serial
import json
import traceback
import serial.tools.list_ports
import subprocess
import can  # pip install python-can
from phoenix6 import hardware, controls, configs, signals
            

serial_port = '/dev/ttyTHS1'
baud_rate = 115200 

# Local variables
angle = 0
wind_dir = 0.0
battery_level = 100

def find_esp32_serial_ports() -> list:
    # Common VID:PID pairs for USB-to-Serial adapters used with ESP32
    esp32_vid_pid = [
        ('10C4', 'EA60'),  # Silicon Labs CP210x
        ('1A86', '7523'),  # HL-340
        ('0403', '6001'),  # FTDI
    ]
    
    ports = serial.tools.list_ports.comports()
    esp32_ports = []
    
    for port in ports:
        vid_pid = (hex(port.vid)[2:].upper(), hex(port.pid)[2:].upper()) if port.vid and port.pid else ('', '')
        if vid_pid in esp32_vid_pid:
            esp32_ports.append(port.device)
    
    return esp32_ports


class ESPComms(LifecycleNode):
    """
    A ROS2 Lifecycle Node for handling communications with the onboard ESP32 for rudder, trim tab, and ballast control.

    :ivar last_winds: Stores recent wind measurements for processing.
    :ivar autonomous_mode: Current mode of operation, based on 'AutonomousMode' message.
    :ivar force_neutral_position: Flag to keep the trim tab in a neutral position, regardless of wind conditions.
    :ivar could_be_tacking: Indicates if the boat is be performing a tacking maneuver.
    :ivar last_lift_state: Last state of the trim tab concerning lift, stored as a 'TrimState'.
    :ivar rudder_angle_limit_deg: Configurable limit for rudder angle to avoid extreme positions and stalls.
    :ivar tailscale_connected: Status of Tailscale connectivity for remote access.
    :ivar launch_complete: Indicates whether the system has completed its launch sequence.
    :ivar trim_auto: Indicates if the trim tab is in automatic mode.
    :ivar rudder_auto: Indicates if the rudder is in automatic mode.
    :ivar battery_ok: Status of the Jetson battery, indicating if it's within acceptable levels.
    """


    # damper CAN bus tracking variables
    last_roll_readings = []
    max_roll_samples = 30  
    damper_active = False

    # Damper control mode (0=AUTO, 1=MANUAL_ON, 2=MANUAL_OFF)
    damper_mode = 0  # Start in AUTO mode
    last_damper_toggle_time = 0.0  
    damper_toggle_debounce_time = 1.5

    last_roll_values_timeout = 10.0
    oscillation_threshold_deg = 10.0
    small_oscillation_threshold_deg = 5
    oscillation_count_threshold = 5
    oscillation_time_window = 18.0
    last_oscillation_times = []
    last_small_oscillation_times = []  
    last_roll_direction = 0           # -1 = port, 0 = neutral, 1 = starboard
    speed = 0.0
    heartbeat = True
    heartbeat_fail = 0

    # Wingsail LED display parameters
    status_timer: Optional[Timer] = None
    tailscale_connected = False
    launch_complete = False
    trim_auto = False
    rudder_auto = False
    battery_ok = True

    critical_nodes = [
        "airmar_reader",
        "path_follower",
        "heading_controller"
    ]
    
    last_heartbeat_times = {}
    heartbeat_timeout = 5.0  # seconds - if no heartbeat in 5s, node is dead

    buoy_detected = False  # Buoy detection flag
    reach_buoy = False
    last_buoy_detection_time = 0.0

    last_winds = []
    autonomous_mode = 0
    force_neutral_position = True
    could_be_tacking = False
    last_lift_state = TrimState.TRIM_STATE_MIN_LIFT
    rudder_angle_limit_deg = None

    request_tack_timer_duration = 3.0  # seconds
    request_tack_timer: Timer = None

    request_jibe_timer_duration = 5.0  # seconds
    request_jibe_timer: Timer = None

    request_tack_override = False
    sent_clear_winds_this_tack = False

    request_jibe_override = False

    def __init__(self):
        super(ESPComms, self).__init__('esp32_comms')

        self.set_parameters()
        self.get_parameters()

        self.tt_battery_publisher: Optional[Publisher]
        self.ballast_pos_publisher: Optional[Publisher]
        self.trim_state_debug_publisher: Optional[Publisher]
        self.damper_state_publisher: Optional[Publisher]

        self.error_publisher: Optional[Publisher]

        self.tt_control_subscriber: Optional[Subscription]
        self.tt_angle_subscriber: Optional[Subscription]
        self.rudder_angle_subscriber: Optional[Subscription]

        self.autonomous_mode_subscriber: Optional[Subscription]
        self.current_path_subscription: Optional[Subscription]
        self.apparent_wind_subscriber: Optional[Subscription]
        self.roll_subscription: Optional[Subscription]
        self.speed_subscription: Optional[Subscription]
        self.damper_mode_subscription: Optional[Subscription]

        self.request_tack_subscription: Optional[Subscription]

        self.timer_pub: Optional[Publisher]

        self.heartbeat_timer: Optional[Timer]
        self.timer: Optional[Timer]
        self.last_successful_write = get_time()
        self.restart_cli = self.create_client(RestartNode, 'state_manager/restart_node')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def set_parameters(self) -> None:
        self.declare_parameter('sailbot.rudder.angle_limit_deg', 30)

    def get_parameters(self) -> None:
        self.rudder_angle_limit_deg = self.get_parameter('sailbot.rudder.angle_limit_deg').get_parameter_value().integer_value

    #lifecycle node callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("In configure")

        # Wingsail LED update timer
        self.status_timer = self.create_timer(1.0, self.status_timer_callback)
        self.status_check_timer = self.create_timer(1.0, self.status_check_callback)

        current_time = get_time()
        for node_name in self.critical_nodes:
            self.last_heartbeat_times[node_name] = current_time
            
            # Subscribe to each node's heartbeat topic
            self.create_subscription(
                Empty,
                f'/heartbeat/{node_name}',
                lambda msg, name=node_name: self.heartbeat_callback(msg, name),
                10
            )

        # Initialize CAN bus for damper control
        try:
            
            DAMPER_MOTOR_ID = 1  # ← Change to your Talon FX CAN ID
            CAN_BUS = "can0"
            
            # Create motor object
            self.damper_motor = hardware.TalonFX(DAMPER_MOTOR_ID, CAN_BUS)
            
            # Base configuration
            # config = configs.TalonFXConfiguration()
            
            # IMPORTANT: Set initial neutral mode to COAST (damper OFF by default)
            # config.motor_output.neutral_mode = signals.NeutralModeValue.COAST
            # config.motor_output.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            
            # Safety: Current limits (important for brake mode)
            # config.current_limits.stator_current_limit = 40.0  # Amps
            # config.current_limits.stator_current_limit_enable = True
            
            # Apply initial config
            # status = self.damper_motor.configurator.apply(config, timeout_seconds=0.5)
            
            # if status.is_ok():
            #     self.get_logger().info(f"✓ Damper motor initialized : {status}")
            # else:
            #     self.get_logger().warn(f"⚠️ Damper config warning: {status}")
            
            # Create control request to stop motor at 0% output
            # (Motor will use whatever neutral mode is currently set)
            self.damper_motor.set_control(controls.DutyCycleOut(0.0))
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize damper motor: {e}")
            self.damper_motor = None
        
        self.roll_subscription = self.create_subscription(Float64, '/airmar_data/roll',self.roll_callback, 10)
        self.speed_subscription = self.create_subscription(Float64, '/airmar_data/speed_knots',self.speed_callback, 10)
        self.damper_mode_subscription = self.create_subscription(Empty, 'damper_mode', self.damper_mode_callback, 10)
        # uncomment this when the fix works
        self.reach_buoy_subscription = self.create_subscription(Bool, 'reached_buoy', self.reach_buoy_callback, 10)
        
        self.damper_check_timer = self.create_timer(0.5,self.damper_check_callback)

        #reset ESP32 in case it stopped working from brownout
        esp32_ports = find_esp32_serial_ports()
        if esp32_ports:
            self.get_logger().info("ESP32 may be connected to the following ports:")
            for port in esp32_ports:
                print(port)
                try:
                    subprocess.run(['python3', '-m', 'esptool', '--port', port, 'run'], check=True)
                except Exception as e:
                    self.get_logger().error(f"ESP is not responding!")
                    raise(e)
        else:
            self.get_logger().warn("No ESP32 ports found!")

        self.ballast_pos_publisher = self.create_lifecycle_publisher(Int16, 'current_ballast_position', 10)

        self.tt_battery_publisher = self.create_lifecycle_publisher(Int8, 'tt_battery', 10)  # Battery level
        self.trim_state_debug_publisher = self.create_lifecycle_publisher(TrimState, 'trim_state', 10)
        self.damper_state_publisher = self.create_lifecycle_publisher(Int8, 'damper_state', 10)

        self.error_publisher = self.create_lifecycle_publisher(String, f'{self.get_name()}/error', 10)

        # Subscribe to buoy detections
        self.buoy_detection_subscription = self.create_subscription(
            BuoyDetectionStamped,
            '/buoy_position',
            self.buoy_detection_callback,
            10
        )

        self.tt_angle_subscriber = self.create_subscription(Int16, 'tt_angle', self.tt_angle_callback, 10)

        self.rudder_angle_subscriber = self.create_subscription(Int16, 'rudder_angle', self.rudder_angle_callback, 10)
        # self.ballast_pwm_subscriber = self.create_subscription(Int16, 'ballast_pwm', self.ballast_pwm_callback, 10)

        self.apparent_wind_subscriber = self.create_subscription(Wind, 'apparent_wind_smoothed', self.apparent_wind_callback, 10)

        self.autonomous_mode_subscriber = self.create_subscription(AutonomousMode, 'autonomous_mode', self.autonomous_mode_callback, 10)

        self.current_path_subscription = self.create_subscription(
            GeoPath,
            'current_path',
            self.current_path_callback,
            10)
        
        
        self.request_tack_subscription = self.create_subscription(
            Empty,
            'request_tack',
            self.request_tack_callback,
            10)
        
        self.request_jibe_subscription = self.create_subscription(Float32, 'request_jibe', self.request_jibe_callback, 10)

        

        self.timer_pub = self.create_lifecycle_publisher(Empty, '/heartbeat/trim_tab_comms', 1)
        
        self.esp_heartbeat = self.create_timer(1, self.esp_heartbeat_callback)

        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.05, write_timeout=1.0)
        except Exception as e:
            self.get_logger().info(str(e))
            
        self.serial_watchdog_timer = self.create_timer(
            1.0,
            self.serial_watchdog_callback
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating...")
        # Start publishers or timers
        return super().on_activate(state)


    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")
        return super().on_deactivate(state)
        

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")
        # Destroy subscribers, publishers, and timers
        self.destroy_lifecycle_publisher(self.tt_battery_publisher)
        self.destroy_lifecycle_publisher(self.ballast_pos_publisher)
        self.destroy_lifecycle_publisher(self.timer_pub)
        self.destroy_lifecycle_publisher(self.damper_state_publisher)
        self.destroy_subscription(self.tt_control_subscriber)
        self.destroy_subscription(self.tt_angle_subscriber)
        self.destroy_timer(self.heartbeat_timer)
        self.destroy_timer(self.status_timer)
        self.destroy_timer(self.status_check_timer)
        self.destroy_timer(self.damper_check_timer)
        if hasattr(self, 'damper_motor') and self.damper_motor is not None:
            try:
                self.damper_motor.set_control(controls.CoastOut())  # ← Explicit COAST
                self.get_logger().info("✓ Damper set to COAST mode")
            except:
                self.get_logger().error("⚠️ Could not stop damper motor!")
        # uncomment when we fix the damper

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        # Perform final cleanup if necessary
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().error("Error caught!")
        return super().on_error(state)
    
    #end callbacks

    def current_path_callback(self, msg: GeoPath) -> None:
        if len(msg.points) == 0:
            self.force_neutral_position = True
        else:
            # self.get_logger().info("Valid path received, allowing auto trimtab movement")
            self.force_neutral_position = False

    def autonomous_mode_callback(self, msg: AutonomousMode) -> None:
        # self.get_logger().info(f"Got autonomous mode: {msg.mode}")
        if(msg.mode == AutonomousMode.AUTONOMOUS_MODE_NONE):
            message = {
                "state": "manual",
            }
            trim_state_msg = TrimState()
            trim_state_msg.state = TrimState.TRIM_STATE_MANUAL
            self.trim_state_debug_publisher.publish(trim_state_msg)

        self.autonomous_mode = msg.mode
        # update wingsail display status
        self.trim_auto = (msg.mode == AutonomousMode.AUTONOMOUS_MODE_TRIMTAB or 
                            msg.mode == AutonomousMode.AUTONOMOUS_MODE_FULL)
        self.rudder_auto = (msg.mode == AutonomousMode.AUTONOMOUS_MODE_FULL)

    def apparent_wind_callback(self, msg: Wind) -> None:
        #self.get_logger().info(f"Got apparent wind: {msg.direction}")
        self.find_trim_tab_state(msg.direction)

    def find_trim_tab_state(self, relative_wind) -> None:  # five states of trim
        """
        Determines the trim tab state based on the relative wind angle and current autonomous mode.
        
        This function sets the trim tab state to optimize the sail's position by adjusting the trim tab
        to either maximize lift or drag on the port or starboard side, or to minimize lift when "in irons" (directly into the wind).
        The function also handles state changes during tacking by detecting tacking signals from the rudder controller.

        :param relative_wind: The angle of the wind relative to the boat, in degrees.

        **Key Steps**:
        - **Mode Check**: Exits if the system is not in full autonomous or trim tab autonomous mode.
        - **State Determination**: Sets the trim tab state based on the relative wind angle.
        - **Tacking Adjustment**: Adjusts trim tab states during tacking maneuvers.
        - **Serial Communication**: Sends the determined state to the ESP32 for execution.

        **Trim Tab States**:
        - **Max Lift Port**: Applied when the relative wind is between 25 and 100 degrees.
        - **Max Drag Port**: Applied when the relative wind is between 100 and 180 degrees.
        - **Max Drag Starboard**: Applied when the relative wind is between 180 and 260 degrees.
        - **Max Lift Starboard**: Applied when the relative wind is between 260 and 335 degrees.
        - **Min Lift**: Default state, used when the wind angle does not match any other conditions or when in irons.

        **Behavior**:
        - Updates the trim tab state based on wind angle, adjusting for tacking if necessary.
        - Publishes the new state to a ROS topic for other components to utilize.
        - Handles exceptional cases where the boat might be in an unexpected state due to sudden wind changes.

        """
        #self.get_logger().info(f"apparent wind: {relative_wind}")
        
        # Check autonomous mode TODO: This is a coupling that shouldn't be necessary. 
        # Can be fixed by separating nodes and using lifecycle state transitions, or by finishing behavior tree
        autonomous_modes = AutonomousMode()
        #self.get_logger().info(f"Auto mode: {self.autonomous_mode}")
        if ((self.autonomous_mode != autonomous_modes.AUTONOMOUS_MODE_FULL) and (self.autonomous_mode != autonomous_modes.AUTONOMOUS_MODE_TRIMTAB)):
            #self.get_logger().info(f"Skipping")
            return

        msg = None
        trim_state_msg = TrimState()

        force_tack = False
        if(self.request_tack_override):
            # Only tack if we're going upwind
            if (0 <= relative_wind < 100) or (260 <= relative_wind < 365):
                force_tack = True
            else:
                self.get_logger().warn("Tack requested, but we're going downwind...")

        if 25.0 <= relative_wind < 100 and force_tack is False:
            # Max lift port
            msg = {
                "state": "max_lift_port"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MAX_LIFT_PORT
            self.last_lift_state = TrimState.TRIM_STATE_MAX_LIFT_PORT
            #self.get_logger().info("Max lift port")
        elif 100 <= relative_wind < 180 and self.request_jibe_override is False:
            # Max drag port
            msg = {
                "state": "max_drag_starboard" # switched for testing, need to swap in trim_tab_client
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MAX_DRAG_PORT
            #self.last_state = TrimState.TRIM_STATE_MAX_DRAG_PORT
            #self.get_logger().info("Max drag port")

        elif 180 <= relative_wind < 260 and self.request_jibe_override is False:
            # Max drag starboard
            msg = {
                "state": "max_drag_port"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MAX_DRAG_STARBOARD
            #self.last_state = TrimState.TRIM_STATE_MAX_DRAG_STARBOARD
            #self.get_logger().info("Max drag starboard")
        elif 260 <= relative_wind < 335 and force_tack is False:
            # Max lift starboard
            msg = {
                "state": "max_lift_starboard"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MAX_LIFT_STARBOARD
            self.last_lift_state = TrimState.TRIM_STATE_MAX_LIFT_STARBOARD
            # self.get_logger().info("Max lift starboard")
        else:
            # clear_winds was crashing trimtab during competition. Didn't have time to debug.

            # # Adjust behavior to not stop during a tack
            # if(self.could_be_tacking or force_tack):
            #     self.get_logger().info("Tacking detected!")
            #     if(self.switched_sides_this_tack is False):
            #         self.switched_sides_this_tack = True
            #         if(self.last_lift_state == TrimState.TRIM_STATE_MAX_LIFT_STARBOARD):
            #             trim_state_msg.state = TrimState.TRIM_STATE_MAX_LIFT_PORT
            #             self.get_logger().info("Switching from starboard to port")
            #             msg = {
            #                 #"clear_winds": True,
            #                 "state": "min_lift"
            #             }
            #         elif (self.last_lift_state == TrimState.TRIM_STATE_MAX_LIFT_PORT):
            #             self.get_logger().info("Switching from port to starboard")
            #             trim_state_msg.state = TrimState.TRIM_STATE_MAX_LIFT_STARBOARD
            #             msg = {
            #                 #"clear_winds": True,
            #                 "state": "min_lift"
            #             }
            #         else:
            #             # How did we get here?
            #             self.get_logger().warn("Went into min lift in tack mode, but previous state was not max lift. Did the wind change suddenly?")
            #             msg = {
            #                 #"clear_winds": True,
            #                 "state": "min_lift"
            #             }
            #             trim_state_msg.state = TrimState.TRIM_STATE_MIN_LIFT
            # else:
            msg = {
                "state": "min_lift"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MIN_LIFT

        #if we're in full auto and have no target, don't go anywhere
        if self.force_neutral_position and self.autonomous_mode == AutonomousMode.AUTONOMOUS_MODE_FULL:
            msg = {
                "state": "min_lift"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MIN_LIFT
            # self.get_logger().info("Force neutral")
        
        if(msg is not None):
            self.trim_state_debug_publisher.publish(trim_state_msg)
            self.serial_write(msg)
        else:
            self.get_logger().info("Trim message is None, taking no action")

    def tt_angle_callback(self, msg: Int16) -> None:
        # self.get_logger().info("Sending trimtab angle")
        angle = msg.data
        this_time = get_time()
        message = {
            "state": "manual",
            "angle": angle,
            "timestamp": this_time
        }
        self.serial_write(message)

    # Wingsail LED display helper functions
    def send_system_status(self, tailscale_connected: bool, buoy_detected: bool, 
                       trim_auto: bool, rudder_auto: bool, battery_ok: bool,reach_buoy:bool):
        """Send multiple system status flags to ESP32 in one message"""
        message = {
            "tailscale": tailscale_connected,
            "found_buoy": buoy_detected,
            "trim_auto": trim_auto,
            "rudder_auto": rudder_auto,
            "battery_ok": battery_ok,
            "reach_buoy": reach_buoy,
        }
        self.serial_write(message)


    
    def check_tailscale(self) -> bool:
        return
        """Check if Tailscale is connected
        
        Returns:
            bool: True if DOWN (problem), False if connected (OK)
        """
        try:
            result = subprocess.run(
                ['tailscale', 'status'],
                capture_output=True,
                text=True,
                timeout=2.0
            )
            
            if result.returncode != 0:
                return True  # Command failed = problem
            
            first_line = result.stdout.split('\n')[0].lower()
            
            # Good if: has "ubuntu" AND does NOT have "offline"
            is_good = ('ubuntu' in first_line) and ('offline' not in first_line)
            
            if not is_good:
                self.get_logger().warn(f"Tailscale problem: {result.stdout.split(chr(10))[0]}")
            
            return not is_good  # Return True if problem
            
        except Exception as e:
            self.get_logger().error(f"Tailscale check failed: {e}")
            return True
        
    def check_node_heartbeats(self) -> bool:
        """Check if all critical nodes are alive
        
        Returns:
            bool: True if ANY node is dead, False if all OK
        """
        current_time = get_time()
        
        for node_name, last_time in self.last_heartbeat_times.items():
            time_since_heartbeat = current_time - last_time
            
            if time_since_heartbeat > self.heartbeat_timeout:
                self.get_logger().warn(
                    f"Node {node_name} is DEAD! Last heartbeat {time_since_heartbeat:.1f}s ago"
                )
                return True  # Problem detected!
        
        # All nodes are alive
        return False
        

    def heartbeat_callback(self, msg: Empty, node_name: str):
        """Called whenever a node sends a heartbeat - just update timestamp"""
        self.last_heartbeat_times[node_name] = get_time()

    def buoy_detection_callback(self, msg: BuoyDetectionStamped):
        """Called whenever a buoy is detected"""
        self.last_buoy_detection_time = get_time()
        self.buoy_detected = True
        
        # self.get_logger().info(
        #     f"Buoy detected! ID: {msg.id}, "
        #     f"Lat: {msg.position.latitude:.6f}, "
        #     f"Lon: {msg.position.longitude:.6f}"
        # )

    def reach_buoy_callback(self, msg: Bool):
        """Called when we reach the buoy"""
        self.reach_buoy = msg.data
        # self.get_logger().info(f"Reached buoy: {self.reach_buoy}")

    def status_check_callback(self):
        """Periodically check and update system status variables"""
        # Update all global status variables

        # Check if we've seen a buoy recently (within 5 seconds)
        current_time = get_time()
        time_since_buoy = current_time - self.last_buoy_detection_time
        
        # Buoy detected if we saw one in last 5 seconds
        self.buoy_detected = (time_since_buoy < 10.0)

        self.tailscale_connected = self.check_tailscale()
        self.battery_ok = not self.battery_ok # waiting for BMS
        self.launch_complete = self.check_node_heartbeats() #uncommented this line 
        # trim_auto and rudder_auto are already updated in autonomous_mode_callback
        
    def status_timer_callback(self):
        """Called every 1 second by the timer - sends status to ESP32"""
        self.send_system_status(
            self.tailscale_connected,
            self.buoy_detected,
            self.trim_auto,
            self.rudder_auto,
            self.battery_ok,
            self.reach_buoy
        )

    def rudder_angle_callback(self, msg: Int16) -> None:
        """
        Callback function that processes received rudder angle data, applies constraints, and sends a corrected value to the ESP32.

        This function is triggered by a ROS subscription whenever a new rudder angle message is published. It checks the received
        angle against preset limits, adjusts the angle if necessary to prevent extreme positions, and then sends the corrected
        rudder angle to the ESP32 via serial communication.

        :param msg: A message containing the rudder angle as an integer.

        **Process**:

        - **Angle Limiting**: Checks if the received rudder angle exceeds preset limits (''self.rudder_angle_limit_deg'').
        - **Tacking Detection**: Sets a flag (''self.could_be_tacking'') if the rudder angle is beyond its limit, which heading_controller will use to indicate tacking.
        - **Serial Communication**: Sends the adjusted rudder angle to the ESP32 in a JSON formatted string over a serial connection.

        **Example of Serial Message**:

        - Sent: '{"rudder_angle": 20}'

        **Usage**:

        - The node must be managed by state_manager

        **Note**:
        
        - The function modifies the state variable ''self.could_be_tacking'' based on the rudder angle's relation to its limits.

        """

        #self.get_logger().info(f"Got rudder position: {msg.data}")
        degrees = msg.data
        # If rudder angles are high, limit them, and note that we could be tacking
        # This lets find_trim_tab_state adjust its behavior accordingly, if it would enter min_lift.
        if(degrees>self.rudder_angle_limit_deg):
            self.could_be_tacking = True
            degrees = self.rudder_angle_limit_deg
        elif (degrees<-self.rudder_angle_limit_deg):
            self.could_be_tacking = True
            degrees = -self.rudder_angle_limit_deg
        else:
            self.could_be_tacking = False
        #degrees = degrees+13 #Servo degree offset
        message = {
            "rudder_angle": degrees
        }
        message_string = json.dumps(message)+'\n'
        # self.get_logger().info("Attempting Rudder Send")
        self.ser.write(message_string.encode())

    def ballast_pwm_callback(self, msg: Int16) -> None:
        #self.get_logger().info("Got ballast position")
        pwm = msg.data
        message = {
            "ballast_pwm": pwm
        }
        self.serial_write(message)
    
    def damper_check_callback(self):
        """Check if damper should activate based on IMU data"""

        # Only run in AUTO mode (mode 0)
        if self.damper_mode != 0:
            return  # Skip - we're in manual mode
    
        if self.speed > 10.0:
            if not self.damper_active:
                self.damper_active = False
                self.send_damper_can_command(self.damper_active)
            return
        else:
            # Check if we have enough data
            if len(self.last_roll_readings) < 5:
                self.get_logger().info("no enough data")
                return
            
            current_time, current_roll = self.last_roll_readings[-1]

            # Clean up old readings (remove values older than timeout)
            cutoff_time = current_time - self.last_roll_values_timeout
            self.last_roll_readings = [
                (t, v) for t, v in self.last_roll_readings 
                if t >= cutoff_time
            ]

            roll_values = [v for t, v in self.last_roll_readings]
            neutral_zone = 1.0  # degrees
            if current_roll > neutral_zone:
                current_direction = 1  # Starboard
            elif current_roll < -neutral_zone:
                current_direction = -1  # Port
            else:
                current_direction = 0  # Neutral
            # self.get_logger().info(f"current direction: {current_direction}")

            # if current != 0, and different directions, then we crossed zero.
            crossed_zero = (current_direction != 0 and self.last_roll_direction != current_direction)

            if crossed_zero:
                # Find peak from previous direction
                if self.last_roll_direction == 1:
                    peak = max(roll_values)  # Was starboard, find max
                elif self.last_roll_direction == -1:
                    peak = min(roll_values)
                else:  # last_roll_direction == 0 (first crossing)
                    # Use most extreme value (furthest from 0)
                    peak = max(roll_values, key=abs)
                
                # Calculate amplitude
                amplitude = abs(peak - current_roll)
                
                # Check if amplitude is large enough
                if amplitude >= self.oscillation_threshold_deg:
                    self.last_oscillation_times.append(current_time)
                    # self.get_logger().info(f"✓ Oscillation detected! Amplitude: {amplitude:.1f}°")
                    # self.get_logger().info(f"✓ Oscillations: {len(self.last_oscillation_times):.1f}")
                
                # Check if it's a SMALL oscillation
                if amplitude >= self.small_oscillation_threshold_deg:
                    self.last_small_oscillation_times.append(current_time)
                    # self.get_logger().info(
                    #     f"🟡 Small oscillation. Amplitude: {amplitude:.1f}° ")
                    # self.get_logger().info(
                    #     f"🟡 Small oscillations: {len(self.last_small_oscillation_times):.1f}")
                

            if current_direction != 0:
                # update direction
                self.last_roll_direction = current_direction
            
            # Clean up old oscillations
            self.last_oscillation_times = [
                t for t in self.last_oscillation_times 
                if (current_time - t) <= self.oscillation_time_window
            ]

            self.last_small_oscillation_times = [
                t for t in self.last_small_oscillation_times 
                if (current_time - t) <= self.oscillation_time_window
            ]

            # should_activate = False 
            recent_oscillation_count = len(self.last_oscillation_times)
            recent_small_oscillation_count = len(self.last_small_oscillation_times)
            if recent_oscillation_count >= self.oscillation_count_threshold:
                should_activate = True
            if recent_small_oscillation_count < self.oscillation_count_threshold:
                should_activate = False

            # self.get_logger().info(f"Damper command: {should_activate}")
            # If state changed, send CAN command
            if should_activate != self.damper_active:
                self.damper_active = should_activate
                self.send_damper_can_command(self.damper_active)
    
    def send_damper_can_command(self, switch:bool):
        if not hasattr(self, 'damper_motor') or self.damper_motor is None:
            self.get_logger().warn("⚠️ Damper motor not initialized!")
            return
        
        try:
            config = configs.TalonFXConfiguration()
            if switch:
                # Damper ON = Send StaticBrake control
                # This actively brakes the motor (shorts the windings)
                # brake_request = controls.StaticBrake()
                # self.damper_motor.set_control(brake_request)
                config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
                self.damper_motor.configurator.apply(config)

                self.get_logger().info("🟢 DAMPER ON (active brake)")
            else:
                # Damper OFF = Send NeutralOut in COAST mode
                # # Motor spins freely with no resistance
                # coast_request = controls.CoastOut()
                # self.damper_motor.set_control(coast_request)
                config.motor_output.neutral_mode = signals.NeutralModeValue.COAST
                self.damper_motor.configurator.apply(config, timeout_seconds=0.5)

                self.get_logger().info("🔴 DAMPER OFF (coasting)")
                
        except Exception as e:
            self.get_logger().error(f"❌ Failed to control damper: {e}")

    def damper_mode_callback(self, msg: Empty):
        """Cycle damper mode: AUTO → MANUAL_ON → MANUAL_OFF → AUTO"""
    
        # current_time = get_time()
        
        # # Debounce check
        # time_since_last = current_time - self.last_damper_toggle_time
        # if time_since_last < self.damper_toggle_debounce_time:
        #     self.get_logger().info(f"⏸️ Debouncing ({time_since_last:.2f}s)")
        #     return
        
        # self.last_damper_toggle_time = current_time
        
        # Cycle to next mode
        self.damper_mode = (self.damper_mode + 1) % 3
        
        if self.damper_mode == 0:
            # AUTO mode - let oscillation detector control it
            self.get_logger().info("🤖 Mode: AUTO (oscillation-based)")
            self.damper_state_publisher.publish(Int8(data=0))  # Publish current mode for monitoring
            # Don't send command - let oscillation callback handle it
            
        elif self.damper_mode == 1:
            # MANUAL ON - force damper on
            self.damper_active = True
            self.send_damper_can_command(True)
            self.get_logger().info("🟢 Mode: MANUAL ON (forced brake)")
            self.damper_state_publisher.publish(Int8(data=1))  # Publish current mode for monitoring
        elif self.damper_mode == 2:
            # MANUAL OFF - force damper off
            self.damper_active = False
            self.send_damper_can_command(False)
            self.get_logger().info("🔴 Mode: MANUAL OFF (forced coast)")
            self.damper_state_publisher.publish(Int8(data=2))  # Publish current mode for monitoring
    
    def speed_callback(self, msg: Float64):
        self.speed = msg.data
    
    def roll_callback(self, msg: Float64) -> None:
        # self.get_logger().info(f"Got roll: {msg.data}")
        roll_dict = {
                "roll": msg.data
        }
        self.serial_write(roll_dict)

        # Store roll readings for damper control
        current_time = get_time()
        self.last_roll_readings.append((current_time, msg.data))
    
    def request_tack_timer_callback(self):
        self.request_tack_override = False
        self.get_logger().info('Tack timer expired.')

        # Cancel the timer to clean up
        if self.request_tack_timer is not None:
            self.request_tack_timer.cancel()
            self.switched_sides_this_tack = False
            self.request_tack_timer = None

    def request_jibe_timer_callback(self):
        self.request_jibe_override = False
        self.get_logger().info('Jibe timer expired.')

        # Cancel the timer to clean up
        if self.request_jibe_timer is not None:
            self.request_jibe_timer.cancel()
            self.request_jibe_timer = None
            

    def request_tack_callback(self, msg: Empty) -> None:
        self.request_tack_override = True
        if self.request_tack_timer is not None:
            self.request_tack_timer.cancel()
        self.request_tack_timer = self.create_timer(self.request_tack_timer_duration, self.request_tack_timer_callback)

    def request_jibe_callback(self, msg: Float64) -> None:
        self.request_jibe_override = True
        if self.request_jibe_timer is not None:
            self.request_jibe_timer.cancel()
        self.request_jibe_timer = self.create_timer(self.request_jibe_timer_duration, self.request_jibe_timer_callback)

        trim_state_msg = TrimState()
        if(msg.data>0):
            msg = {
                "state": "max_drag_port" # Also switched due to mistake somewhere else. Fix?
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MAX_DRAG_STARBOARD
        else:
            msg = {
                "state": "max_drag_starboard"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MAX_DRAG_PORT
            
        if self.force_neutral_position and self.autonomous_mode == AutonomousMode.AUTONOMOUS_MODE_FULL:
            msg = {
                "state": "min_lift"
            }
            trim_state_msg.state = TrimState.TRIM_STATE_MIN_LIFT
            self.get_logger().info("Force neutral")
        
        if(msg is not None):
            self.trim_state_debug_publisher.publish(trim_state_msg)
            self.serial_write(msg)
        else:
            self.get_logger().info("Trim message is None, taking no action")
        
    
    def esp_heartbeat_callback(self) -> None:
        """
        Timer callback function that sends a request to the ESP32 for the current position of the ballast and handles the response.
        
        This function formats a JSON message to request the ballast position, sends it via serial communication, reads the response,
        decodes the JSON data received, and publishes the ballast position. It handles possible exceptions like serial communication
        errors or JSON decoding errors and logs them accordingly.
        
        **Process**:

        - **Request**: Sends a JSON message with a request for ballast position.
        - **Response Handling**: Attempts to read and decode the response. If successful, publishes the ballast position.
        - **Error Handling**: Captures and logs errors related to serial communication or JSON decoding.
        - **Logging**: Logs messages indicating the status of data reception and errors.
        
        **Details**:

        - The request is sent periodically, triggered by a ROS timer.
        - If no data is received, or if there is an error in the data, logs this as an warning message.
        - Even if the potentiometer is reading strangely (position is 0), the position is published to indicate that a response was received.
        
        **Example of Serial Message**:

        - Sent: '{"get_ballast_pos": True}'
        - Received: '{"ballast_pos": 102}'

        """

        message = {
            "get_heartbeat": True
        }
        self.serial_write(message)
        line = None
        try:
            line = self.ser.readline().decode('utf-8').rstrip()
        except:
            self.heartbeat_fail = self.heartbeat_fail + 1
            self.get_logger().warn("Serial Corruption")
            #serial corruption
            pass
        
        if line:
            try:
                message = json.loads(line)
                pos = Bool()
                pos.data = message["heartbeat"]
                if(pos.data != self.heartbeat):
                    self.heartbeat_fail = self.heartbeat_fail + 1
                    self.heartbeat = not(pos.data)
                else:
                    self.heartbeat = not(self.heartbeat)
                    self.heartbeat_fail = 0
            except json.JSONDecodeError:
                self.get_logger().warn("Error decoding JSON")
        else:
            self.heartbeat_fail = self.heartbeat_fail + 1
            
        if self.heartbeat_fail > 5:
            self.get_logger().warn("Restarting ESp32 node")
            self.restart = RestartNode()
            self.restart.node_name = "esp32_comms"
            response = self.restart_cli.send_request(self.restart)
            self.get_logger().info("esp32 restart: " + str(response.success) + ", " + response.message)
            self.destroy_node()
            rclpy.shutdown()
            
            
    def restart_serial(self):
        self.get_logger().warn("Restarting serial connection")

        try:
            if hasattr(self, "ser") and self.ser:
                self.ser.close()
        except Exception:
            pass

        try:
            self.ser = serial.Serial(
                serial_port,
                baud_rate,
                timeout=0.05,
                write_timeout=1.0
            )

            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            self.get_logger().info("Serial connection restarted")

        except Exception as e:
            self.get_logger().error(f"Failed reopening serial port: {e}")

    def serial_write(self, message_dict):
        message_string = json.dumps(message_dict) + '\n'

        try:
            self.ser.write(message_string.encode())
            self.last_successful_write = get_time()

        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")
            self.restart_serial()

    def serial_watchdog_callback(self):
        age = get_time() - self.last_successful_write

        if age > 5.0:
            self.get_logger().error(
                f"No successful serial writes for {age:.1f}s"
            )
            self.restart_serial()        
            
    
    def publish_error(self, string: str):
        error_msg = String()
        error_msg.data = string
        self.error_publisher.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)

    esp_comms = ESPComms()

    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        ser.close()
    except Exception as e:
        trace = traceback.format_exc()
        error_string = f'Unhandled exception: {e}\n{trace}'
        esp_comms.get_logger().fatal(error_string)
        esp_comms.publish_error(error_string)
    # Use the SingleThreadedExecutor to spin the node.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(esp_comms)

    executor.spin()
    esp_comms.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
