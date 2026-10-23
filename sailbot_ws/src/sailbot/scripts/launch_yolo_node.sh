#!/bin/bash
cd /home/sailbot/sailbot24-25
source .venv/bin/activate
source /opt/ros/humble/setup.bash
source /home/sailbot/sailbot24-25/sailbot_ws/install/setup.bash
exec python3 /home/sailbot/sailbot24-25/sailbot_ws/src/sailbot/sailbot/buoy_detection_yolo.py "$@"