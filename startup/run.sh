#!/bin/bash
echo "Sourcing environment"
export CTR_TARGET=Hardware
source /opt/ros/humble/setup.bash
source /usr/bin/python3
source /home/sailbot/ros2_ws/install/local_setup.bash
source /home/sailbot/sailbot24-25/sailbot_ws/install/local_setup.bash
echo "Launching server"
python3 /home/sailbot/sailbot24-25/startup/startup_server.py #/opt/sailbot/startup_server.py
