#!/bin/bash

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Setup Turtle Bot
source /turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

# Robot Specific
export ROS_DOMAIN_ID=53

# Execute Command
exec "$@"