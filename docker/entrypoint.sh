#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers
 
# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
 
# Source the base workspace, if built
if [ -f /xarm_ws/install/setup.bash ]
then
  source /xarm_ws/install/setup.bash
fi
 
# Source the overlay workspace, if built
if [ -f /overlay_ws/install/setup.bash ]
then
  source /overlay_ws/install/setup.bash
fi
 
# Execute the command passed into this entrypoint
exec "$@"