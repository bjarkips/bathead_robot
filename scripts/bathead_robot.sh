#!/bin/sh

# Before running this, ROS_MASTER_URI must be set up according to ros master's IP
#  and roscore should be running on the ros master.

# Start snapshot program
sudo gain ext 120
nohup sudo /usr/sbin/snapshot -v -v -v --bps=16 -b 32 -u pi -g pi -g iocard -R 5 -W 4 -S /tmp --allow-path-reuse=1 $* &
sleep 1s
snapchat -m p i g z

# Start ROS
nohup rosrun bathead_robot bathead_range_node.py &
rosrun rosaria RosAria &
sleep 10s
rosrun bathead_robot bathead_control_node
