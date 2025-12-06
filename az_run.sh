#!/bin/bash

. install/setup.bash 
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=30
ros2 launch ros2_utils laptop_azzam.launch.py 