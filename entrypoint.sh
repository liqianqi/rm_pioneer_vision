#!/bin/bash
source /opt/intel/openvino_2021/bin/setupvars.sh
source /ros_ws/install/setup.bash
ros2 launch rm_vision_bringup vision_bringup.launch.py robot:=${ROBOT}