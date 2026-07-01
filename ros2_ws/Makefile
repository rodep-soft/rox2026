.PHONY: build launch 

SHELL := /bin/bash

build:
	colcon build --symlink-install

launch:
	
	source /opt/ros/humble/setup.bash && \
	source install/setup.bash && \
	ros2 run ros2socketcan_bridge ros2socketcan && \
	sleep 2 && \
	ros2 run wheel_to_can wheel_to_can_node && \
	ros2 run mecanum_controller mecanum_controller_node && \
	ros2 run mecanum_controller base_controller_node && \
	ros2 run joy joy_node


search:
	source /opt/ros/humble/setup.bash
	source install/setup.bash
	ros2 pkg executables | grep sock
