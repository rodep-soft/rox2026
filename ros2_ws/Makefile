.PHONY: build launch clean

SHELL := /bin/bash

build:
	colcon build --symlink-install

clean:
	rm -rf install/ log/ build/

launch:

	source install/setup.bash && \
	ros2 launch robot_bringup robot.launch.py


search:
	source /opt/ros/humble/setup.bash
	source install/setup.bash
	ros2 pkg executables | grep sock
