.PHONY: dev sync build launch-robot launch-manual test-cam test-apriltag test-yolo test-imu test-ekf launch-game1 launch-game2 debug can-check help

# Docker & Workspace
dev:
	docker compose up -d
	docker compose exec ros2_rox2026 bash

sync:
	git submodule update --init --recursive
	vcs import ros2_ws/src < ros2_ws/src/rox2026.repos

build:
	cd ros2_ws && colcon build --symlink-install --event-handlers console_direct- status+ console_stderr+

# ── 🛠️ トラブルシューティング＆診断一発起動 ──
debug:
	bash ros2_ws/src/robot_bringup/scripts/debug_robot.sh

can-check:
	python3 ros2_ws/src/robot_bringup/scripts/can_health_check.py

# ── 🚀 ロボット本番起動ショートカット ──
launch-robot:
	cd ros2_ws && . install/setup.bash && ros2 launch robot_bringup robot.launch.py

launch-manual:
	cd ros2_ws && . install/setup.bash && ros2 launch robot_bringup manual_robot.launch.py

launch-game1:
	cd ros2_ws && . install/setup.bash && ros2 launch robot_bringup robot.launch.py enable_game1:=true

launch-game2:
	cd ros2_ws && . install/setup.bash && ros2 launch robot_bringup robot.launch.py enable_game2:=true

# ── 🧪 センサー・単体テスト一発起動 ──
test-cam:
	cd ros2_ws && . install/setup.bash && ros2 launch robot_bringup webcam_launch.py

test-apriltag:
	cd ros2_ws && . install/setup.bash && ros2 launch robot_bringup apriltag_launch.py

test-yolo:
	cd ros2_ws && . install/setup.bash && ros2 launch robot_bringup vision_launch.py

test-imu:
	cd ros2_ws && . install/setup.bash && ros2 launch robot_bringup bno055.launch.py

test-ekf:
	cd ros2_ws && . install/setup.bash && ros2 launch robot_bringup ekf.launch.py

help:
	@echo "=== ROX2026 ROS2 Command Shortcuts ==="
	@echo "  make debug          : Reset FastDDS/CAN and run full robot diagnosis"
	@echo "  make can-check      : Run pro-level CAN bus & node health check"
	@echo "  make build          : Build ROS2 workspace with symlink-install"
	@echo "  make launch-robot   : Launch all main robot nodes"
	@echo "  make launch-manual  : Launch manual control mode"
	@echo "  make launch-game1   : Launch robot with Game1 Auto mode"
	@echo "  make launch-game2   : Launch robot with Game2 Auto mode"
	@echo "  make test-cam       : Test webcam stream (/image_raw)"
	@echo "  make test-apriltag  : Test AprilTag detection"
	@echo "  make test-yolo      : Test YOLO ball detection"
	@echo "  make test-imu       : Test BNO055 IMU sensor"
	@echo "  make test-ekf       : Test EKF sensor fusion"

