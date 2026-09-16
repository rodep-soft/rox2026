.PHONY: dev sync build build-pkg launch-manual launch-game2 launch-game3 launch-pk test-apriltag test-ekf debug can-check help

# Docker & Workspace
dev:
	docker compose up -d
	docker compose exec ros2_rox2026 bash

sync:
	git submodule update --init --recursive
	vcs import ros2_ws/src < ros2_ws/src/rox2026.repos || true
	vcs custom ros2_ws/src --git --args pull origin main || (cd ros2_ws/src/libbno055-linux && git pull origin main)

build:
	$(MAKE) -C ros2_ws build

build-pkg:
	@if [ -z "$(pkg)" ]; then \
		echo "Error: パッケージ名を指定してください"; \
		echo "   例: make build-pkg pkg=libbno055_linux"; \
		exit 1; \
	fi
	$(MAKE) -C ros2_ws build-package PACKAGE=$(pkg)

# トラブルシューティングと診断
debug:
	bash ros2_ws/src/robot_bringup/scripts/debug_robot.sh

can-check:
	python3 ros2_ws/src/robot_bringup/scripts/can_health_check.py

# 競技用launch
launch-manual:
	cd ros2_ws && . install/setup.bash && ros2 launch robot_bringup manual_robot.launch.py

launch-game2:
	cd ros2_ws && . install/setup.bash && ros2 launch robot_bringup game2_auto.launch.py

launch-game3:
	cd ros2_ws && . install/setup.bash && ros2 launch robot_bringup game3_robot.launch.py

launch-pk:
	cd ros2_ws && . install/setup.bash && ros2 launch robot_bringup pk_auto.launch.py

# センサー確認
test-apriltag:
	cd ros2_ws && . install/setup.bash && ros2 launch robot_bringup apriltag_launch.py

test-ekf:
	cd ros2_ws && . install/setup.bash && ros2 launch robot_bringup ekf.launch.py

help:
	@echo "=== ROX2026 ROS2 Command Shortcuts ==="
	@echo "  make build                        : Build entire ROS2 workspace"
	@echo "  make build-pkg pkg=<name>         : Build a specific package only"
	@echo "  make debug                        : Reset FastDDS/CAN and run full robot diagnosis"
	@echo "  make can-check                    : Run pro-level CAN bus & node health check"
	@echo "  make launch-manual                : Launch manual control mode"
	@echo "  make launch-game2                 : Launch robot with Game2 Auto mode"
	@echo "  make launch-game3                 : Launch Game3 manual configuration"
	@echo "  make launch-pk                    : Launch PK Auto mode"
	@echo "  make test-apriltag                : Test AprilTag detection"
	@echo "  make test-ekf                     : Test EKF sensor fusion"
