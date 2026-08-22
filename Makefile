.PHONY: dev sync build launch-robot launch-manual test-cam test-apriltag test-yolo test-imu test-ekf launch-game1 launch-game2 debug can-check help

# Docker & Workspace
dev:
	docker compose up -d
	docker compose exec ros2_rox2026 bash

sync:
	git submodule update --init --recursive
	vcs import --skip-existing ros2_ws/src < ros2_ws/src/rox2026.repos || true
	vcs pull ros2_ws/src || true

build:
	$(MAKE) -C ros2_ws build

# ── ⚡ CIビルド済みバイナリのワンクリックダウンロード ──
download:
	@BRANCH=$$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "main"); \
	echo "⬇️ GitHub Actions からブランチ [$$BRANCH] の最新ビルド済みバイナリを取得中..."; \
	if command -v gh > /dev/null 2>&1; then \
		rm -rf /tmp/rox2026-dl; \
		RUN_ID=$$(gh run list --branch "$$BRANCH" --limit 1 --json databaseId -q '.[0].databaseId' 2>/dev/null); \
		if [ -n "$$RUN_ID" ]; then \
			gh run download "$$RUN_ID" -n rox2026-install -D /tmp/rox2026-dl; \
		else \
			gh run download -n rox2026-install -D /tmp/rox2026-dl; \
		fi && \
		tar -xzf /tmp/rox2026-dl/rox2026-install.tar.gz -C ros2_ws && \
		rm -rf /tmp/rox2026-dl && \
		echo "✅ デプロイ完了 [$$BRANCH]！'source ros2_ws/install/setup.bash' して即起動できます。"; \
	else \
		echo "❌ エラー: GitHub CLI (gh) がインストールされていません。"; \
		echo "   インストールコマンド: sudo apt update && sudo apt install -y gh && gh auth login"; \
		exit 1; \
	fi

# ── 📦 パッケージ個別・クリーンビルド ──
clean-build:
	@if [ -z "$(pkg)" ]; then \
		echo "❌ エラー: パッケージ名を指定してください"; \
		echo "   例: make clean-build pkg=robot_controller"; \
		echo "   例: make clean-build pkg=robot_bringup"; \
		exit 1; \
	fi
	@echo "🧹 クリーンビルド中: $(pkg)..."
	rm -rf ros2_ws/build/$(pkg) ros2_ws/install/$(pkg)
	$(MAKE) -C ros2_ws build-package PACKAGE=$(pkg)

build-pkg:
	@if [ -z "$(pkg)" ]; then \
		echo "❌ エラー: パッケージ名を指定してください"; \
		echo "   例: make build-pkg pkg=robot_controller"; \
		exit 1; \
	fi
	$(MAKE) -C ros2_ws build-package PACKAGE=$(pkg)

# ── 🛠️ トラブルシューティング＆診断一発起動 ──
debug:
	bash ros2_ws/src/robot_bringup/scripts/debug_robot.sh

can-check:
	python3 ros2_ws/src/robot_bringup/scripts/can_health_check.py

bench-cpu:
	python3 ros2_ws/src/robot_bringup/scripts/cpu_benchmark.py $(sec)

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
	@echo "  make build                        : Build entire ROS2 workspace"
	@echo "  make build-pkg pkg=<name>         : Build a specific package only"
	@echo "  make clean-build pkg=<name>       : Clean and rebuild a specific package only"
	@echo "  make debug                        : Reset FastDDS/CAN and run full robot diagnosis"
	@echo "  make can-check                    : Run pro-level CAN bus & node health check"
	@echo "  make launch-robot                 : Launch all main robot nodes"
	@echo "  make launch-manual                : Launch manual control mode"
	@echo "  make launch-game1                 : Launch robot with Game1 Auto mode"
	@echo "  make launch-game2                 : Launch robot with Game2 Auto mode"
	@echo "  make test-cam                     : Test webcam stream (/image_raw)"
	@echo "  make test-apriltag                : Test AprilTag detection"
	@echo "  make test-yolo                    : Test YOLO ball detection"
	@echo "  make test-imu                     : Test BNO055 IMU sensor"
	@echo "  make test-ekf                     : Test EKF sensor fusion"

