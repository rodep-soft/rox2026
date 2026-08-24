.PHONY: build build-test build-package test reconfigure-ninja launch sync clean

BUILD_JOBS ?= 2
PARALLEL_WORKERS ?= 4
EXECUTOR ?= parallel

# BUILD_JOBS ?= 8
# PARALLEL_WORKERS ?= 1
# EXECUTOR ?= sequential

CMAKE_ARGS = \
	-G Ninja \
	-DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_C_COMPILER_LAUNCHER=ccache \
	-DCMAKE_CXX_COMPILER_LAUNCHER=ccache

COLCON_ARGS = \
	--executor $(EXECUTOR) \
	--parallel-workers $(PARALLEL_WORKERS) \
	--symlink-install \
	--event-handlers console_direct+

build:
	CMAKE_BUILD_PARALLEL_LEVEL=$(BUILD_JOBS) \
	colcon build \
		$(COLCON_ARGS) \
		--cmake-args $(CMAKE_ARGS) -DBUILD_TESTING=OFF

build-test:
	CMAKE_BUILD_PARALLEL_LEVEL=$(BUILD_JOBS) \
	colcon build \
		$(COLCON_ARGS) \
		--cmake-args $(CMAKE_ARGS) -DBUILD_TESTING=ON

PACKAGE ?= $(pkg)

build-package:
	@test -n "$(PACKAGE)" || (echo "Usage: make build-package PACKAGE=<package> (or pkg=<package>)" && exit 1)
	@echo "🧹 Cleaning install/$(PACKAGE) and build/$(PACKAGE)..."
	rm -rf install/$(PACKAGE) build/$(PACKAGE)
	CMAKE_BUILD_PARALLEL_LEVEL=$(BUILD_JOBS) \
	colcon build \
		$(COLCON_ARGS) \
		--packages-up-to $(PACKAGE) \
		--cmake-args $(CMAKE_ARGS) -DBUILD_TESTING=OFF

test: build-test
	colcon test --event-handlers console_cohesion+
	colcon test-result --verbose

reconfigure-ninja:
	CMAKE_BUILD_PARALLEL_LEVEL=$(BUILD_JOBS) \
	colcon build \
		$(COLCON_ARGS) \
		--cmake-clean-cache \
		--cmake-args $(CMAKE_ARGS) -DBUILD_TESTING=OFF

launch:
	ros2 launch robot_bringup manual_robot.launch.py

sync:
	vcs import --skip-existing --input src/rox2026.repos src || true
	vcs pull src || true

vcs-status:
	vcs status src

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
		tar -xzf /tmp/rox2026-dl/rox2026-install.tar.gz -C . && \
		rm -rf /tmp/rox2026-dl && \
		echo "✅ デプロイ完了 [$$BRANCH]！'source install/setup.bash' して即起動できます。"; \
	else \
		echo "❌ エラー: GitHub CLI (gh) がインストールされていません。"; \
		echo "   インストールコマンド: sudo apt update && sudo apt install -y gh && gh auth login"; \
		exit 1; \
	fi

clean:
	rm -rf build install log
