.PHONY: build build-test build-package test reconfigure-ninja launch sync clean

BUILD_JOBS ?= 3
PARALLEL_WORKERS ?= 2
EXECUTOR ?= parallel

CMAKE_ARGS = \
	-G Ninja \
	-DCMAKE_C_COMPILER_LAUNCHER=ccache \
	-DCMAKE_CXX_COMPILER_LAUNCHER=ccache

COLCON_ARGS = \
	--executor $(EXECUTOR) \
	--parallel-workers $(PARALLEL_WORKERS) \
	--symlink-install

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

build-package:
	@test -n "$(PACKAGE)" || (echo "Usage: make build-package PACKAGE=<package>" && exit 1)
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
	vcs import --input src/rox2026.repos src

vcs-status:
	vcs status src

clean:
	rm -rf build install log