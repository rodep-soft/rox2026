.PHONY: build launch 

BUILD_JOBS ?= 2
EXECUTOR ?= parallel
PARALLEL_WORKERS ?= 3

CMAKE_ARGS = \
	-G Ninja \
	-DCMAKE_C_COMPILER_LAUNCHER=ccache \
	-DCMAKE_CXX_COMPILER_LAUNCHER=ccache

build:
	CMAKE_BUILD_PARALLEL_LEVEL=$(BUILD_JOBS) \
	colcon build \
		--executor $(EXECUTOR) \
		--parallel-workers $(PARALLEL_WORKERS) \
		--event-handlers console_direct+ \
		--symlink-install \
		--cmake-args $(CMAKE_ARGS)
