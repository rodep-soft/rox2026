.PHONY: build launch 

BUILD_JOBS ?= 3
EXECUTOR ?= parallel
PARALLEL_WORKERS ?= 2

CMAKE_ARGS = \
	-G Ninja \
	-DCMAKE_C_COMPILER_LAUNCHER=ccache \
	-DCMAKE_CXX_COMPILER_LAUNCHER=ccache

build:
	CMAKE_BUILD_PARALLEL_LEVEL=$(BUILD_JOBS) \
	colcon build \
		--executor $(EXECUTOR) \
		--parallel-workers $(PARALLEL_WORKERS) \
		--symlink-install \
		--cmake-args $(CMAKE_ARGS)
