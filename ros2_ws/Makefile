.PHONY: build launch 

BUILD_JOBS ?= 4
EXECUTOR ?= sequential

CMAKE_ARGS = \
	-G Ninja \
	-DCMAKE_C_COMPILER_LAUNCHER=ccache \
	-DCMAKE_CXX_COMPILER_LAUNCHER=ccache

build:
	CMAKE_BUILD_PARALLEL_LEVEL=$(BUILD_JOBS) \
	colcon build \
		--executor $(EXECUTOR) \
		--symlink-install \
		--cmake-args $(CMAKE_ARGS)
