.PHONY: build launch 

CMAKE_ARGS = \
	-G Ninja \
	-DCMAKE_C_COMPILER_LAUNCHER=ccache \
	-DCMAKE_CXX_COMPILER_LAUNCHER=ccache

build:
	CMAKE_BUILD_PARALLEL_LEVEL=$(BUILD_JOBS) \
	colcon build \
		--symlink-install \
		--cmake-args $(CMAKE_ARGS)


