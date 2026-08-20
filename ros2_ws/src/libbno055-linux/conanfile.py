from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout

class LibBNO055LinuxConan(ConanFile):
    name = "libbno055-linux"
    version = "1.9.0"
    license = "MIT"
    author = "lazytatzv"
    url = "https://github.com/lazytatzv/libbno055-linux"
    description = "C++17 BNO055 library and ROS 2 nodes for Linux."
    topics = ("bno055", "imu", "i2c", "linux", "robotics")
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}
    exports_sources = "CMakeLists.txt", "src/*", "include/*", "cmake/*"

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = ["bno055-linux"]
