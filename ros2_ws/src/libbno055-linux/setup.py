import os
import sys
from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension, build_ext

ext_modules = [
    Pybind11Extension(
        "libbno055",
        [
            "src/python/bindings.cpp",
            "src/core/bno055.cpp",
            "src/core/bno055_c.cpp",
        ],
        include_dirs=["include"],
        cxx_std=17,
    ),
]

setup(
    name="libbno055-linux",
    version="1.9.0",
    author="lazytatzv",
    author_email="lazytatzv@users.noreply.github.com",
    url="https://github.com/lazytatzv/libbno055-linux",
    description="Python bindings for C++17 BNO055 library on Linux",
    long_description=open("README.md").read() if os.path.exists("README.md") else "",
    long_description_content_type="text/markdown",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
)
