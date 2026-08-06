# Contributing to libbno055-linux

First off, thank you for considering contributing to `libbno055-linux`! 

## Setting up your Development Environment

1. **Clone the repository**:
   ```bash
   git clone https://github.com/lazytatzv/libbno055-linux.git
   cd libbno055-linux
   ```
2. **Install CMake and a C++17 compiler**:
   - On Ubuntu: `sudo apt install build-essential cmake`
3. **Build the project**:
   ```bash
   mkdir build && cd build
   cmake -DBUILD_TESTING=ON -DENABLE_CLANG_TIDY=ON ..
   make
   ```
4. **Run tests**:
   ```bash
   ctest --output-on-failure
   ```

## Pull Request Process

1. Update the README.md or docs with details of changes to the interface if applicable.
2. Make sure all tests pass locally. If you add new functionality, please add a test for it!
3. Format your code using `clang-format` and check with `clang-tidy` (configs are provided in the repo).
4. Create a Pull Request against the `main` branch.

## Reporting Bugs

Please use the provided Bug Report template in the Issues tab. Ensure you include:
* OS Version
* BNO055 wiring / hardware setup
* Library version (or git commit)
* Minimal reproducible example

## Suggesting Enhancements

Please use the Feature Request template in the Issues tab and provide as much detail as possible about why the feature is needed and how it should work.
