# Release Guide (Maintainer Only)

This document serves as a cheat sheet for the maintainer on how to release a new version of `libbno055-linux` to both ROS 2 (`rosdistro`) and the Ubuntu PPA.

## 1. Bump the Version

Updating the version across all 7 configuration files is now fully automated! You have two options:

### Option A: Via GitHub UI (Recommended)
1. Go to **Actions ➔ Release ➔ Run workflow**.
2. Input the new version (e.g. `1.9.0`) and click **Run workflow**.
3. Actions will automatically update all configuration files, commit, tag (`v1.9.0`), create a GitHub Release, and publish to PyPI and crates.io.

### Option B: Via Local Script
Run the automated version bump script before committing:
```bash
./scripts/bump_version.sh 1.9.0
```
This updates `CMakeLists.txt`, `package.xml`, `setup.py`, `Cargo.toml`, `conanfile.py`, `vcpkg.json`, and `README.md` instantly.

Then commit and push:
```bash
git add -A
git commit -m "chore: bump version to 1.9.0"
git tag -a v1.9.0 -m "Release v1.9.0"
git push origin main --tags
```

---

## 2. ROS 2 Release (via Bloom)

To release to the official ROS 2 apt repositories, we use `bloom-release`. This automatically opens a Pull Request to `ros/rosdistro`.

**Prerequisites**: Make sure you have a GitHub Personal Access Token configured for `bloom`.

### Releasing for Humble
```bash
bloom-release --rosdistro humble --track humble libbno055_linux
```

### Releasing for Jazzy
```bash
bloom-release --rosdistro jazzy --track jazzy libbno055_linux
```

### Releasing for Kilted
```bash
bloom-release --rosdistro kilted --track kilted libbno055_linux
```

*Note: If `bloom` warns that a pull request already exists, ensure you delete the previous `bloom-libbno055_linux-X` branch from your GitHub fork of `rosdistro` before running.*

---

## 3. Ubuntu PPA Release (Standalone C++)

To distribute the pure C++ library to non-ROS users via `apt`, upload the source package to your Launchpad PPA.

**Prerequisites**: You must have `devscripts`, `debhelper`, and `dput` installed, and your GPG key configured for Launchpad.

1. **Build the Source Package**:
   ```bash
   # This will build the source package and sign it with your GPG key
   debuild -S -sa
   ```

2. **Upload to Launchpad**:
   ```bash
   # Move up one directory where the generated .changes file is located
   cd ..
   
   # Replace `lazytatzv/bno055` with your actual PPA name if different
   dput ppa:lazytatzv/bno055 libbno055-linux_X.Y.Z-1_source.changes
   ```

Launchpad will then build the `.deb` files for various architectures in the cloud and publish them to your PPA.

---

## 4. Automated `crates.io` Release (via GitHub Actions)

Pushing a Git tag matching `v*` (e.g. `git push origin v1.5.0`) triggers the automated release workflow in `.github/workflows/release.yml`.

**Prerequisites**:
1. Obtain an API Token from [crates.io](https://crates.io/settings/tokens).
2. Store the token as a GitHub Secret in your repository:
   - Go to **Settings ➔ Secrets and variables ➔ Actions**
   - Add a New repository secret named **`CRATES_IO_TOKEN`**

Once set up, whenever a tag is pushed:
* GitHub Release will be automatically generated with release notes.
* CPack `.deb` and `.tar.gz` packages will be built and attached.
* The `libbno055` Rust crate will be automatically published to `crates.io`.

