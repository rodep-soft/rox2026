#!/usr/bin/env bash
set -e

if [ -z "$1" ]; then
    echo "Usage: ./scripts/bump_version.sh <new_version>"
    echo "Example: ./scripts/bump_version.sh 1.9.0"
    exit 1
fi

NEW_VER="$1"
# Strip leading 'v' if provided
NEW_VER="${NEW_VER#v}"

# Get current version from CMakeLists.txt
OLD_VER=$(grep -E 'project\(libbno055_linux VERSION' CMakeLists.txt | sed -E 's/.*VERSION ([0-9]+\.[0-9]+\.[0-9]+).*/\1/')

if [ -z "$OLD_VER" ]; then
    echo "Error: Could not extract current version from CMakeLists.txt"
    exit 1
fi

if [ "$OLD_VER" == "$NEW_VER" ]; then
    echo "Version is already $NEW_VER"
    exit 0
fi

echo "Bumping version: $OLD_VER -> $NEW_VER"

# 1. CMakeLists.txt
sed -i "s/project(libbno055_linux VERSION ${OLD_VER}/project(libbno055_linux VERSION ${NEW_VER}/" CMakeLists.txt

# 2. package.xml
sed -i "s/<version>${OLD_VER}<\/version>/<version>${NEW_VER}<\/version>/" package.xml

# 3. setup.py
sed -i "s/version=\"${OLD_VER}\"/version=\"${NEW_VER}\"/" setup.py

# 4. rust/Cargo.toml & rust/Cargo.lock
sed -i "s/version = \"${OLD_VER}\"/version = \"${NEW_VER}\"/" rust/Cargo.toml
if [ -f rust/Cargo.lock ]; then
    sed -i "s/version = \"${OLD_VER}\"/version = \"${NEW_VER}\"/" rust/Cargo.lock
fi

# 5. conanfile.py
sed -i "s/version = \"${OLD_VER}\"/version = \"${NEW_VER}\"/" conanfile.py

# 6. vcpkg.json
sed -i "s/\"version\": \"${OLD_VER}\"/\"version\": \"${NEW_VER}\"/" vcpkg.json

# 7. README.md
sed -i "s/version-${OLD_VER}-green/version-${NEW_VER}-green/" README.md
sed -i "s/version (v${OLD_VER})/version (v${NEW_VER})/" README.md
sed -i "s/release (v${OLD_VER})/release (v${NEW_VER})/" README.md
sed -i "s/version: v${OLD_VER}/version: v${NEW_VER}/" README.md

echo "All version fields updated to $NEW_VER."
