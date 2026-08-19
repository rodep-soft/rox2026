#!/usr/bin/env bash
# bloom_release_all.sh
# Runs bloom-release for Humble, Jazzy, Kilted, and Lyrical in sequence.
# Iron is EOL (November 2024) and intentionally skipped.
#
# Prerequisites:
#   sudo apt install python3-bloom python3-catkin-pkg
#
# Usage:
#   bash bloom_release_all.sh
#
# Bloom will interactively ask for GitHub credentials on the first run.
# After that it caches them automatically.

set -e

PACKAGE="libbno055_linux"
DISTROS=("humble" "jazzy" "kilted" "lyrical")

echo "=================================================="
echo " bloom-release: ${PACKAGE}"
echo " Targets: ${DISTROS[*]}"
echo " (Iron skipped — EOL November 2024)"
echo "=================================================="
echo ""

for DISTRO in "${DISTROS[@]}"; do
    echo "--------------------------------------------------"
    echo " Releasing to: ${DISTRO}"
    echo "--------------------------------------------------"
    bloom-release "${PACKAGE}" --rosdistro "${DISTRO}" --track "${DISTRO}"
    echo ""
    echo "[OK] ${DISTRO} done."
    echo ""
done

echo "=================================================="
echo " All distros released successfully!"
echo " Check your release repository and open PRs to"
echo " ros/rosdistro if prompted by bloom."
echo "=================================================="
