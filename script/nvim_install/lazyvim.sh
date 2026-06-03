#!/usr/bin/env bash
set -euo pipefail

# Install LazyVim starter and overlay this repo's config.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

backup_if_exists() {
  local src="$1"
  if [ -e "$src" ]; then
    mv -- "$src" "${src}.bak-${TIMESTAMP}"
  fi
}

# Clone the starter first so we do not disturb an existing config if this fails.
git clone https://github.com/LazyVim/starter "$TMP_DIR/nvim"
rm -rf "$TMP_DIR/nvim/.git"

# Back up old Neovim data directories before replacing them.
backup_if_exists "$HOME/.config/nvim"
backup_if_exists "$HOME/.local/share/nvim"
backup_if_exists "$HOME/.local/state/nvim"
backup_if_exists "$HOME/.cache/nvim"

echo "LazyVim installed. Run: nvim"
echo "But config is submodule"
echo "So, you take the repository"
