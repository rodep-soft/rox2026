#!/usr/bin/env bash
set -euo pipefail

if ! command -v apt-get >/dev/null 2>&1; then
  echo "This script currently targets Debian/Ubuntu systems with apt-get."
  exit 1
fi

# Install Neovim
sudo apt-get update
sudo apt-get install -y software-properties-common
sudo add-apt-repository ppa:neovim-ppa/unstable -y
sudo apt-get update
sudo apt-get install -y neovim

# Install tools for LazyVim
sudo apt-get install -y fd-find fzf ripgrep git curl unzip

# Enable fd command
mkdir -p "$HOME/.local/bin"

if command -v fdfind >/dev/null 2>&1 && [ ! -e "$HOME/.local/bin/fd" ]; then
  ln -s "$(command -v fdfind)" "$HOME/.local/bin/fd"
fi

# Add ~/.local/bin to PATH
if ! grep -q 'export PATH="$HOME/.local/bin:$PATH"' "$HOME/.bashrc"; then
  echo 'export PATH="$HOME/.local/bin:$PATH"' >>"$HOME/.bashrc"
fi

export PATH="$HOME/.local/bin:$PATH"

# Show versions
echo "===== installed versions ====="
nvim --version | head -n 1
fd --version
fzf --version
rg --version
echo "=============================="
echo "Setup finished. If fd command is not found, run: source ~/.bashrc"
