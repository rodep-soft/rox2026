# setupのためのshell script

# パッケージの更新
sudo apt update

# ツールのインストール
sudo apt install -y \
  tmux \
  fzf \
  openssh-server \
  build-essential make cmake \
  git curl unzip zip \
  python3-pip \
  libssl-dev zlib1g-dev \
  libncurses5-dev \
  device-tree-compiler \
  u-boot-tools \
  rsync \
  udev

#  SSHサーバの有効化
sudo systemctl enable ssh
sudo systemctl start ssh

echo "Setup Finished"
