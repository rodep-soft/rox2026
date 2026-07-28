#!/usr/bin/env bash
set -Eeuo pipefail

if [ "$EUID" -eq 0 ]; then
  echo "Do not run this script with sudo." >&2
  echo "Run: ./setup.sh" >&2
  exit 1
fi
echo "RDK X5 initial setup start."

sudo apt update
sudo apt install -y \
  ca-certificates \
  gnupg \
  git \
  vim \
  curl \
  wget \
  unzip \
  zip \
  ccache \
  tree \
  tmux \
  neovim \
  htop \
  lsof \
  fzf \
  openssh-server \
  build-essential cmake \
  ninja-build \
  pkg-config \
  python3-pip \
  python3-colcon-common-extensions \
  python3-vcstool \
  python3-argcomplete \
  python3-rosdep \
  iputils-ping \
  iproute2 \
  usbutils \
  can-utils \
  device-tree-compiler \
  bluez bluez-tools

#ca~~  :https認証用
#gpg   :GPG署名の確認用
#git   :gitコマンド用
#vim   :エディタ
#curl  :API通信経由でのファイル取得用
#wget  :大きなファイル取得用
#unzip :zip解答
#zip   :zip作成
#ccache:CMakeのビルド高速化
#tree  :ディレクトリ構造をtree経由で表示
#tmux  :画面分割用 ssh切断後もROSノードを動かし続けられる
#neovim:vimの拡張版のエディタ
#htop  :CPU使用率監視用
#lsof  :どのプロセスがが何を使用しているかを表示するツール
#fzf   :あいまい検索用ツール
#openssh-server       : ssh用
#build-essential cmake:gcc g++ makeなどのツールとCmake用のツール
#ninja-build          :CMakeと組み合わせて高速化できるツール
#pkg-config           :ライブラリのヘッダやリンクを自動的に取得できるツール
#python3-pip          :pythonパッケージ管理用ツール pip installなど
#python3-colcon-common-extensions :colconの拡張機能をまとめたパッケージ
#python3-vcstool      :複数のgitrepoを一括取得・変更用
#python3-argcomplete  :bashでros2コマンドのタブ補完有効化
#python3-rosdep       :ros2パッケージの依存ライブラリを自動インストール
#iputils-ping         :pingコマンド用
#iproute2             :ipコマンド用
#usbutils             :lsusbコマンド用
#can-utils            :candumpなど用
#device-tree-compiler :dtcコマンド用

#  SSHサーバの有効化
sudo systemctl enable --now ssh
if sudo systemctl is-active --quiet ssh; then
  echo "SSH server is active."
else
  echo "Error: SSH server failed to start." >&2
  exit 1
fi

# CAN
# ====================================================
CAN_INTERFACE="can0"
CAN_BITRATE="1000000"
CAN_RESTART_MS="100"
CAN_TX_QUEUE_LEN="1024"

sudo tee /usr/local/bin/can_setup.sh > /dev/null <<EOF
#!/usr/bin/env bash
set -Eeuo pipefail

INTERFACE="${CAN_INTERFACE}"
BITRATE="${CAN_BITRATE}"
RESTART_MS="${CAN_RESTART_MS}"
TX_QUEUE_LEN="${CAN_TX_QUEUE_LEN}"

for i in {1..20}; do
  if ip link show "\${INTERFACE}" > /dev/null 2>&1; then
    break
  fi
  sleep 0.5
done

if ! ip link show "\${INTERFACE}" > /dev/null 2>&1; then
  echo "CAN interface \${INTERFACE} was not found." >&2
  exit 1
fi

ip link set "\${INTERFACE}" down 2>/dev/null || true

ip link set "\${INTERFACE}" type can \
  bitrate "\${BITRATE}" \
  restart-ms "\${RESTART_MS}"

ip link set "\${INTERFACE}" txqueuelen "\${TX_QUEUE_LEN}"
ip link set "\${INTERFACE}" up
EOF

sudo chmod 755 /usr/local/bin/can_setup.sh

sudo tee /etc/systemd/system/can0.service > /dev/null <<'EOF'
[Unit]
Description=Configure SocketCAN interface can0
After=network-pre.target
Wants=network-pre.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/can_setup.sh
ExecStop=/sbin/ip link set can0 down
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now can0.service

if sudo systemctl is-active --quiet can0.service; then
  echo "CAN interface setup service is active."
else
  echo "Error: CAN interface setup failed." >&2
  sudo systemctl status can0.service --no-pager || true
  exit 1
fi
#=======================================================
ROS_SETUP="/opt/ros/humble/setup.bash"
if [ ! -f "$ROS_SETUP" ]; then
  echo "Error: ROS 2 Humble is not installed." >&2
  echo "Missing: $ROS_SETUP" >&2
  exit 1
fi
source "$ROS_SETUP"

# rosdepの初期化
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

#bashrcへ登録
BASHRC="$HOME/.bashrc"
CONFIG_BEGIN="# >>> RDK X5 setup >>>"
if ! grep -Fq "$CONFIG_BEGIN" "$BASHRC"; then
  cat >>"$BASHRC" <<'EOF'
# >>> RDK X5 setup >>>


if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "$HOME/rox2026/ros2_ws/install/setup.bash" ]; then
  source "$HOME/rox2026/ros2_ws/install/setup.bash"
fi
export CCACHE_DIR="$HOME/.ccache"

alias canshow='ip -details -statistics link show can0'
alias canrestart='sudo systemctl restart can0.service'
alias canstatus='systemctl status can0.service'

EOF
fi

# rox2026ワークスペースが存在する場合は依存関係を導入
ROS2_WS="$HOME/rox2026/ros2_ws"
if [ -d "$ROS2_WS/src" ]; then
  echo "Installing workspace dependencies..."

  source /opt/ros/humble/setup.bash

  rosdep install \
    --from-paths "$ROS2_WS/src" \
    --ignore-src \
    --rosdistro humble \
    -r \
    -y
fi

echo "Setup Finished"
echo "Run the following command to apply bashrc:"
echo "source ~/.bashrc"
