#!/usr/bin/env bash
set -Eeuo pipefail

# shが実行できるか確認
if [ "$EUID" -eq 0 ]; then
  echo "Do not run this script with sudo." >&2
  echo "Run: ./setup.sh" >&2
  exit 1
fi

# RDKのログインパスワードの設定
read -rp "Change login/SSH password now? [y/N]: " PASS_CONFIRM
if [[ "$PASS_CONFIRM" =~ ^[Yy]$ ]]; then
  passwd
else
  echo "Skipping password change."
fi

# Wi-Fiの接続
# すでにWi-Fi接続済みならそのまま使用する。
# 未接続の場合のみ、既存のNetworkManager接続プロファイルまたは任意SSIDへ接続する。
echo
WIFI_DEV="$(LC_ALL=C nmcli -t -f DEVICE,TYPE device status | awk -F: '$2 == "wifi" {print $1; exit}')"
WIFI_CONNECTED_DEV="$(LC_ALL=C nmcli -t -f DEVICE,TYPE,STATE device status | awk -F: '$2 == "wifi" && $3 == "connected" {print $1; exit}')"

if [ -n "$WIFI_CONNECTED_DEV" ]; then
  WIFI_CONNECTION="$(nmcli -g GENERAL.CONNECTION device show "$WIFI_CONNECTED_DEV" 2>/dev/null || true)"
  echo "Wi-Fi is already connected. Skipping Wi-Fi setup."
  echo "  device:     $WIFI_CONNECTED_DEV"
  echo "  connection: ${WIFI_CONNECTION:-unknown}"
elif [ -z "$WIFI_DEV" ]; then
  echo "Warning: No Wi-Fi device was found. Skipping Wi-Fi setup." >&2
else
  echo "Wi-Fi is not connected."
  echo
  echo "Saved NetworkManager connections:"
  nmcli -f NAME,TYPE connection show | sed -n '1p;/wifi/p' || true
  echo
  echo "Nearby Wi-Fi networks:"
  sudo nmcli device wifi rescan ifname "$WIFI_DEV" >/dev/null 2>&1 || true
  nmcli -f IN-USE,SSID,SIGNAL,SECURITY device wifi list ifname "$WIFI_DEV" || true
  echo

  read -rp "Wi-Fi connection profile name or SSID (blank to skip): " WIFI_TARGET

  if [ -z "$WIFI_TARGET" ]; then
    echo "Skipping Wi-Fi connection."
  elif nmcli -t -f NAME connection show | grep -Fxq "$WIFI_TARGET"; then
    echo "Using saved NetworkManager connection: $WIFI_TARGET"
    sudo nmcli connection up id "$WIFI_TARGET" ifname "$WIFI_DEV"
  else
    read -rsp "Wi-Fi password (blank for an open network): " WIFI_PASS
    echo
    if [ -n "$WIFI_PASS" ]; then
      sudo nmcli device wifi connect "$WIFI_TARGET" password "$WIFI_PASS" ifname "$WIFI_DEV"
    else
      sudo nmcli device wifi connect "$WIFI_TARGET" ifname "$WIFI_DEV"
    fi
  fi
fi

# eth0の設定
if nmcli connection show eth0-static >/dev/null 2>&1; then
    echo "Updating eth0-static profile..."
    sudo nmcli connection modify eth0-static \
        ipv4.method manual \
        ipv4.addresses 192.168.127.10/24 \
        ipv6.method ignore \
        ipv4.gateway "" \
        ipv4.dns "" \
        ipv4.never-default yes \
        connection.autoconnect yes
else
    echo "Creating eth0-static profile..."
    sudo nmcli connection add \
        type ethernet \
        ifname eth0 \
        con-name eth0-static \
        ipv4.method manual \
        ipv4.addresses 192.168.127.10/24 \
        ipv6.method ignore \
        ipv4.gateway "" \
        ipv4.dns "" \
        ipv4.never-default yes \
        autoconnect yes
fi

# 各種パッケージのインストールとアップデート
echo "RDK X5 initial setup start."
sudo apt update
sudo apt install -y \
  ca-certificates \
  gnupg \
  vim \
  git \
  gh \
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
  python3-yaml \
  python3-colcon-common-extensions \
  python3-vcstool \
  python3-argcomplete \
  python3-rosdep \
  iputils-ping \
  iproute2 \
  usbutils \
  can-utils \
  device-tree-compiler \
  bluez bluez-tools \
  v4l-utils \
  ros-humble-v4l2-camera \
  ros-humble-apriltag-ros

sudo apt install --reinstall -y ros-humble-tf2-ros

# ====================================================
# 起動時間の高速化
# ====================================================
# ロボット運用中にAPTの自動処理が起動してブート完了を遅らせないようにする。
# apt コマンドによる手動の update / upgrade は通常どおり使用できる。
echo "Optimizing boot services..."

for unit in \
  apt-show-versions.timer \
  apt-daily.timer \
  apt-daily-upgrade.timer; do
  if systemctl list-unit-files --no-legend "$unit" 2>/dev/null | grep -q "^${unit}"; then
    sudo systemctl disable --now "$unit" >/dev/null 2>&1 || true
  fi
done

for unit in \
  apt-show-versions.service \
  apt-daily.service \
  apt-daily-upgrade.service; do
  if systemctl list-unit-files --no-legend "$unit" 2>/dev/null | grep -q "^${unit}"; then
    sudo systemctl mask "$unit" >/dev/null 2>&1 || true
  fi
done

# RDK標準の hobot-rc は /app 以下を1ファイルずつ stat/chgrp するため、
# ファイル数が多い環境では起動に数秒以上かかる。
# find 自身にgroup判定をさせ、chgrpをまとめて実行する形へ置換する。
HOBOT_RC="/etc/init.d/hobot-rc"
HOBOT_RC_BACKUP="/etc/init.d/hobot-rc.before-rox2026-setup"

if [ -f "$HOBOT_RC" ]; then
  if [ ! -e "$HOBOT_RC_BACKUP" ]; then
    sudo cp -a "$HOBOT_RC" "$HOBOT_RC_BACKUP"
    echo "Backed up hobot-rc to: $HOBOT_RC_BACKUP"
  fi

  sudo python3 - "$HOBOT_RC" <<'PY_HOBOT_RC'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text()
original = text

# 検証時に追加した計測ログが残っている場合は除去する。
text = re.sub(
    r'^\s*echo\s+"\[hobot-rc\].*?"\s*$',
    '',
    text,
    flags=re.MULTILINE,
)

# change_grp() を高速版へ置換する。すでに高速版なら何もしない。
if '-group root -o -group sudo' not in text:
    pattern = re.compile(
        r'^change_grp\(\)\s*\n\{\s*\n.*?^\}\s*$',
        flags=re.MULTILINE | re.DOTALL,
    )
    replacement = '''change_grp()
{
        local DIR="${1}"
        local group_name="${2}"

        find "$DIR" -depth \\
                \\( -type f -o -type d \\) \\
                \\( -group root -o -group sudo \\) \\
                -exec chgrp "$group_name" {} +
}'''
    text, count = pattern.subn(lambda _m: replacement, text, count=1)
    if count != 1:
        raise SystemExit('Error: could not locate change_grp() in /etc/init.d/hobot-rc')

# X5上にiar_test_attrが無い場合の不要なエラーを防ぐ。
old_iar = '[[ -d /etc/lightdm ]] && echo desktop > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr'
new_iar = '''if [ -d /etc/lightdm ] && [ -e /sys/devices/virtual/graphics/iar_cdev/iar_test_attr ]; then
                echo desktop > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
        fi'''
if old_iar in text:
    text = text.replace(old_iar, new_iar, 1)

# /usr/bin/python3.8 が存在しないRDK X5で getcap のエラーを出さない。
old_py = '''python_path="/usr/bin/python3.8"
        desired_caps="cap_sys_rawio,cap_sys_nice+eip"
        if getcap "$python_path" | grep -q "$desired_caps"; then
                setcap -r "$python_path"
        fi'''
new_py = '''python_path="/usr/bin/python3.8"
        desired_caps="cap_sys_rawio,cap_sys_nice+eip"
        if [ -x "$python_path" ] && getcap "$python_path" | grep -q "$desired_caps"; then
                setcap -r "$python_path"
        fi'''
if old_py in text:
    text = text.replace(old_py, new_py, 1)

if text != original:
    path.write_text(text)
    print('Optimized /etc/init.d/hobot-rc')
else:
    print('/etc/init.d/hobot-rc is already optimized')
PY_HOBOT_RC

  sudo chmod 755 "$HOBOT_RC"
else
  echo "Warning: $HOBOT_RC was not found. Skipping hobot-rc optimization." >&2
fi

# SysV generator / systemdへ変更を反映する。
sudo systemctl daemon-reload

echo "Boot optimization finished."


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
#bluez                :bluetoothの使用

# SSHサーバの有効化
sudo systemctl enable --now ssh
if sudo systemctl is-active --quiet ssh; then
  echo "SSH server is active."
else
  echo "Error: SSH server failed to start." >&2
  exit 1
fi
# bluetoothの有効化
sudo systemctl enable --now bluetooth
if sudo systemctl is-active --quiet bluetooth; then
  echo "Bluetooth service is active."
else
  echo "Warning: Bluetooth service failed to start." >&2
fi

#githubのレポジトリのクローン
# GitHub authentication and repository clone
# ====================================================
REPO_URL="git@github.com:rodep-soft/rox2026.git"
REPO_DIR="$HOME/rox2026"
REPO_BRANCH="main-v2"
echo
echo "GitHub setup start."
# GitHub CLIの認証状態を確認
if gh auth status --hostname github.com >/dev/null 2>&1; then
  echo "GitHub CLI is already authenticated."
else
  echo "GitHub authentication is required."
  echo "Follow the instructions displayed below."
  gh auth login \
    --hostname github.com
fi

# 認証確認
if ! gh auth status --hostname github.com >/dev/null 2>&1; then
  echo "Error: GitHub authentication failed." >&2
  exit 1
fi
echo "GitHub authentication successful."
# Git操作でSSHを使うよう設定
gh config set git_protocol ssh --host github.com
# GitHubとのSSH接続確認
SSH_OUTPUT="$(ssh \
  -o StrictHostKeyChecking=accept-new \
  -T git@github.com 2>&1 || true)"

if printf '%s\n' "$SSH_OUTPUT" | grep -q "successfully authenticated"; then
  echo "GitHub SSH connection successful."
else
  echo "$SSH_OUTPUT" >&2
  echo "Error: GitHub SSH connection failed." >&2
  exit 1
fi
# リポジトリをクローン
if [ -d "$REPO_DIR/.git" ]; then
  echo "Repository already exists:"
  echo "  $REPO_DIR"
  echo "Skipping clone."
elif [ -e "$REPO_DIR" ]; then
  echo "Error: $REPO_DIR exists but is not a Git repository." >&2
  exit 1
else
  echo "Cloning repository..."
  git clone \
    --branch "$REPO_BRANCH" \
    "$REPO_URL" \
    "$REPO_DIR"
  echo "Repository cloned successfully:"
  echo "  $REPO_DIR"
fi

# Tailscaleのインストール
# ====================================================
if ! command -v tailscale >/dev/null 2>&1; then
    echo "Installing Tailscale..."
    curl -fsSL https://tailscale.com/install.sh | sh
else
    echo "Tailscale is already installed."
fi
sudo systemctl enable --now tailscaled
if sudo systemctl is-active --quiet tailscaled; then
    echo "Tailscale service is active."
else
    echo "Error: Tailscale service failed to start." >&2
    exit 1
fi

# CANについてsystemdに登録する
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

for i in {1..60}; do
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

# ROSが既にインストールされているか確認
ROS_SETUP="/opt/ros/humble/setup.bash"
if [ ! -f "$ROS_SETUP" ]; then
  echo "Error: ROS 2 Humble is not installed." >&2
  echo "Missing: $ROS_SETUP" >&2
  exit 1
fi

set +u
# shellcheck source=/dev/null
source "$ROS_SETUP"
set -u

# rosdepの初期化
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

# SSH / ローカル端末の両方でROS環境を自動読み込みする
# .bashrcだけではログインシェルや実行方法によって読まれない場合があるため、
# 共通環境ファイルを作成して .bashrc と .profile の両方からsourceする。
echo "Configuring ROS 2 shell environment and ccache..."

ROX_CONFIG_DIR="$HOME/.config/rox2026"
ROX_ENV_FILE="$ROX_CONFIG_DIR/env.sh"
mkdir -p "$ROX_CONFIG_DIR"

cat >"$ROX_ENV_FILE" <<'EOF'
# rox2026 shell environment
# This file is generated by the RDK setup script.

# 二重sourceを避ける
if [ -z "${ROX2026_ENV_LOADED:-}" ]; then
  export ROX2026_ENV_LOADED=1

  if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
  fi

  # workspaceを一度でもbuildしてinstall/setup.bashが生成されれば、
  # 以降のSSHログイン/新規terminalで自動的にoverlayする。
  if [ -f "$HOME/rox2026/ros2_ws/install/setup.bash" ]; then
    source "$HOME/rox2026/ros2_ws/install/setup.bash"
  fi

  # ccache
  export CCACHE_DIR="$HOME/.ccache"
  export CMAKE_C_COMPILER_LAUNCHER=ccache
  export CMAKE_CXX_COMPILER_LAUNCHER=ccache

  # Debian/Ubuntuのccache compiler wrappersも優先する。
  if [ -d /usr/lib/ccache ]; then
    case ":$PATH:" in
      *:/usr/lib/ccache:*) ;;
      *) export PATH="/usr/lib/ccache:$PATH" ;;
    esac
  fi

  # interactive shell用alias
  case "$-" in
    *i*)
      alias canshow='ip -details -statistics link show can0'
      alias canrestart='sudo systemctl restart can0.service'
      alias canstatus='systemctl status can0.service'
      ;;
  esac
fi
EOF
chmod 644 "$ROX_ENV_FILE"

# 以前のsetup.shが追加した管理ブロックがあれば、新しいsource形式へ置換する。
python3 - "$HOME/.bashrc" "$HOME/.profile" "$ROX_ENV_FILE" <<'PY_SHELL_ENV'
from pathlib import Path
import re
import sys

bashrc = Path(sys.argv[1])
profile = Path(sys.argv[2])
env_file = sys.argv[3]

begin = '# >>> RDK X5 setup >>>'
end = '# <<< RDK X5 setup <<<'
block = f'{begin}\nif [ -f "{env_file}" ]; then\n  source "{env_file}"\nfi\n{end}\n'

for path in (bashrc, profile):
    text = path.read_text() if path.exists() else ''
    pattern = re.compile(
        rf'^\s*{re.escape(begin)}.*?^\s*{re.escape(end)}\s*$',
        flags=re.MULTILINE | re.DOTALL,
    )
    if pattern.search(text):
        text = pattern.sub(block.rstrip(), text, count=1)
        if not text.endswith('\n'):
            text += '\n'
    else:
        if text and not text.endswith('\n'):
            text += '\n'
        text += '\n' + block
    path.write_text(text)
PY_SHELL_ENV

# ccacheを実際のcolcon/CMakeビルドに常時適用する。
# CMakeの環境変数は初回configure時の初期値なので、既存build treeにも効くよう
# colcon defaultsから毎回CMake launcherを指定する。
mkdir -p "$HOME/.colcon" "$HOME/.ccache"
ccache --set-config="cache_dir=$HOME/.ccache"
ccache --set-config=max_size=10G

COLCON_DEFAULTS="$HOME/.colcon/defaults.yaml"
python3 - "$COLCON_DEFAULTS" <<'PY_COLCON_DEFAULTS'
from pathlib import Path
import sys
import yaml

path = Path(sys.argv[1])
data = yaml.safe_load(path.read_text()) if path.exists() else {}
data = data or {}

if not isinstance(data, dict):
    raise SystemExit(f'Error: {path} must contain a YAML mapping')

build = data.setdefault('build', {})
if not isinstance(build, dict):
    raise SystemExit(f'Error: build section in {path} must be a mapping')

args = build.get('cmake-args', [])
if args is None:
    args = []
elif isinstance(args, str):
    args = [args]
elif not isinstance(args, list):
    raise SystemExit(f'Error: build.cmake-args in {path} must be a list or string')

for arg in (
    '-DCMAKE_C_COMPILER_LAUNCHER=ccache',
    '-DCMAKE_CXX_COMPILER_LAUNCHER=ccache',
):
    if arg not in args:
        args.append(arg)

build['cmake-args'] = args
path.write_text(yaml.safe_dump(data, sort_keys=False))
print(f'Configured colcon defaults: {path}')
PY_COLCON_DEFAULTS

# ヒット率を分かりやすく確認できるよう、setup時点で統計だけリセットする。
ccache --zero-stats >/dev/null 2>&1 || true
ccache --show-config | grep -E 'cache_dir|max_size' || true

# rox2026ワークスペースが存在する場合は依存関係を導入
ROS2_WS="$HOME/rox2026/ros2_ws"
if [ -d "$ROS2_WS/src" ]; then
  echo "Installing workspace dependencies..."

  set +u
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  set -u

  rosdep install \
    --from-paths "$ROS2_WS/src" \
    --ignore-src \
    --rosdistro humble \
    -r \
    -y
fi

echo "Install Finished"

# tailscaleのログインについて任意で行う
if tailscale status >/dev/null 2>&1; then
  echo "Tailscale is already connected."
else
  echo
  read -rp "Authenticate Tailscale now? [y/N]: " TS_CONFIRM

  if [[ "$TS_CONFIRM" =~ ^[Yy]$ ]]; then
    curl -fsSL https://tailscale.com/install.sh | sh
    sudo hostnamectl set-hostname rdk-rox
    sudo tailscale up --accept-routes=false
  else
    echo "Skipping Tailscale authentication."
    echo "Run later:"
    echo "  sudo tailscale up --accept-routes=false"
  fi
fi

echo "Setup Finished"
echo "ROS 2 environment is configured for both SSH login shells and interactive bash."
echo "Open a new SSH session/terminal, or run:"
echo "source ~/.config/rox2026/env.sh"
echo ""
echo "ccache is enabled for colcon C/C++ builds. Check cache effectiveness with:"
echo "ccache -s"
echo ""
echo "Reboot once to apply the boot-time optimizations:"
echo "sudo reboot"