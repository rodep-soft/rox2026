# setupのためのshell script

# パッケージの更新
sudo apt update

# ツールのインストール
sudo apt install -y \
  tmux \
  fzf \
  openssh-server

#  SSHサーバの有効化
sudo systemctl enable ssh
sudo systemctl start ssh

echo "Setup Finished"
