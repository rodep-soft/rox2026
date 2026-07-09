# Setup Notes

このディレクトリには、開発環境のセットアップ手順と設定ファイルがあります。

## 必要最低限のツール

- 基本ツールは `env_setup.sh` にまとまっています
- このスクリプトで `tmux`、`fzf`、`openssh-server` を導入し、SSH サービスを有効化・起動します

```bash
chmod +x env_setup.sh
./env_setup.sh
```

- SSH サービスだけ手動で起動したい場合は、次を実行します

```bash
sudo systemctl start ssh
```

## `nvim_install/`

- Neovim と LazyVim を入れるためのスクリプトと設定一式があります
- `nvim_install/nvim/` の内容は、`~/.config/nvim` に重ねて使う想定です
- 詳しい手順は [`nvim_install/README.md`](./nvim_install/README.md) を見てください

## `tmux.config`

- `tmux` の設定ファイルです
