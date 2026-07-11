# Install Nvim

LazyVim を入れるための手順メモです。
このディレクトリには、Neovim の導入スクリプトと、`~/.config/nvim` に上書きする設定一式が入っています。

## 使い方

実行権限を付けてから、`nvim.sh` で Neovim と依存ツールを入れ、`lazyvim.sh` で LazyVim を入れます。

```bash
chmod +x nvim.sh lazyvim.sh
./nvim.sh
./lazyvim.sh
```

最後に `nvim` を起動します。

```bash
nvim
```

## `nvim.sh`

Debian / Ubuntu 系を想定したセットアップスクリプトです。
以下をインストールします。

- `neovim`
- `fd-find`
- `fzf`
- `ripgrep`
- `git`
- `curl`
- `unzip`

`fd-find` は `fd` として使えるように、`~/.local/bin/fd` へシンボリックリンクを作ります。
また、`~/.local/bin` を `PATH` に追加します。

## `lazyvim.sh`

- 既存の Neovim 設定を日付付きでバックアップします
- LazyVim starter を `~/.config/nvim` に入れます
- このリポジトリの `nvim/` の内容を上書きします
- 既存の `~/.local/share/nvim`、`~/.local/state/nvim`、`~/.cache/nvim` も退避します
