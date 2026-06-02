# RDK X5のセットアップ


## 0. RDK X5の"Install Operating System"を見る
[RDK X5(Install OS) Docs](https://developer.d-robotics.cc/rdk_doc/en/Quick_start/install_os/rdk_x5/system_burn)
===
## 1. イメージをSDカードに書き込む
### 1.1 RufusのDownload
* [RufusのDownloadリンク](https://rufus.ie/en/)から"rufus-4.14.exe"を取ってくる
* その後ダウンロードしたファイルを実行

## 1.2. イメージのダウンロード
(一応記述するがDocsを見れはよろしい)
* [ImageをDownloadできるリンク](https://archive.d-robotics.cc/downloads/os_images/rdk_x5/rdk_os_3.5.0-2026-4-9/)に飛ぶ
* Ubuntu22.04のDesktopをDownloadしてくる

## 1.3. イメージをSDカードに書き込む
* SDカードを差し込み、Rufusを起動
* SDカードを選択し、Imageを選択し書き込む

## 2. SSHサービスの有効化
(もともと"SSHサービス"自体有効化されていたが一応記述する)
* 下に記述した"System"は左上の "application"を押すと出てきたはず <!--確認が必要-->
* System -> RDK Configuration -> Interface Options -> SSHでOn, Off可能

## 3. VNC(Vertual Network Computing)
: SSH越しで相手のPCのGUIを操作可能
* System -> RDK Configuration -> Interface Options -> VSCでOn, Off可能
(*デフォルトでPasswordは設定してないのでLogin Passwordを8字いないで作る必要がある)

