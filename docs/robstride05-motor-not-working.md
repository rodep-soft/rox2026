# Robstride05 Motorが回らない件

## 発生日

2026-06-01

## 症状

- Robstride05モータが回転しない

## 構成

- [debug]
Laptop --> USB-to-CAN module --> Robstride05

- [本番(予定)]
RDKX5 --> [CAN] --> Robstride05

## resources

### SpeedStudioのDocs

[TechnicalDocs](https://wiki.seeedstudio.com/robstride_control/)

- Linux環境だとsocketCan前提


## 確認/調査済事項

### 回路

- テスターで40Vがバッテリーからでていることは判明

- CAN配線(ほぼ確定)

CAN_H --> 赤
CAN_L --> 黒

### USB-to-CAN mod

- lsusb(Linux)で認識できた

- MotorStudio(on Windows)でも認識できた

- 基板上の2つのDipSwitch

* 基盤の裏に(Boot/CAN)と書いてある
0 --> Boot mode
1 --> CAN bus 終端抵抗

- 電源をONにし、通信を初めて送ると青に光る

- 通信を送ると緑に光る(確定ではない)

#### CAUTION

[以下製品ページより意訳]

このモジュールは公式のホストソフトウェア（GUIツール）専用です。シリアル通信プロトコルのドキュメントは公開していません。独自開発やプログラミングを行う場合は、CANableなどの標準的なCANアダプタを使用してください。

- プロトコルは一応非公開

- ch340を使っているのでsocketCAN(can0)として認識はしない


### RDKX5

- CAN FD ネイティブ対応

## 試したこと

- CAN_H, CAN_Lの交換 --> failed

- MotorStudioからの制御 --> failed but 応答っぽいものは返ってきた

- Pythonから制御 --> failed but 応答っぽいものが返ってきた


## 仮説/調査すべき点

- CANのGNDを共通化すべきか

- Arduinoでの動作確認

- RobstrideMotorを変えてみる

- 





