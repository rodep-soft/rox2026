# AGENTS.md

このディレクトリ配下で作業するときは、ユーザーから明示的に実装・修正を依頼されていない限り、勝手にファイルを変更しないでください。

コードや設定ファイルを変更する必要がある場合は、先に以下を提案してください。

- 変更するファイル
- 変更する理由
- 変更内容の概要
- 互換性や副作用の見込み

ユーザーが承認してからファイル変更を行ってください。

修正の管理には `fix.md` を使用する。未対応の修正項目は `fix.md` に記録し、対応が完了した項目は `fix.md` から削除する。

buildする際は~/rodep/rox2026/にDocker fileがあるのでその中でbuildを通すようにしてほください。

ros2を主に使いますが実際に動かすのは"Ubuntu22.04"のため、"jazzy"ではなく"humble"です。

hardwareとしては、マイコンには"rdk x5", "stm32"を使用し、mainはrdkからros2を使ってcanでstmに通信を送ります。

用いるmotorは"robstride 05(edulite 05)", "MAD motor", "赤ブラシ"です

RobStrideまたはEduLite 05を制御する実装では、[EduLite 05 Instruction Manual](https://wiki.aifitlab.com/robstride-docs/edulite-05-instruction-manual) に従うこと。

## ROS 2パッケージの責務

パッケージの依存方向は、原則として以下の通りです。

```text
joy_controller → robot_controller → hardware_driver → STM32
```

- `joy_controller`: ジョイスティックなどの操作入力をROS 2の指令へ変換する。
- `robot_controller`: 操作指令を受け取り、各機構の状態遷移・制御判断を行う。CANの実装詳細に依存しない、機構として意味のある指令を`hardware_driver`へ出力する。
- `hardware_driver`: `robot_controller`からの指令をCANフレームへ変換し、リトルエンディアン処理、CAN送受信、受信フレームからの状態復元を担当する。

CAN ID、8 byteフレームの形式、エンディアン、STM32との通信仕様は`hardware_driver`内に閉じ込める。`robot_controller`にはCANフレームの組み立てや送受信処理を書かない。

ジョイスティックのボタン・軸の番号と配置は、`joy_controller/README.md` を正とする。`sensor_msgs/msg/Joy` の `buttons[]` / `axes[]` を扱う実装では、必ずこの対応表を参照すること。

C++の定数名には、`k` プレフィックスを付けない。

git commit のメッセージは日本語で記述する。

## ROS 2ノードのコメントと関数配置

- publisher/subscription を作成する箇所には、topic ごとに用途、送信元・送信先、メッセージの意味をコメントする。
- callback には、どの topic を受信したときに呼ばれるか、更新・判断する状態、publish する topic をコメントする。処理を許可・拒否・停止する条件と、条件を満たさない場合の動作も記載する。
- 関数は原則として、constructor / destructor、interface 作成・parameter 宣言、parameter 取得・検証、subscription callback、timer callback、主処理、値取得・変換・判定などの小さな補助関数、の順に配置する。ヘッダの宣言順も実装順に合わせる。
- callback や timer callback から publish する場合は、出力topicとその目的をコメントから追えるようにする。
- 関数名だけから条件や状態遷移を判断できない関数には、関数定義・宣言の直前に、入力、前提条件、状態の変更、出力を具体的に記載する。
