# edulite05_node

EduLite 05のVelocity・Position制御を共通実行ファイルで提供する。mecanum 4台、
Spring 1台、dribble位置1台をnode名とYAMLで分ける。

## 関連ファイル

- node: `hardware_driver/src/nodes/edulite05_node.cpp`
- protocol宣言: `hardware_driver/include/edulite05_driver/edulite05_protocol.hpp`
- protocol実装: `hardware_driver/src/protocol/edulite05_protocol.cpp`
- 設定: `robot_bringup/config/edulite05_driver.yaml`
- 起動: `robot_bringup/launch/hardware/edulite05.launch.py`

最初にnodeの`init_motor()`とcallbackを読み、その後protocolの`Velocity`、
`Position`各classとframe encodeを読む。

## node構成

| 用途 | motor ID | runmode | feedback |
|---|---:|---|---|
| mecanum FL/FR/RL/RR | `0x01`〜`0x04` | Velocity | ROS publishなし |
| Spring | `0x0A` | Velocity | ROS publishなし |
| dribble位置 | `0x38` | Position | 位置radをpublish |

## 起動時処理

`runmode`からVelocityまたはPositionのframe生成器を作り、100 ms待って初期化する。
run mode、制限値、enableなど複数frameを50 ms間隔で送る。

CAN feedbackで電源投入通知を受けた場合、またはenable後にdisable状態を検出した場合は
初期化を再実行する。callback内のsleep中は同nodeの他callback処理が止まる。

## commandとfeedback

commandは`std_msgs/msg/Float32`。Velocityではrad/s、Positionではradとして
control frameへ変換する。node側にはNaN・Infや範囲の検証がないため、controller側で
有効値へ制限する。

feedback CAN IDのmotor IDが一致するframeだけをdecodeする。Velocityならrad/s、
Positionならradを選ぶ。`is_requested_fb_pub=true`のnodeだけROS 2へpublishする。

現在、dribble位置controllerだけがfeedback到達とtimeoutを必要とするため、
`edulite05_dribble_position_driver`だけtrueである。

## 終了

通常のspin終了後にdisable frameを送る。強制kill、プロセスクラッシュ、電源断では
終了frameを送れない可能性がある。起動時の自動enableも含め、実機周辺を安全にしてから
nodeを起動する。

## protocol内の固定値

電流制限、加速度、Position最大速度、host IDなどはprotocolクラス内の固定値で、
YAML parameterではない。変更時はEduLite 05 manualと単位・許容範囲を確認する。

## 調査

動かない場合はnode名とYAML最上位名、motor ID、runmode、初期化frameを確認する。
dribble位置feedbackだけ出ない場合はID `0x38`と`is_requested_fb_pub`を確認する。
