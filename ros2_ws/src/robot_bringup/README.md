# robot_bringup

ROX 2026のnode構成、parameter YAML、通常起動と機構別テスト起動を管理する。
controllerやdriverの処理本体は持たず、どのnodeをどの設定で起動するかを決める。

## ディレクトリ

```text
robot_bringup/
├── config/       controller・driverのparameter
└── launch/
    ├── controllers/  robot_controller
    ├── hardware/     EduLite・VESC単体launch
    ├── input/        joy_node・joy_controller
    └── test/         機構別の組み合わせ起動
```

## 通常起動

```bash
ros2 launch robot_bringup robot.launch.py
```

`can_interface`の既定値は`can0`。別interfaceを使う場合は次のように指定する。

```bash
ros2 launch robot_bringup robot.launch.py can_interface:=can1
```

通常起動では以下をすべて起動する。

- ros2_socketcan sender・receiver
- STM32 driver
- VESC driver 3台
- EduLite driver 6台
- belt・dribble、mecanum、Spring、dribble位置controller
- `joy_node`とjoy_controller

Joy deviceの既定値は`/dev/input/js0`である。別deviceを使う場合は通常起動へ
`device:=/dev/input/js1`のように渡す。

## hardware.launch.py

| 引数 | 既定値 | `true`で起動するもの |
|---|---|---|
| `can_interface` | `can0` | SocketCAN interfaceを選ぶ |
| `use_vesc` | `true` | belt・dribble用VESC 3台 |
| `use_stm32` | `true` | limit switch・LED・heartbeat用STM32 |
| `use_edulite_mecanum` | `true` | mecanum用EduLite 4台 |
| `use_edulite_spring` | `true` | Spring用EduLite |
| `use_edulite_dribble_position` | `true` | dribble位置用EduLite |

SocketCAN bridgeはhardwareの選択に関係なく起動する。VESCのnode名は
`vesc_driver_1`〜`3`で、各nodeは同じ`vesc_driver.yaml`から自分の名前に対応する
parameterだけを読む。

## 機構別テストlaunch

| launch | 起動するcontroller | 起動するhardware |
|---|---|---|
| `test/robot_belt_dribble.launch.py` | belt・dribble、Joy | VESC 3台 |
| `test/robot_mecanum.launch.py` | mecanum、Joy | mecanum EduLite 4台 |
| `test/robot_dribble_position.launch.py` | Spring、dribble位置、Joy | STM32、Spring・位置EduLite |

例:

```bash
ros2 launch robot_bringup test/robot_belt_dribble.launch.py
```

テストlaunchでもSocketCAN bridgeが起動する。同じCAN interfaceに対して通常launchと
テストlaunchを同時起動しない。

## config一覧

| YAML | 対象 | 主な設定 |
|---|---|---|
| `joy_controller.yaml` | Joy変換 | button・axis index、timeout、速度上限 |
| `belt_dribble_controller.yaml` | belt・dribble | level RPM、許容差、feedback timeout |
| `mecanum_controller.yaml` | mecanum | 寸法、補正係数、車輪速度上限 |
| `spring_controller.yaml` | Spring | limit switch index、速度、時間 |
| `dribble_position_controller.yaml` | dribble位置 | 各位置、許容差、timeout |
| `vesc_driver.yaml` | VESC 3台 | CAN ID、RPM topic、最大RPM、timeout |
| `edulite05_driver.yaml` | EduLite 6台 | motor ID、mode、topic、feedback有無 |
| `stm32_driver.yaml` | STM32 | CAN topic、limit switch・LED topic、heartbeat |

## 設定変更時の確認

1. YAMLのnode名がlaunchのnode名と一致していることを確認する。
2. 単位を確認する。RPM、rad/s、rad、m、ms、sを混同しない。
3. VESC・EduLiteのIDが実機と一致することを確認する。
4. topicのpublish側とsubscribe側の型が一致することを確認する。
5. Humble Docker内でbuildする。
6. 機構別test launchで1系統ずつ確認してから通常launchを使う。

## 起動後の確認例

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo /operation_mode
ros2 topic echo /underbelt/current/rpm
ros2 topic echo /limit_switchs
ros2 topic hz /dribble/position_feedback
```

CAN interface自体はROS 2起動前に存在し、UPしている必要がある。

```bash
ip -details link show can0
```

## よくある問題

- YAMLを変えたのに反映されない: sourceしているinstallが古い可能性があるため再buildする。
- parameterが既定値になる: YAML最上位のnode名とlaunchの`name`を比較する。
- 同じdriverが複数起動する: 通常launchと機構別test launchの重複起動を確認する。
- CANを受信できない: `can_interface`、bridge topic、CAN bitrate、実機電源を確認する。
- 一部機構だけ不要: `hardware.launch.py`の`use_*`引数をfalseにする。

処理内容と安全動作は`robot_controller/README.md`、CAN変換と実機driverは
`hardware_driver/README.md`、Joy操作は`joy_controller/README.md`を参照する。

さらに詳しいcallback・timer処理は、各パッケージREADME冒頭の
「node別の詳細資料」から個別READMEを参照する。
