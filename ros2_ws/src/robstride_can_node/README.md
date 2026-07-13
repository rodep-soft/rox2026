# robstride_can_node

RobStride (EduLite-05) モーターへ、位置指令 or 速度指令をCAN frameとして送るnode。

## 全体の流れ

```
[Float32 topic]              [robstride_can_node]                [can_msgs/msg/Frame topic]         [実CANバス]
/robstride/position_cmd  --> CommandCallback で受信・clamp   --> can_tx_topic (例: /CAN/can0/transmit) --> ros2socketcan_bridge --> can0/can1 --> RobStrideモーター
   (または velocity_cmd)      -> command_target_ に保持
                              TimerCallback が send_period_ms
                              ごとにcommand_target_をCAN frame化
                              して publish
```

1. `command_topic` (`std_msgs/msg/Float32`) に目標値(位置なら`[rad]`、速度なら`[rad/s]`)を送る。
2. `CommandCallback` が受け取った値を `position_min_rad_`〜`position_max_rad_`(または velocity側の範囲)で clamp して `command_target_` に保持する。
3. `TimerCallback` が `send_period_ms` 周期で `command_target_` をRobStrideのCANプロトコルに従ったフレームに変換し、`can_tx_topic` に publish し続ける。
4. `ros2socketcan_bridge` がそのtopicをsubscribeして実際のCANバス(`can0`/`can1`)に送信し、モーターに届く。

このnode自身はSocketCANへ直接writeしない。あくまで「ROS 2 topic → CAN frame topic」の変換に徹し、実際の送信は`ros2socketcan_bridge`に任せる設計。

## 起動時の初期化

ノード起動から100ms後、以下を一度だけ送る(`startup_timer_`、送信順はこの通り。マニュアル推奨の「パラメータ設定→有効化」の順序に合わせている):

1. `control_mode` に応じて `run_mode` (位置=1 / 速度=2) を書き込み
2. `limit_torque` / `limit_cur` / `limit_spd` のうち値が設定されているもの(0.0以上)だけを書き込み
3. `enable_on_startup: true` ならモーター有効化コマンド(Communication Type 3)を送信

`run_mode`や`limit_*`はモーター側が設定を保持するため、これらは起動時の1回だけで十分。Type 3(有効化)はType 18の書き込みを「反映」させるものではなく、モーターの制御ループ・出力そのものを有効にするスイッチなので、先にパラメータを確定させてから有効化する。

## 終了時の安全停止

ノード終了時(Ctrl-C / SIGTERM)には、モーターが有効化されたまま放置されないよう停止フレームを送る:

1. velocity modeの場合はまず `spd_ref=0` を送信(回転指令を確実に落とす)
2. モーター停止コマンド(Communication Type 4)を送信してトルク出力を切る

ROS 2標準のsignal handlerは受信直後にcontextを無効化してしまい、その後のpublishが失敗する。そのため`main`で`shutdown_on_signal=false`を指定し、自前のsignal handlerでフラグを立てて、**contextが有効なうちに停止フレームを送ってからshutdownする**構成にしている。

## control_modeによる動作の違い

`control_mode`は起動時に固定するパラメータで、実行中の動的切り替えはしない。position用/velocity用にそれぞれ専用のlaunchファイルを用意し、モーターごとに使う方を起動する設計。

- **position mode**: 指令が途切れても、最後に受け取った目標角度(`loc_ref`)を送り続ける。PP modeとして目標位置に居座るだけなので安全。
- **velocity mode**: 指令が`timeout_ms`より長く途切れた場合、`spd_ref=0`を送って安全停止させる(回転し続けると危険なため)。

## Launch

```bash
# 位置制御用ノードを起動 (robstride_position_node)
ros2 launch robstride_can_node robstride_position.launch.py

# 速度制御用ノードを起動 (robstride_velocity_node)
ros2 launch robstride_can_node robstride_velocity.launch.py

# 両方まとめて起動
ros2 launch robstride_can_node robstride_can_node.launch.py
```

いずれも `config_file` 引数(デフォルトは `config/config.yaml`)でパラメータファイルを差し替え可能。

## 主要パラメータ (config/config.yaml)

| パラメータ | 説明 |
|---|---|
| `can_tx_topic` | `ros2socketcan_bridge`へ渡すCAN送信topic |
| `motor_can_id` / `host_can_id` | 対象モーターのCAN ID / このnode(ホスト)側のCAN ID |
| `control_mode` | `"position"` または `"velocity"` (起動時に固定) |
| `command_topic` | 目標値を受け取るtopic (`std_msgs/Float32`) |
| `send_period_ms` | 指令フレームを送信する周期 |
| `timeout_ms` | velocity modeで指令途絶とみなすまでの時間(安全停止用) |
| `position_min_rad`/`max_rad` | 位置指令のclamp範囲 |
| `velocity_min_rad_s`/`max_rad_s` | 速度指令のclamp範囲(マニュアル上の`spd_ref`範囲: -50~50) |
| `enable_on_startup` | 起動時にモーター有効化コマンドを送るか |
| `limit_torque` / `limit_cur` / `limit_spd` | 起動時に書き込むトルク/電流/速度制限。負値(デフォルト)は未指定でモーター側の既存設定を変更しない |

## RobStrideのCANプロトコル概要

29bit拡張CAN IDは以下のビットフィールドを持つ:

```
Bit28-24 (5bit) : Communication Type (このnodeが使うのはType 3とType 18のみ)
Bit23-8  (16bit): Data Area 2 (このnodeでは常にhost_can_idを入れる)
Bit7-0   (8bit) : 宛先CAN ID (motor_can_id)
```

- **Type 3 (モーター有効化)**: dataは全byte 0。
- **Type 18 (単一パラメータ書き込み)**: 8byteのdata部分が独自フォーマットを持つ。
  - `data[0-1]`: パラメータのindex(リトルエンディアンuint16)
  - `data[2-3]`: 未使用(0)
  - `data[4-7]`: 値本体。uint8なら`data[4]`のみ、floatならIEEE754をリトルエンディアンで4byte

使用しているパラメータindex(詳細はマニュアル参照):

| index | 名前 | 型 | 内容 |
|---|---|---|---|
| `0x7005` | run_mode | uint8 | 1=位置モード(PP), 2=速度モード |
| `0x7016` | loc_ref | float | 目標角度[rad] |
| `0x700A` | spd_ref | float | 目標角速度[rad/s] |
| `0x700B` | limit_torque | float | トルク制限[N・m] (0~6) |
| `0x7017` | limit_spd | float | CSP位置モード速度制限[rad/s] (0~50) |
| `0x7018` | limit_cur | float | 電流制限[A] (0~11) |

参考: [RobStride EduLite-05 Instruction Manual](https://wiki.aifitlab.com/robstride-docs/edulite-05-instruction-manual)

## 既知の制約

- 実機のRobStrideモーターに対する通信確認はまだ行っていない(ロジックとマニュアル記述の整合性のみ確認済み)。
- `control_mode`は起動後の動的切り替え非対応。切り替えたい場合はノードを再起動する。
