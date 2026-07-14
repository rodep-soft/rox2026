# robstride_can_node

RobStride 05 モーターへ、位置指令をCAN frameとして送るnode。

## 全体の流れ

```
[Float32 topic]              [robstride_can_node]                [can_msgs/msg/Frame topic]         [実CANバス]
/robstride/position_cmd  --> CommandCallback で受信・clamp   --> can_tx_topic (例: /CAN/can0/transmit) --> ros2socketcan_bridge --> can0/can1 --> RobStrideモーター
                              -> command_target_ に保持
                              TimerCallback が send_period_ms
                              ごとにcommand_target_をCAN frame化
                              して publish
```

1. `command_topic` (`std_msgs/msg/Float32`) に目標角度 `[rad]` を送る。
2. `CommandCallback` が受け取った値を `position_min_rad_`〜`position_max_rad_` で clamp して `command_target_` に保持する。
3. `TimerCallback` が `send_period_ms` 周期で `command_target_` をRobStrideのCANプロトコルに従ったフレームに変換し、`can_tx_topic` に publish し続ける。
4. `ros2socketcan_bridge` がそのtopicをsubscribeして実際のCANバス(`can0`/`can1`)に送信し、モーターに届く。

このnode自身はSocketCANへ直接writeしない。あくまで「ROS 2 topic → CAN frame topic」の変換に徹し、実際の送信は`ros2socketcan_bridge`に任せる設計。

## 起動時の初期化

`enable_topic`で起動許可を受け、かつCAN送信topicのsubscriberが接続された後、RobStride 05マニュアルのPP位置モード手順に合わせて以下を一度だけ送る(`startup_timer_`):

1. `run_mode=1` を書き込み(PP位置モード)
2. `enable_on_startup: true` ならモーター有効化コマンド(Communication Type 3)
3. `position_current_limit` が0.0以上なら `limit_cur` を書き込み
4. `position_speed` が0.0以上なら `vel_max` を書き込み
5. `position_acceleration` が0.0以上なら `acc_set` を書き込み
6. `home_position_rad`（既定`0.0 rad`）を起動時の初期 `loc_ref` として送信

各初期化フレームの間には`startup_inter_frame_ms`だけ待機する。`run_mode`、`limit_cur`、`vel_max`、`acc_set`はモーター側が設定を保持するため、これらは起動時の1回だけで十分。Type 3(有効化)はType 18の書き込みを「反映」させるものではなく、モーターの制御ループ・出力そのものを有効にするスイッチ。

## 終了時の安全停止

ノード終了時(Ctrl-C / SIGTERM)には、モーターを`home_position_rad`（既定`0.0 rad`）へ復帰させてからトルクを切る:

1. `loc_ref=home_position_rad` を`send_period_ms`周期で送信する
2. `shutdown_return_wait_ms`だけ待ち、モーターがhome位置へ移動する時間を確保する
3. モーター停止コマンド(Communication Type 4)を送信してトルク出力を切る

位置フィードバックは使用していないため、待機時間は実際の移動完了を検出するものではない。最大移動距離と`position_speed`、実機の負荷を考慮して設定する。現在のbringup設定では、`±4π`の範囲を`2 rad/s`で戻る最長約6.3秒に対して7秒を設定している。

ROS 2標準のsignal handlerは受信直後にcontextを無効化してしまい、その後のpublishが失敗する。そのため`main`で`shutdown_on_signal=false`を指定し、自前のsignal handlerでフラグを立てて、**contextが有効なうちに停止フレームを送ってからshutdownする**構成にしている。

## 指令途絶時の動作

指令が途切れても、最後に受け取った目標角度(`loc_ref`)を送り続ける。PP modeとして目標位置に居座るだけなので、指令途絶による停止処理は持たない。

## 起動

この package 内の launch file は削除済み。複数 node の起動構成は `robot_bringup` 側に集約する。

## 主要パラメータ (`robot_bringup/config/robstride05.yaml`)

| パラメータ | 説明 |
|---|---|
| `can_tx_topic` | `ros2socketcan_bridge`へ渡すCAN送信topic |
| `motor_can_id` / `host_can_id` | 対象モーターのCAN ID / このnode(ホスト)側のCAN ID |
| `command_topic` | 目標角度を受け取るtopic (`std_msgs/Float32`) |
| `send_period_ms` | 指令フレームを送信する周期 |
| `startup_inter_frame_ms` | 起動時の各CANフレーム間の待機時間[ms] |
| `position_min_rad`/`max_rad` | 位置指令のclamp範囲 |
| `home_position_rad` | 起動時の初期目標位置と、終了時に復帰する位置。既定は`0.0 rad` |
| `enable_on_startup` | 起動時にモーター有効化コマンドを送るか |
| `position_speed` | PP位置モード速度。負値(デフォルト)は未指定で変更せず、起動時にWARNを出す |
| `position_acceleration` | PP位置モード加速度。負値(デフォルト)は未指定で変更せず、起動時にWARNを出す |
| `position_current_limit` | PP/速度モード電流上限[A]。負値(デフォルト)は未指定で変更しない。上限11 A |
| `shutdown_return_wait_ms` | 終了時にhome位置へ戻す待機時間[ms]。待機後にType 4でトルクを切る |

## RobStrideのCANプロトコル概要

29bit拡張CAN IDは以下のビットフィールドを持つ:

```
Bit28-24 (5bit) : Communication Type
Bit23-8  (16bit): Data Area 2 (このnodeでは常にhost_can_idを入れる)
Bit7-0   (8bit) : 宛先CAN ID (motor_can_id)
```

- **Type 3 (モーター有効化)**: dataは全byte 0。
- **Type 4 (モーター停止)**: dataは全byte 0。
- **Type 18 (単一パラメータ書き込み)**: 8byteのdata部分が独自フォーマットを持つ。
  - `data[0-1]`: パラメータのindex(リトルエンディアンuint16)
  - `data[2-3]`: 未使用(0)
  - `data[4-7]`: 値本体。uint8なら`data[4]`のみ、floatならIEEE754をリトルエンディアンで4byte

使用しているパラメータindex(詳細はマニュアル参照):

| index | 名前 | 型 | 内容 |
|---|---|---|---|
| `0x7005` | run_mode | uint8 | 1=位置モード(PP) |
| `0x7016` | loc_ref | float | 目標角度[rad] |
| `0x7024` | vel_max | float | PP位置モード速度[rad/s] |
| `0x7025` | acc_set | float | PP位置モード加速度[rad/s^2] |

参考: [RobStride 05 Instruction Manual](https://wiki.aifitlab.com/robstride-docs/robstride-05-instruction-manual)

## 既知の制約

- 実機のRobStrideモーターに対する通信確認はまだ行っていない(ロジックとマニュアル記述の整合性のみ確認済み)。
- 速度モードはこのpackageから削除済み。退避版は `../../archive/robstride_can_node/` にある。
