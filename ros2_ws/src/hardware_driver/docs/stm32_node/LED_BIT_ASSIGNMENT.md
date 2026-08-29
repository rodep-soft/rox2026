# LEDコマンド ビット割り当て仕様

## 1. 概要

RDKからSTM32へ、CAN ID `0x201`のStandard Data FrameでLED状態を送信する。
データ長は5 byte（DLC 5）とする。

| CANデータ | 内容 |
|---|---|
| data[0] | 主表示モード |
| data[1] | 同時表示する付加状態のbit field |
| data[2]～data[4] | 9マス状態（各2 bit、little endian） |

ROS 2では`/hardware/led_cmd`へ`std_msgs/msg/UInt64`を送信する。
`stm32_driver`が次のリトルエンディアン形式でCANフレームへ変換する。

```text
CAN data[0] = ros_value & 0xff
CAN data[1] = (ros_value >> 8) & 0xff
CAN data[2] = (ros_value >> 16) & 0xff
CAN data[3] = (ros_value >> 24) & 0xff
CAN data[4] = (ros_value >> 32) & 0xff
```

## 1.1 Game2ターゲットグリッド

9マス状態はdata[2]～data[4]の下位18 bitへ、1マス2 bitで格納する。

    grid_states |= (state[index] & 0x03) << (index * 2)
    ros_value = mode | (flags << 8) | (grid_states << 16)

状態値は0=STANDING、1=TARGET、2=FALLEN、3=INVALIDとする。

| 配置 | 左 | 中央 | 右 |
|---|---|---|---|
| 上段 | index 0 / Tag 14 | index 1 / Tag 15 | index 2 / Tag 16 |
| 中段 | index 3 / Tag 17 | index 4 / Tag 18 | index 5 / Tag 19 |
| 下段 | index 6 / Tag 20 | index 7 / Tag 21 | index 8 / Tag 22 |

タグIDはgame2_auto_aim.yamlのtop_tags、middle_tags、bottom_tagsで変更できるため、
indexはタグIDではなく位置を表す。STM32での推奨色はSTANDING=緑、TARGET=黄、
FALLEN=消灯、INVALID=紫とする。

Game2有効時も、グリッド表示に割り当てたLED以外は手動モードと同じ通常方向・
前後反転・ドリブルON/OFF・ベルトレベルの色とアニメーションを維持する。
Game2固有色は9マスのグリッドLEDにだけ重畳する。

## 2. data[0]: 主表示モード

`data[0]`はLED全体の基本的な表示方法を指定する。

| 値 | 16進数 | 名前 | 意味 |
|---:|---:|---|---|
| 0 | `0x00` | `STARTUP` | 起動中、または非常停止状態をまだ受信していない |
| 1 | `0x01` | `READY` | 操作可能な通常状態 |
| 2 | `0x02` | `EMERGENCY_STOP` | 非常停止中 |
| 3 | `0x03` | `ARM_OPEN` | ドリブルアームのOPEN姿勢 |
| 4 | `0x04` | `LOADING` | ボールの装填動作中 |
| 5 | `0x05` | `FIRING` | 発射中、または発射通知表示中 |
| 6 | `0x06` | `RETURNING` | 発射後にDRIBBLE位置へ復帰中 |
| 7 | `0x07` | `GAME2_SEARCHING` | Game2でターゲットを探索中 |
| 8 | `0x08` | `GAME2_ALIGNING` | Game2でターゲットへ位置合わせ中 |
| 9 | `0x09` | `ERROR` | ばね機構の異常 |
| 10 | `0x0a` | `ARM_DRIBBLE` | ドリブルアームのDRIBBLE姿勢 |
| 11 | `0x0b` | `SLOW_FIRING` | スロー発射中 |
| 12 | `0x0c` | `ARM_FEED` | ドリブルアームのFEED姿勢 |
| 13 | `0x0d` | `ARM_RECEIVE` | ドリブルアームのRECEIVE姿勢 |
| 14 | `0x0e` | `ARM_HOME` | ドリブルアームのHOME姿勢 |
| 15 | `0x0f` | `BELT_SPINUP` | ベルト発射サイクルの回転立上げ中 |
| 16 | 0x10 | BELT_OFFSET_MINUS_3 | ベルトRPMオフセット -3 step |
| 17 | 0x11 | BELT_OFFSET_MINUS_2 | ベルトRPMオフセット -2 step |
| 18 | 0x12 | BELT_OFFSET_MINUS_1 | ベルトRPMオフセット -1 step |
| 19 | 0x13 | BELT_OFFSET_ZERO | ベルトRPMオフセット 0 step |
| 20 | 0x14 | BELT_OFFSET_PLUS_1 | ベルトRPMオフセット +1 step |
| 21 | 0x15 | BELT_OFFSET_PLUS_2 | ベルトRPMオフセット +2 step |
| 22 | 0x16 | BELT_OFFSET_PLUS_3 | ベルトRPMオフセット +3 step |
| 23～31 | 0x17～0x1f | Reserved | 将来拡張用。主表示として送信しないこと |

未定義値を受信した場合、STM32は安全な既定表示として扱うこと。

### 2.1 入力状態から主表示モードへの変換

現行の`led_controller_node`は、購読した状態を次のように`data[0]`へ変換する。
表にない状態は、より優先度の高い状態がなければ`READY`になる。

| 入力topic | 入力状態 | `data[0]` | 表示 |
|---|---|---:|---|
| `/system/emergency_stop` | 初回メッセージ未受信 | `0x00` | `STARTUP` |
| `/system/emergency_stop` | `true` | `0x02` | `EMERGENCY_STOP` |
| `/spring/fire_request` | `false`から`true`への立上り後`firing_display_ms`の間 | `0x05` | `FIRING` |
| `/spring/operation_state` | `NORMAL_FIRE` | `0x05` | `FIRING` |
| `/spring/operation_state` | `SLOW_FIRE` | `0x0b` | `SLOW_FIRING` |
| `/spring/operation_state` | `ERROR` | `0x09` | `ERROR` |
| `/game2/state` | `SEARCHING` | `0x07` | `GAME2_SEARCHING` |
| `/game2/state` | `ALIGNING` | `0x08` | `GAME2_ALIGNING` |
| `/game2/state` | `PREPARING_SHOOT` | `0x04` | `LOADING` |
| `/game2/state` | `SHOOTING` | `0x05` | `FIRING` |
| `/game2/state` | `WAITING_RESULT` | `0x06` | `RETURNING` |
| `/game2/state` | `STANDBY`または`COMPLETED` | 下位状態で決定 | Game2固有表示なし |
| `/dribble/shot_cycle_state` | `BELT_SPINUP` | `0x0f` | `BELT_SPINUP` |
| `/dribble/shot_cycle_state` | `FEEDING` | `0x04` | `LOADING` |
| `/dribble/shot_cycle_state` | `RETURNING` | `0x06` | `RETURNING` |
| `/dribble/command_position` | `DRIBBLE` | `0x0a` | `ARM_DRIBBLE` |
| `/dribble/command_position` | `OPEN` | `0x03` | `ARM_OPEN` |
| `/dribble/command_position` | `FEED` | `0x0c` | `ARM_FEED` |
| `/dribble/command_position` | `RECEIVE` | `0x0d` | `ARM_RECEIVE` |
| `/dribble/command_position` | `HOME` | `0x0e` | `ARM_HOME` |
| /belt/command_mode | rpm_offset_step が 0 以外 | 0x10～0x16 | 累積オフセット -3～+3 step |

firing_display_msの既定値は500 ms、コマンドの配信周期publish_period_msの既定値は100 msである。
spring/fire_requestをtrueのまま保持しても表示時間は延長されず、いったんfalseを受信した後の
次の立上りで再トリガーされる。

ベルトRPMオフセット表示時間 belt_offset_display_ms の既定値は1000 msである。
オフセット変更メッセージを受信すると、led_controller_node もベルトコントローラと
同じく増分を -3～+3 step に累積し、その値に対応する主表示モードを表示時間中送信する。

### 2.2 ベルトRPMオフセットの表示方法

オフセット変更後は belt_offset_display_ms の間、符号と絶対値をバー表示する。
正の値は緑、負の値は赤、0 stepは青で表示する。絶対値1はLED列の1/3、
絶対値2は2/3、絶対値3は全体を点灯する。表示期間終了後は通常の優先順位へ戻る。

この表示は rpm_offset_step の単発値ではなく、-3～+3にクランプした累積値を示す。
ベルトコントローラの belt_rpm_offset_per_step が100 RPMなら、+2 stepは+200 RPMに相当する。

## 3. data[1]: 付加状態

```text
bit:       7       6       5       4       3       2       1       0
         +-------+-------+-------+-------+-------+-------+-------+-------+
data[1]  |ローラ逆|ローラ正| Game2 | 反転  |Dribble|     Belt level       |
         +-------+-------+-------+-------+-------+-------+-------+-------+
mask       0x80    0x40    0x20    0x10    0x08       0x07
```

### 3.1 bit 0～2: ベルト速度段階

マスクは`0x07`。値を取得するときは`data[1] & 0x07`とする。

| bit 2～0 | 値 | ベルト状態 |
|---|---:|---|
| `000` | 0 | STOP |
| `001` | 1 | LEVEL 1 |
| `010` | 2 | LEVEL 2 |
| `011` | 3 | LEVEL 3 |
| `100` | 4 | LEVEL 4 |
| `101`～`111` | 5～7 | 予約。不正値としてSTOP表示にする |

この値は`/belt/command_mode`の0～4に対応する。Game2から直接指定されるRPMそのものは
この3 bitには格納しない。

### 3.2 bit 3: ドリブル状態

| 値 | 意味 |
|---:|---|
| 0 | ドリブル停止 |
| 1 | ドリブル稼働 |

マスクは`0x08`。

```c
bool dribble_enabled = (data[1] & 0x08U) != 0U;
```

### 3.3 bit 4: ロボット前後反転

| 値 | 意味 |
|---:|---|
| 0 | 通常方向 |
| 1 | 前後反転方向 |

マスクは`0x10`。反転中は、LEDで示す前方色と後方色を入れ替える。

```c
bool drive_reversed = (data[1] & 0x10U) != 0U;
```

### 3.4 bit 5: Game2自動制御

| 値 | 意味 |
|---:|---|
| 0 | Game2停止中 |
| 1 | Game2実行中 |

マスクは`0x20`。詳細なGame2進行状態は`data[0]`で表す。

```c
bool game2_enabled = (data[1] & 0x20U) != 0U;
```

`led_controller_node`は`/game2/command_start` (`std_msgs/msg/Bool`) の最新値が
`true`のときにこのbitを1にする。`/game2/state`はGame2の詳細な進行表示を
`data[0]`で選択するために使用し、自動モードの有効判定には使用しない。

### 3.5 bit 6～7: ドリブルローラ回転方向

`led_controller_node`は`/vesc/target`のうち`roller_logical_id`（既定値12）
と一致する指令値を監視する。

| bit 7～6 | `data[1]`加算値 | ローラ状態 |
|---|---:|---|
| `00` | `0x00` | 停止（目標RPM = 0） |
| `01` | `0x40` | 正転（目標RPM > 0） |
| `10` | `0x80` | 逆転（目標RPM < 0） |
| `11` | `0xc0` | 予約。送信しない |

bit 3はドリブル機能の有効/無効、bit 6～7は実際に出力されている目標回転方向を
表すため、両者は独立して送信する。スロー発射やベルト発射サイクル中のローラも
bit 6～7へ反映される。

## 4. 主表示の優先順位

複数状態が同時に成立した場合、RDKの`led_controller_node`は次の優先順位で
`data[0]`を決定する。

```text
STARTUP判定
  > EMERGENCY_STOP
  > ばね機構ERROR
  > 通常ばね発射
  > スロー発射
  > Game2進行状態
  > ベルト発射サイクル進行状態
  > ベルトRPMオフセット変更表示
  > ドリブルアーム姿勢
  > READY
```

`data[1]`の付加状態は主表示とは別に保持される。ただしSTM32は、非常停止などの
主表示を優先し、付加状態を画面へ反映しなくてもよい。

主表示の具体的な決定順は次のとおり。

1. `/system/emergency_stop`をまだ受信していなければ`STARTUP`
2. 非常停止中なら`EMERGENCY_STOP`
3. ばね機構が異常なら`ERROR`
4. 通常ばね発射中、または手動発射の通知期間中なら`FIRING`
5. スロー発射中なら`SLOW_FIRING`
6. Game2が進行中なら、その進行状態に対応する表示
7. ベルト発射サイクル中なら、その進行状態に対応する表示
8. ベルトRPMオフセットの変更表示期間中なら、累積値 -3～+3 に対応する表示
9. サイクル外では、指令されたドリブルアーム姿勢に対応する表示
10. いずれにも該当しなければ`READY`

## 5. エンコード例

### READY、ベルトLEVEL 3、通常方向

```text
data[0] = 0x01
data[1] = 0b00000011 = 0x03
UInt64  = 0x0301 = 769
```

### READY、ベルトLEVEL 2、ドリブルON、前後反転

```text
data[0] = 0x01
data[1] = 0b00011010 = 0x1a
UInt64  = 0x1a01 = 6657
```

### Game2装填中、ドリブルON、Game2有効

```text
data[0] = 0x04
data[1] = 0b00101000 = 0x28
UInt64  = 0x2804 = 10244
```

### 非常停止、ベルト・ドリブル停止

```text
data[0] = 0x02
data[1] = 0x00
UInt64  = 0x0002 = 2
```

### Game2位置合わせ、Tag 22狙い、複数タグ倒れ

Tag 22（index 8）をTARGET、index 0、1、3～7をFALLENとした場合。

    grid_states = 0x1aa8a
    data[0] = 0x08
    data[1] = 0x20
    UInt64 = 0x1aa8a2008 = 7156146184
    CAN payload = 08 20 8A AA 01

## 6. 動作別の送信パターン

以下はベルトSTOP、ドリブルOFF、通常方向を基準としたパターンである。
Game2の各パターンではGame2有効bit（`0x20`）を付加する。

| 動作 | `data[0]` | `data[1]` | `UInt64`（16進数） | `UInt64`（10進数） | CAN payload |
|---|---:|---:|---:|---:|---|
| 起動中 | `0x00` | `0x00` | `0x0000` | 0 | `00 00 00 00 00` |
| 通常待機 | `0x01` | `0x00` | `0x0001` | 1 | `01 00 00 00 00` |
| 非常停止 | `0x02` | `0x00` | `0x0002` | 2 | `02 00 00 00 00` |
| shot cycle装填中 | `0x04` | `0x00` | `0x0004` | 4 | `04 00 00 00 00` |
| 通常ばね発射 | `0x05` | `0x00` | `0x0005` | 5 | `05 00 00 00 00` |
| スロー発射 | `0x0b` | `0x00` | `0x000b` | 11 | `0B 00 00 00 00` |
| ベルト発射・回転立上げ | `0x0f` | `0x00` | `0x000f` | 15 | `0F 00 00 00 00` |
| shot cycle復帰中 | `0x06` | `0x00` | `0x0006` | 6 | `06 00 00 00 00` |
| ドリブル姿勢DRIBBLE | `0x0a` | `0x00` | `0x000a` | 10 | `0A 00 00 00 00` |
| ドリブル姿勢OPEN | `0x03` | `0x00` | `0x0003` | 3 | `03 00 00 00 00` |
| ドリブル姿勢FEED | `0x0c` | `0x00` | `0x000c` | 12 | `0C 00 00 00 00` |
| ドリブル姿勢RECEIVE | `0x0d` | `0x00` | `0x000d` | 13 | `0D 00 00 00 00` |
| ドリブル姿勢HOME | `0x0e` | `0x00` | `0x000e` | 14 | `0E 00 00 00 00` |
| 前後反転・DRIBBLE姿勢 | `0x0a` | `0x10` | `0x100a` | 4106 | `0A 10 00 00 00` |
| ドリブルON・ローラ正転・DRIBBLE姿勢 | `0x0a` | `0x48` | `0x480a` | 18442 | `0A 48 00 00 00` |
| ドリブルON・ローラ逆転・DRIBBLE姿勢 | `0x0a` | `0x88` | `0x880a` | 34826 | `0A 88 00 00 00` |
| Game2探索中 | `0x07` | `0x20` | `0x2007` | 8199 | `07 20 00 00 00` |
| Game2位置合わせ中 | `0x08` | `0x20` | `0x2008` | 8200 | `08 20 00 00 00` |
| Game2発射準備中 | `0x04` | `0x20` | `0x2004` | 8196 | `04 20 00 00 00` |
| Game2発射中 | `0x05` | `0x20` | `0x2005` | 8197 | `05 20 00 00 00` |
| Game2結果待ち | `0x06` | `0x20` | `0x2006` | 8198 | `06 20 00 00 00` |

付加状態を組み合わせる場合は、次の式で`data[1]`を生成する。

```text
data[1] = belt_level
        | (dribble_enabled ? 0x08 : 0x00)
        | (drive_reversed  ? 0x10 : 0x00)
        | (game2_enabled   ? 0x20 : 0x00)
        | (roller_forward ? 0x40 : 0x00)
        | (roller_reverse ? 0x80 : 0x00)

UInt64 = data[0] | (data[1] << 8) | (grid_states << 16)
```

例えばGame2発射中、ベルトLEVEL 4、ドリブルON、前後反転では次の値になる。

```text
data[0] = 0x05
data[1] = 0x04 | 0x08 | 0x10 | 0x20 = 0x3c
UInt64  = 0x3c05 = 15365
CAN payload = 05 3C 00 00 00
```

## 7. ROS 2試験コマンド

例えば`READY + LEVEL 2 + ドリブルON + 前後反転`を直接送る場合は次を使用する。

```bash
ros2 topic pub --once /hardware/led_cmd std_msgs/msg/UInt64 "{data: 6657}"
```

CANへ正しく変換されると、`0x201`のデータは次の並びになる。

```text
5 byteのlittle endianで 01 1A 00 00 00
```

```bash
candump can0,201:7FF
```

想定表示例：

```text
can0  201   [5]  01 1A 00 00 00
```

追加動作の代表パターンは次のコマンドで直接送信できる。

```bash
# Game2探索中: UInt64 0x2007
ros2 topic pub --once /hardware/led_cmd std_msgs/msg/UInt64 "{data: 8199}"

# Game2位置合わせ中: UInt64 0x2008
ros2 topic pub --once /hardware/led_cmd std_msgs/msg/UInt64 "{data: 8200}"

# Game2発射準備中: UInt64 0x2004
ros2 topic pub --once /hardware/led_cmd std_msgs/msg/UInt64 "{data: 8196}"

# Game2発射中: UInt64 0x2005
ros2 topic pub --once /hardware/led_cmd std_msgs/msg/UInt64 "{data: 8197}"

# Game2結果待ち: UInt64 0x2006
ros2 topic pub --once /hardware/led_cmd std_msgs/msg/UInt64 "{data: 8198}"

# shot cycle装填中: UInt64 0x0004
ros2 topic pub --once /hardware/led_cmd std_msgs/msg/UInt64 "{data: 4}"

# shot cycle復帰中: UInt64 0x0006
ros2 topic pub --once /hardware/led_cmd std_msgs/msg/UInt64 "{data: 6}"

# 通常ばね発射: UInt64 0x0005
ros2 topic pub --once /hardware/led_cmd std_msgs/msg/UInt64 "{data: 5}"

# スロー発射: UInt64 0x000b
ros2 topic pub --once /hardware/led_cmd std_msgs/msg/UInt64 "{data: 11}"

# ベルト発射・回転立上げ: UInt64 0x000f
ros2 topic pub --once /hardware/led_cmd std_msgs/msg/UInt64 "{data: 15}"

# 前後反転・DRIBBLE姿勢: UInt64 0x100a
ros2 topic pub --once /hardware/led_cmd std_msgs/msg/UInt64 "{data: 4106}"

# ドリブルON・ローラ正転・DRIBBLE姿勢: UInt64 0x480a
ros2 topic pub --once /hardware/led_cmd std_msgs/msg/UInt64 "{data: 18442}"

# ドリブルON・ローラ逆転・DRIBBLE姿勢: UInt64 0x880a
ros2 topic pub --once /hardware/led_cmd std_msgs/msg/UInt64 "{data: 34826}"
```

`led_controller_node`自身の状態遷移を確認する場合は、出力値を直接publishせず、
対応する入力topic（`/game2/state`、`/dribble/shot_cycle_state`、
`/spring/fire_request`など）へ試験メッセージを送り、次で出力を確認する。

```bash
ros2 topic echo /hardware/led_cmd
```

## 8. 互換性規則

- `data[0]`の既存値0～9の意味を変更しない。
- `data[1]`の既存bitを別の用途へ再割り当てしない。
- bit 6～7はローラ回転方向として使用する。古いSTM32はこのbitを無視してよい。
- 新規主表示モードは値10以降へ追加し、既存値0～9の意味を維持する。
- data[2]～data[4]の未使用bit 18～23は0にする。
- CAN ID、DLC、またはbyte配置を変更する場合は、RDKとSTM32を同時に更新する。
