# joy_controller

`sensor_msgs/msg/Joy`を機構指令と4つのoperation modeへ変換する。

## コードを読む順序

`joy_controller`は1パッケージ1nodeのため、このREADMEを個別node資料として扱う。
実装を読むときは次の順が分かりやすい。

1. `include/joy_controller/joy_controller_node.hpp`のenumと内部状態
2. constructorのparameter検証とpub/sub作成
3. `update_chord_inputs()`の同時押し判定
4. `handle_operation_mode()`のmode遷移
5. `joy_callback()`の機構指令とstick変換
6. 2つのtimer callbackの周期publish・通信断処理
7. shot cycleのrunning・complete callback

Joy nodeは機構のCANや到達判定を行わず、操作意図をROS topicへ変換するところまでを
責務とする。

## Joy配置

### buttons

| index | ボタン |
|---:|---|
| 0 | □ |
| 1 | × |
| 2 | ○ |
| 3 | △ |
| 4 | L1 |
| 5 | R1 |
| 6 | L2 |
| 7 | R2 |
| 8 | Create |
| 9 | Options |
| 10 | L3 |
| 11 | R3 |
| 12 | PS |
| 13 | タッチパッド（Home） |

### axes

| index | 入力 |
|---:|---|
| 0 | 左スティック左右 |
| 1 | 左スティック上下 |
| 2 | 右スティック左右 |
| 3 | L2（通常=`1`、押下=`-1`） |
| 4 | R2（通常=`1`、押下=`-1`） |
| 5 | 右スティック上下 |
| 6 | DPAD左右（左=`1`、右=`-1`） |
| 7 | DPAD上下（上=`1`、下=`-1`） |

## operation mode

| 値 | mode |
|---:|---|
| 0 | STOP |
| 1 | DRIVE |
| 2 | SHOT_CYCLE |
| 3 | BELT_ONLY |

```text
STOP -- Home --> DRIVE
STOP・DRIVE <-- Create --> SHOT_CYCLE
STOP・DRIVE <-- Options --> BELT_ONLY
STOP以外 -- Home --> STOP
```

SHOT_CYCLEの動作中でもHomeはSTOPとして受理する。
SHOT_CYCLEの動作中は、Home以外のmode変更を無視する。
BELT_ONLY中のCreateと、SHOT_CYCLE中のOptionsは無視する。
`auto_drive_on_shot_cycle_complete=true`の場合、
`/shot_cycle/complete=true`を受けるとDRIVEへ戻る。

## 操作

| 入力 | 動作 |
|---|---|
| Home | STOPへ移動。STOP中はDRIVEへ戻る |
| Create | STOP・DRIVEからSHOT_CYCLEへ移動。同modeの待機中はDRIVEへ戻る |
| Options | STOP・DRIVEからBELT_ONLYへ移動。同mode中はDRIVEへ戻る |
| DPAD上 / 下 | belt modeをSTOPとLEVEL_1〜LEVEL_6の範囲で増減 |
| R1 | dribble ON/OFF。DRIVE・SHOT_CYCLEだけで受理 |
| L1 + ○ | Spring発射要求。DRIVEだけで、押下中にtrueを送る |
| L2 + ○ | SHOT_CYCLE中の実行要求 |
| R2 + DPAD左 | DRIBBLE位置へ移動。DRIVE・SHOT_CYCLEだけで受理 |
| R2 + DPAD右 | OPEN位置へ移動。DRIVE・SHOT_CYCLEだけで受理 |
| PS | `linear.x/y`の前後左右反転 |

STOPへ入った場合とJoy通信が途切れた場合は、belt mode・dribble状態・
位置シーケンス状態を保持せず初期化する。
Joy通信が途切れた場合はSTOPへ移動し、Joy入力が復帰するまで状態の周期publishを止める。

## Joy入力の処理順

`/joy`を受けるたびに次の順で処理する。

1. Joy受信時刻を更新し、timeout状態を解除する。
2. buttonとaxisから同時押し状態を作る。
3. Home、Create、Optionsの立ち上がりでoperation modeを更新する。
4. DPAD上下の立ち上がりでbelt levelを1段階変更する。
5. R1の立ち上がりでdribble ON/OFFを切り替える。
6. PSの立ち上がりで並進方向反転を切り替える。
7. shot・手動positionの同時押しを必要なmodeでだけpublishする。
8. stick入力を`cmd_vel`へ変換してpublishする。
9. 現在の入力を前回値として保存する。

button操作は基本的に立ち上がり判定なので、押し続けてもmodeやlevelは連続変化しない。
SpringだけはL1+○を押している間、周期timerからtrueを送り続ける。Spring controller側は
そのtrueの立ち上がりだけを発射要求として扱う。

## stick処理

- 左stick上下はdeadzone適用後の連続値を前後速度へ使う。
- 左stick左右は`lateral_axis_threshold`以上で`-1`または`+1`へ丸める。
- 左右移動が成立した周期は、前後速度を必ず0にする。
- 右stick左右はdeadzone適用後の連続値を旋回速度へ使う。
- 各値へlimitとscaleを掛け、PS反転は並進2軸だけへ適用する。

入力indexが配列外の場合はbutton=false、axis=0として扱う。NaNやInfのaxisも0として
扱うため、短いJoy messageや壊れたaxis値で範囲外アクセスしない。

## STOPと通信断

起動直後は`publish_stop_commands()`を実行し、走行0、Spring false、belt STOP、
dribble false、operation STOP、emergency stop trueをpublishする。

最後のJoy受信から`joy_timeout_ms`を超えると同じ停止指令を即時publishし、その後は
Joyが復帰するまで状態の周期publishを止める。controller側には最後に送ったSTOPと
emergency stopがtransient local QoSで残る。

ここでの`/emergency_stop`は専用ハードウェアE-stop入力ではなく、
`operation_mode == STOP`をBoolへ変換したものになる。

## callbackの役割

| callback | 実行契機 | 役割 |
|---|---|---|
| `joy_callback` | `/joy`受信時 | 入力を読み取り、button/chordの立ち上がりを検出する。mode、belt、dribbleの内部状態を更新し、`/mecanum/cmd_vel`、shot cycle要求、手動位置指令を即時publishする。 |
| `state_publish_timer_callback` | `state_publish_period_ms`周期 | belt mode、dribble enabled、operation mode、spring fire requestを再送する。spring fire requestはDRIVE中かつL1+○を押している間だけtrueにする。 |
| `joy_timeout_timer_callback` | 10ms周期 | Joy入力断を監視する。最後の入力から`joy_timeout_ms`を超えた場合は、STOPと各停止指令を即時publishする。 |
| `shot_cycle_running_callback` | `/shot_cycle/running`受信時 | shot cycleが実際に動作中かを保持し、動作中はHome以外のmode変更を抑止する。 |
| `shot_cycle_complete_callback` | `/shot_cycle/complete`受信時 | shot cycle完了を受け取り、`auto_drive_on_shot_cycle_complete`に従ってDRIVEへ復帰する。 |

## parameter

| parameter | 型 | 説明 |
|---|---|---|
| `joy_qos_depth` | int | SensorDataQoSで保持するJoy入力件数。0以下なら1 |
| `command_qos_depth` | int | 通常command topicのqueue depth。0以下なら1 |
| `joy_timeout_ms` | `int` | Joy入力断でSTOPへ移るまでの時間[ms] |
| `state_publish_period_ms` | `int` | belt mode、dribble enabled、operation mode、spring fire requestの再送周期[ms] |
| `auto_drive_on_shot_cycle_complete` | `bool` | `true`ならshot cycle完了時にDRIVEへ自動復帰する |
| `axis_deadzone` | double | stick中心を0とする範囲。`[0, 1]`、不正時0.05 |
| `lateral_axis_threshold` | double | 左右を`-1/0/+1`へ丸める閾値。`(0, 1]`、不正時0.7 |
| `axis_on_threshold` | double | trigger・DPADをONとみなす閾値。`(0, 1]`、不正時0.7 |
| `linear_x_limit` | double | 前後速度係数[m/s]。負値・非finiteなら2.0 |
| `linear_y_limit` | double | 左右速度係数[m/s]。負値・非finiteなら2.0 |
| `angular_z_limit` | double | 旋回速度係数[rad/s]。負値・非finiteなら2.0 |
| `linear_x_scale` | double | 前後速度へ追加で掛ける倍率。非finiteなら1 |
| `linear_y_scale` | double | 左右速度へ追加で掛ける倍率。非finiteなら1 |
| `angular_z_scale` | double | 旋回速度へ追加で掛ける倍率。非finiteなら1 |

button・axis indexもすべてparameterである。対応表を変更する場合は
`sensor_msgs/msg/Joy`の実データを確認し、README先頭の配置表と
`robot_bringup/config/joy_controller.yaml`を同時に更新する。

### belt mode

| 値 | mode |
|---:|---|
| 0 | STOP |
| 1 | LEVEL_1 |
| 2 | LEVEL_2 |
| 3 | LEVEL_3 |
| 4 | LEVEL_4 |
| 5 | LEVEL_5 |
| 6 | LEVEL_6 |

### position

| 値 | position |
|---:|---|
| 0 | DRIBBLE |
| 1 | INTAKE |
| 2 | SHOT |
| 3 | OPEN |

## 主なtopic (`joy_controller_node`)

| 種別 | topic | 型 |
|---|---|---|
| publish | `/operation_mode` | `std_msgs/msg/UInt8` |
| publish | `/shot_cycle/request` | `std_msgs/msg/Bool` |
| publish | `/belt/mode` | `std_msgs/msg/UInt8` |
| publish | `/dribble/enabled` | `std_msgs/msg/Bool` |
| publish | `/spring/fire_request` | `std_msgs/msg/Bool` |
| publish | `/dribble/position_mode` | `std_msgs/msg/UInt8` |
| publish | `/joy_controller/cmd_vel` | `geometry_msgs/msg/Twist` |
| publish | `/emergency_stop` | `std_msgs/msg/Bool` |
| subscribe | `/shot_cycle/running` | `std_msgs/msg/Bool` |
| subscribe | `/shot_cycle/complete` | `std_msgs/msg/Bool` |

---

## 速度切替ノード (`cmd_vel_selector_node`)

手動走行 (`/joy_controller/cmd_vel`) と自動制御 (`/auto_game1/cmd_vel`) の2つの速度指令を `/operation_mode` に応じて切替選択し、最終的な足回り出力 (`/mecanum/cmd_vel`) へ転送する非同期マルチプレクサノード。

| 種別 | topic | 型 | 役割 |
|---|---|---|---|
| subscribe | `/joy_controller/cmd_vel` | `geometry_msgs/msg/Twist` | 手動操作速度指令 |
| subscribe | `/auto_game1/cmd_vel` | `geometry_msgs/msg/Twist` | 自動自律走行速度指令 |
| subscribe | `/operation_mode` | `std_msgs/msg/UInt8` | 操作モード (1: DRIVE/手動, 2: SHOT_CYCLE/自動) |
| publish | `/mecanum/cmd_vel` | `geometry_msgs/msg/Twist` | 最終出力速度指令 (`hardware_driver` 宛) |
