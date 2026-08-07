# joy_controller

`sensor_msgs/msg/Joy`を機構指令と4つのoperation modeへ変換する。

## コードを読む順序

`joy_controller`は1パッケージ1nodeのため、このREADMEを個別node資料として扱う。
実装を読むときは次の順が分かりやすい。

1. `include/joy_controller/joy_controller_node.hpp`のenumと内部状態
2. constructorのparameter検証とpub/sub作成
3. `joy_callback()`のJoyメッセージ保存
4. `loop_callback()`のボタン・軸入力からの機構指令変換
5. 2つのtimer callbackの周期publish・通信断処理

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

## 操作

| 入力 | 動作 |
|---|---|
| Home | 非常停止のON/OFFを切替。ON時は走行、belt、dribblerへ停止指令を送る |
| Options | Game2自動戦術モードのON/OFFを切替 |
| DPAD上 / 下 | R2を押していないとき、belt modeをSTOPとLEVEL_1〜LEVEL_6の範囲で増減 |
| R1 | dribbler ON/OFF |
| L1 + R1 + △ | Spring発射要求 |
| L2 + ○ | armの自動shot cycleを要求 |
| DPAD左 | armをOPEN位置へ移動 |
| DPAD右 | armをFEED位置へ移動 |
| PS | `linear.x/y`の前後左右反転 |

Game2モード中にスティック入力があれば、Game2モードを解除して手動走行へ戻る。
Joy通信が途切れた場合は、走行0、belt STOP、dribbler OFFをpublishする。

## Joy入力の処理順

`/joy`を受信すると最新メッセージを保存する。10 ms周期の`loop_callback()`が、保存した入力を次の順で処理する。

1. Homeの立ち上がりで非常停止を切り替える。
2. 非常停止中でなければ、R2の状態とDPAD上下でbelt levelを変更する。
3. R1でdribbler、PSで走行反転、L2+○でshot cycle、OptionsでGame2を操作する。
4. L1+R1+△でSpring発射、DPAD左右でarm位置を切り替える。
5. Game2が有効でなければ、stick入力を`cmd_vel`へ変換してpublishする。
6. 現在の入力を前回値として保存する。

button・DPAD操作は基本的に立ち上がり判定なので、押し続けても連続実行しない。

## stick処理

- 左stick上下はdeadzone適用後の連続値を前後速度へ使う。
- 左stick左右はdeadzone適用後の連続値を左右速度へ使う。
- 右stick左右はdeadzone適用後の連続値を旋回速度へ使う。
- 各値へlimitとscaleを掛け、PS反転は並進2軸だけへ適用する。

入力indexが配列外の場合はbutton=false、axis=0として扱う。NaNやInfのaxisも0として
扱うため、短いJoy messageや壊れたaxis値で範囲外アクセスしない。

## STOPと通信断

起動直後は`publish_stop_commands()`を実行し、走行0、belt STOP、dribbler OFFをpublishする。
`/emergency_stop`は初期値trueで、状態再送timerからpublishされる。

最後のJoy受信から`joy_timeout_ms`を超えると、走行0、belt STOP、dribbler OFFを即時publishする。

ここでの`/emergency_stop`は専用ハードウェアE-stop入力ではなく、Homeで切り替えるソフトウェア上の停止状態である。

## callbackの役割

| callback | 実行契機 | 役割 |
|---|---|---|
| `joy_callback` | `/joy`受信時 | 最新のJoyメッセージと受信時刻を保存する。 |
| `loop_callback` | 10ms周期 | 保存した入力の立ち上がりを判定し、非常停止、機構指令、Game2切替、走行指令を処理する。 |
| `state_publish_timer_callback` | `state_publish_period_ms`周期 | emergency stop、belt mode、dribbler enabledを再送する。 |
| `joy_timeout_timer_callback` | 10ms周期 | Joy入力断を監視し、超過時に走行・belt・dribblerの停止指令を送る。 |

## parameter

| parameter | 型 | 説明 |
|---|---|---|
| `joy_qos_depth` | int | SensorDataQoSで保持するJoy入力件数。0以下なら1 |
| `command_qos_depth` | int | 通常command topicのqueue depth。0以下なら1 |
| `joy_timeout_ms` | `int` | Joy入力断でSTOPへ移るまでの時間[ms] |
| `state_publish_period_ms` | `int` | emergency stop、belt mode、dribbler enabledの再送周期[ms] |
| `axis_deadzone` | double | stick中心を0とする範囲。`[0, 1]`、不正時0.05 |
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
| 1 | OPEN |
| 2 | FEED |

## 主なtopic

| 種別 | topic | 型 |
|---|---|---|
| publish | `/shot_cycle/request` | `std_msgs/msg/Bool` |
| publish | `/belt/mode` | `std_msgs/msg/UInt8` |
| publish | `/dribble/enabled` | `std_msgs/msg/Bool` |
| publish | `/spring/fire_request` | `std_msgs/msg/Bool` |
| publish | `/dribble/position_mode` | `std_msgs/msg/UInt8` |
| publish | `/mecanum/cmd_vel` | `geometry_msgs/msg/Twist` |
| publish | `/emergency_stop` | `std_msgs/msg/Bool` |
| publish | `/game2/start` | `std_msgs/msg/Bool` |
