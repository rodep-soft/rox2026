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

## 操作一覧

| 操作 (ボタン・コンボ) | 機能・動作 |
|---|---|
| **タッチパッド (Home / Button 13)** | **非常停止 (TOGGLE)** (ACTIVE ↔ STOP) |
| **Share / Create ボタン (Button 8)** | **Heading Hold (IMU姿勢補正) ON / OFF トグル** (異常時等の手動直結バイパス) |
| **PS ボタン (Button 12)** | **操縦 前後反転** (FORWARD ↔ REVERSED) |
| **Options ボタン (Button 9)** | **Game 2 自動戦術モード ON / OFF** (手動スティック入力で自動解除) |
| **R1** | **ドリブラー正回転 ON / OFF (トグル)** |
| **L1** | **スプリングゆっくり射出** (アームをOPENに遷移、ばねを逆回転13.5 radまで押し出し ➔ 完了後自動でDRIBBLE復帰、ドリブル回転なし) |
| **DPAD 上 / 下** *(R2非押下時)* | **射出ベルト速度レベル 変更** (`STOP` ↔ `LEVEL_1` 〜 `LEVEL_4`) |
| **DPAD 左 / 右** *(R2非押下時)* | **自動シュート(Shot Cycle)時の待機回転数 変更** (`+200 RPM` / `-200 RPM`) |
| **L2 + ○** | **自動シュート(Shot Cycle) 実行要求** |
| **L2 + R2 同時押し** | **キッカー（ばね）発射** (`spring_arm_open_delay_ms` 後にアームをOPENへ遷移させ、ばねを発射 ➔ 発射完了後に自動でDRIBBLE姿勢へ復帰) |
| **R2 + DPAD 右** | **【手動アーム操作】 OPEN** (アームを開く) |
| **R2 + DPAD 左** | **【手動アーム操作】 DRIBBLE** (アームをドリブル位置に戻す) |
| **左スティック (上下/左右)** | 前後 / 左右の並進移動 |
| **右スティック (左右)** | 旋回動作 |
| **R2 + スティック (並進・旋回)** | **低速走行・低速旋回** (線速度は `slow_linear_scale` (初期値0.5)、旋回速度は `slow_turn_scale` (初期値0.5) で倍率変更可) |

## Joy入力の処理順

`/joy`を受信すると最新メッセージを保存する。10 ms周期の`loop_callback()`が、保存した入力を次の順で処理する。

1. Homeの立ち上がりで非常停止を切り替える。
2. 非常停止中でなければ、R2の状態とDPAD上下でbelt levelを変更する。
3. R1でdribbler、PSで走行反転、L2+○でshot cycle、OptionsでGame2を操作する。
4. L1+R1+△でSpring発射、DPAD左右でarm位置を切り替える。
5. Game2が有効でなければ、stick入力を`cmd_vel`へ変換してpublishする。
6. 現在の入力を前回値として保存する。

button操作は基本的に立ち上がり判定なので、押し続けてもmodeやlevelは連続変化しない。
SpringはL2とR2が同時に押された瞬間だけtrueを送り、押し続けても再発射しない。
再発射する場合は、どちらか一方を離してから再度同時押しする。

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
| `joy_callback` | `/joy`受信時 | 入力を読み取り、button/chordの立ち上がりを検出する。mode、belt、dribbleの内部状態を更新し、`/mecanum/cmd_vel`、shot cycle要求、手動位置指令を即時publishする。 |
| `state_publish_timer_callback` | `state_publish_period_ms`周期 | emergency stop、belt mode、dribble enabledを再送する。 |
| `joy_timeout_timer_callback` | 10ms周期 | Joy入力断を監視する。最後の入力から`joy_timeout_ms`を超えた場合は、STOPと各停止指令を即時publishする。 |
| `shot_cycle_running_callback` | `/shot_cycle/running`受信時 | shot cycleが実際に動作中かを保持し、動作中はHome以外のmode変更を抑止する。 |
| `shot_cycle_complete_callback` | `/shot_cycle/complete`受信時 | shot cycle完了を受け取り、`auto_drive_on_shot_cycle_complete`に従ってDRIVEへ復帰する。 |

## parameter

| parameter | 型 | 説明 |
|---|---|---|
| `command_qos_depth` | int | 通常command topicのqueue depth。0以下なら1 |
| `joy_timeout_ms` | `int` | Joy入力断でSTOPへ移るまでの時間[ms] |
| `state_publish_period_ms` | int | emergency stop、belt mode、dribbler enabledの再送周期[ms] |
| `dribble_enable_button` | int | dribble ON/OFFのbutton index |
| `axis_deadzone` | double | stick中心を0とする範囲。`[0, 1]`、不正時0.05 |
| `axis_on_threshold` | double | trigger・DPADをONとみなす閾値。`(0, 1]`、不正時0.7 |
| `linear_x_limit` | double | スティック全倒し時の最大前後速度[m/s] |
| `linear_y_limit` | double | スティック全倒し時の最大左右速度[m/s] |
| `angular_z_limit` | double | スティック全倒し時の最大旋回速度[rad/s] |
| `slow_turn_button` | int | 低速走行・低速旋回を有効化するボタン番号。デフォルト: 7 (R2)、-1で無効 |
| `slow_turn_scale` | double | 低速旋回時の旋回速度倍率（減速率）。デフォルト: 0.5 (1/2) |
| `slow_linear_scale` | double | 低速走行時の並進（前後・左右）線速度倍率（減速率）。デフォルト: 0.5 (1/2) |

Joyの各軸は通常`-1.0`から`1.0`であるため、各`*_limit`を直接掛けて`cmd_vel`へ
変換する。同じ値で出力を制限するため、最大速度を変更するときに調整するparameterは
軸ごとに1つだけである。この3つの速度parameterは実行中にも変更できる。

`/joy`のsubscriptionには、最新入力を優先する`SensorDataQoS`を使用する。

### 実行中のYAML再読み込み

次のコマンドで、速度、Joy timeout、deadzone、しきい値、button・axis配置を
実行中のnodeへまとめて反映できる。

```bash
ros2 param load /joy_controller \
  src/robot_bringup/config/joy_controller.yaml
```

`command_qos_depth`と`state_publish_period_ms`はROS interfaceまたはtimerの再生成が
必要になるため、実行中には変更できない。YAML内の値が現在値と同じ場合は、そのまま
読み込みを許可する。

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
| publish | `/dribble/command_enabled` | `std_msgs/msg/Bool` |
| publish | `/spring/fire_request` | `std_msgs/msg/Bool` |
| publish | `/dribble/position_mode` | `std_msgs/msg/UInt8` |
| publish | `/mecanum/cmd_vel` | `geometry_msgs/msg/Twist` |
| publish | `/emergency_stop` | `std_msgs/msg/Bool` |
| publish | `/game2/start` | `std_msgs/msg/Bool` |
| publish | `/heading_control/enable` | `std_msgs/msg/Bool` |
| publish | `/drive/reversed` | `std_msgs/msg/Bool` |
