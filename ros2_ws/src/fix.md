# 修正・実装TODO

## [重大] dribble実速度確認後のばね射出

- 現在は、dribble_controllerが減速指令を`0 rad/s`へ到達させたことを停止完了としている。
- STM32からdribbleモータの実速度を取得するtopicを追加し、実測速度が0付近であることを確認してから`spring_controller`が`FIRE`状態へ遷移するようにする。
- springで射出するときに、dribbleの回転数を必ず0にしなくてもよい可能性があるため、実機で必要性を確認する。

## [重大] spring LOAD状態の巻取りタイムアウト

- 現在は`LOAD`状態でリミットスイッチがONになるまで、ばねの巻取り速度を出し続ける。
- リミットスイッチ断線、feedback途絶、機構詰まり時にモータを継続駆動する可能性がある。
- `load_timeout_sec`を追加し、超過時は`0 rad/s`へ停止してエラー状態へ遷移する。復帰方法は実機運用に合わせて明確にする。

## [高] mecanumパラメータの妥当性検証

- `wheel_radius`が0以下の場合に車輪速度計算でゼロ除算となり、無限大または不正な速度指令をpublishする可能性がある。
- `wheel_radius`は正の有限値、車体寸法は非負の有限値、符号・補正係数は有限値であることを起動時に検証する。
- 不正な設定の場合は、各輪へ`0 rad/s`のみをpublishする。

## [高] dribble_controllerの不正周期設定時の安全な起動

- `command_period_ms`が0以下の場合、エラーログは出るが値を安全な既定値へ戻さずにtimerを生成している。
- timer生成に失敗してnodeが起動せず、明示的な停止指令を送れない可能性がある。
- belt_controller、spring_controllerと同様に、不正値検出時は設定を無効扱いにしつつ周期を安全な既定値へ戻す。

## [中] DribblePosition Actionの到達判定とタイムアウト

- `position_tolerance_rad`が0以下の場合、到達判定が成立せずActionが終了しない可能性がある。
- 位置feedbackが途絶えた場合もActionが実行中のまま残る。
- `position_tolerance_rad`を正の有限値として検証し、移動時間またはfeedback受信のタイムアウト時にはActionを失敗終了して安全な位置指令を出す。

## [中] DribblePosition Actionのキャンセル結果

- cancel要求を受理した場合に`abort()`で終了しており、利用側からはキャンセルと異常終了を区別できない。
- 通常のcancelは`canceled()`で終了し、新しいgoalによる置換などの中断だけを`abort()`で終了する。

## [中] joy_controllerの状態topic publish周期

- 現在はJoyメッセージを受信するたびに、`spring_fire_enabled_`、`belt_fire_enabled_`、belt/dribbleのRPM modeをpublishしている。
- これらはボタン操作時だけ変化する状態なので、走行速度指令とは別にtimer callbackで定期publishする、または状態が変化したときだけpublishする方式を検討する。
- 操作遅延とtopic帯域のバランスを確認し、timer周期はYAMLで設定できるようにする。

## [低] joy_controllerのmode enum値の明示

- `BeltRpmMode`と`DribbleRpmMode`は`UInt8`で別nodeへ送る値の契約になっているため、`STOP = 0`などの明示値は無意味ではない。
- enum定義順と受信側のmode値が一致していることを確認したうえで、値を省略するか、通信仕様として明示値を残すかを統一する。

## [低] joy_controllerのJoy用QoS

- 現在は`KeepLast(joy_qos_depth_).best_effort()`を個別に指定している。
- `sensor_msgs/msg/Joy`はセンサ入力なので、`rclcpp::SensorDataQoS()`をベースにし、必要ならdepthだけYAML値で上書きする方式を検討する。

## [低] joy_controllerのcallback分割と早期return

- `joy_callback()`が入力取得、非常停止、状態更新、Action送信、走行速度計算、publish、前回値更新をまとめて担当しており長い。
- 入力状態の取得、非常停止処理、ボタン操作処理、通常指令publishへ関数を分割し、条件に合わない処理は早期returnで抜ける構成を検討する。
- 分割後も、非常停止中に停止指令を継続publishする現在の安全動作は維持する。
