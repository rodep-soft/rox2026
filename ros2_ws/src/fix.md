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
