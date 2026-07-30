# 修正・実装TODO

## [高] mecanumパラメータの妥当性検証

- `wheel_radius`が0以下の場合に車輪速度計算でゼロ除算となり、無限大または不正な速度指令をpublishする可能性がある。
- `wheel_radius`は正の有限値、車体寸法は非負の有限値、符号・補正係数は有限値であることを起動時に検証する。
- 不正な設定の場合は、各輪へ`0 rad/s`のみをpublishする。

## [高] dribble_controllerの不正周期設定時の安全な起動

- `command_period_ms`が0以下の場合、エラーログは出るが値を安全な既定値へ戻さずにtimerを生成している。
- timer生成に失敗してnodeが起動せず、明示的な停止指令を送れない可能性がある。
- belt_controller、spring_controllerと同様に、不正値検出時は設定を無効扱いにしつつ周期を安全な既定値へ戻す。
