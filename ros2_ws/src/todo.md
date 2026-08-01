# 実機調整TODO

試合ごとの運用値ではなく、機体・機構に合わせて事前に確認するparameterを記録する。

## Joyとmecanum

- `joy_controller.yaml`: `axis_deadzone`、`axis_on_threshold`、`linear_x_limit`、`linear_y_limit`、`angular_z_limit`。
- `mecanum_controller.yaml`: `wheel_radius`、`robot_length`、`robot_width`、`velocity_corrections`、各`*_sign`。

## Beltとdribble

- `belt_dribble_controller.yaml`: `level_1_rpm`から`level_6_rpm`、`dribble_on_rpm`。
- `belt_dribble_controller.yaml`: `belt_rpm_tolerance`、`dribble_rpm_tolerance`、`ready_hold_sec`。
- `hardware.launch.py`: underbelt / upperbelt / dribbleに割り当てたVESC 1 / 2 / 3の
  `controller_id`が実機ID（現在の初期値は`51` / `2` / `3`）と一致すること。

## Spring

- `spring_controller.yaml`: `limit_switch_bit_offset`がSTM32から受けたbyte内の実機のリミットスイッチ位置と一致すること。
- `spring_controller.yaml`: `loading_velocity_rad_s`、`fire_velocity_rad_s`、`fire_duration_sec`。
- `spring_controller.yaml`: `load_timeout_sec`が機構の通常装填時間より十分長く、安全な値であること。

## Dribble position

- `dribble_position_controller.yaml`: `dribble_position_rad`、`intake_position_rad`、`shoot_position_rad`、`open_position_rad`。
- `dribble_position_controller.yaml`: `position_tolerance_rad`、`shoot_to_dribble_delay_sec`、`move_timeout_sec`、`feedback_timeout_sec`。
