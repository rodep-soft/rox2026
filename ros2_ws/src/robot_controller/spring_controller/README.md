# spring_controller_node

Springの装填、待機、発射、異常停止を管理し、EduLite 05へ速度指令を送るnodeである。
装填完了はSTM32がpublishするlimit switch状態から判断する。

## 関連ファイル

- 実装: `src/spring_edulite_controller.cpp`
- 宣言: `include/spring_controller/spring_edulite_controller.hpp`
- 設定: `robot_bringup/config/spring_controller.yaml`
- 起動: `robot_bringup/launch/controllers/spring_controller.launch.py`

constructorの設定検証後、mode・fire・emergency・limit callbackを読み、
最後に`timer_callback()`のstate switchを読むと全体を把握しやすい。

## 入出力

| 種別 | topic | 型 | 内容 |
|---|---|---|---|
| sub | `/operation_mode` | `UInt8` | 発射可否 |
| sub | `/emergency_stop` | `Bool` | 発射中断 |
| sub | `/spring/fire_request` | `Bool` | 発射要求 |
| sub | `/limit_switchs` | `UInt8` | STM32からのリミットスイッチbit列 |
| pub | `/edulite/target` | `ActuatorTarget` | logical ID 4のEduLite速度[rad/s] |

## 状態遷移

```text
起動 → LOAD ── switch ON ──→ READY
         │                    │
         └ timeout → ERROR    └ fire要求 → FIRE
              │                             │
              └ switch ON → READY           └ duration経過 → LOAD
```

- `LOAD`: switch OFFの間、`loading_velocity_rad_s`を送る。
- `READY`: switch ONで0 rad/s。switchがOFFへ戻るとLOAD。
- `FIRE`: `fire_duration_sec`の間、`fire_velocity_rad_s`を送る。
- `ERROR`: LOAD timeout後に0 rad/s。switch ONだけでREADYへ復帰。

発射要求はfalse→trueの立ち上がりだけを見る。設定正常、DRIVE、非常停止なし、
READY、switch ONの全条件を満たした場合だけ予約される。

## STOP・非常停止の注意

STOP、SHOT_CYCLE、BELT_ONLY、emergency stopではFIREを中断する。ただし未装填なら
`prepare_spring_for_stop()`がLOADを開始するため、Spring motorは0にならず装填方向へ
回り続ける。現在の`/emergency_stop`はSpringの即時motor停止ではない。

## parameterと異常

速度は有限かつ絶対値50 rad/s以下、duration・timeoutは0より大きい必要がある。
設定不正ではnodeは動作を続けるが、毎周期0 rad/sをpublishする。

`limit_switch_bit_offset`で、受信した`/limit_switchs`のbyte内から装填判定に使うbit位置を指定する。
LOADが`load_timeout_sec`を超える場合はERRORログにswitch値と速度を出す。

## 確認方法

```bash
ros2 topic echo /limit_switchs
ros2 topic echo /edulite/target
ros2 topic pub --once /spring/fire_request std_msgs/msg/Bool "{data: true}"
```

単体発射前にDRIVEであること、対象switchがONであること、機構周辺が安全であることを
必ず確認する。
