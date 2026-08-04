# belt_dribble_controller_node

belt 2台とdribble回転1台の目標RPMを作り、3台の実RPMがshot可能範囲へ
入ったことを確認してdribble位置制御へ開始通知を送るnodeである。CANやVESCの
frame形式は扱わない。

## 関連ファイル

- 実装: `src/belt_dribble_controller.cpp`
- 宣言: `include/belt_dribble_controller/belt_dribble_controller.hpp`
- 設定: `robot_bringup/config/belt_dribble_controller.yaml`
- 起動: `robot_bringup/launch/controllers/belt_dribble.launch.py`

コードは、constructor → parameter検証 → `create_interfaces()` →
各callback → `timer_callback()` → shot判定関数の順に読むと追いやすい。

## 入出力

| 種別 | topic | 型 | 内容 |
|---|---|---|---|
| sub | `/operation_mode` | `UInt8` | STOP、DRIVE、SHOT_CYCLE、BELT_ONLY |
| sub | `/emergency_stop` | `Bool` | trueなら全targetを0 |
| sub | `/belt/mode` | `UInt8` | STOP、LEVEL_1〜6 |
| sub | `/dribble/enabled` | `Bool` | dribble回転ON/OFF |
| sub | `/shot_cycle/request` | `Bool` | shot実行要求 |
| sub | `/*/current/rpm` | `Int16` | under、upper、dribble実RPM |
| pub | `/*/target/rpm` | `Int16` | VESCへ送る3台の目標RPM |
| pub | `/shot_cycle/start` | `Bool` | 位置シーケンス開始パルス |

operation modeとemergency stopはreliable・transient local、それ以外は
`qos_depth`を使う。

## 周期処理

`command_period_ms`周期で必ず3つのtargetをpublishする。

1. 設定不正、emergency stop、STOPなら3台とも0 RPM。
2. それ以外はbelt levelからunder・upper共通RPMを選ぶ。
3. dribble無効またはBELT_ONLYならdribbleだけ0 RPM。
4. feedback timeoutを更新する。
5. SHOT_CYCLE中だけshot準備状態を計算する。

mode callback自身はRPMをpublishしない。mode変更から次のtimer、現在最大20 ms後に
新しい指令が反映される。DRIVEとSHOT_CYCLEの間に0 RPMは挟まない。

## shot可能判定

次をすべて満たした時刻を`ready_since_`へ保存し、その状態が
`ready_hold_sec`続くと`shoot_ready_`がtrueになる。

- SHOT_CYCLE
- emergency stopではない
- 3台すべてのfeedbackが`feedback_timeout_sec`以内
- under・upperがbelt target ± `belt_rpm_tolerance`
- dribbleがdribble target ± `dribble_rpm_tolerance`

mode、belt level、dribble ON/OFF、非常停止、RPM許容範囲のいずれかが変わると
保持時間を破棄する。要求を先行予約しないため、準備前に押したshot要求は無視される。

targetが0、feedback断、RPM未到達、保持時間中など、拒否理由は要求時にログへ出る。

## parameterと安全動作

RPMはInt16範囲内、toleranceと保持時間は0以上、feedback timeoutは0より大きい
必要がある。不正設定ではnodeを終了せず、3台のtargetを0に固定する。
`command_period_ms<=0`は10 msへ、`qos_depth<=0`は1へ補正する。

現在値は`robot_bringup/config/belt_dribble_controller.yaml`を参照する。

## 確認方法

```bash
ros2 topic echo /underbelt/target/rpm
ros2 topic echo /underbelt/current/rpm
ros2 topic echo /shot_cycle/start
```

shotできない場合は、operation mode、belt/dribble target、3 current RPM、
feedback timeoutログの順に確認する。
