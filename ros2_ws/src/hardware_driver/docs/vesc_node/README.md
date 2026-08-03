# vesc_node

機械RPMとVESC CAN protocolのERPMを相互変換するdriver nodeである。同じ実行ファイルを
node名とYAML parameterだけ変えてunderbelt、upperbelt、dribbleの3台へ使用する。

## 関連ファイル

- node: `hardware_driver/src/nodes/vesc_node.cpp`
- protocol: `hardware_driver/include/vesc_driver/vesc_protocol.hpp`
- 設定: `robot_bringup/config/vesc_driver.yaml`
- 起動: `robot_bringup/launch/hardware.launch.py`

nodeのconstructorでparameterとROS interface、protocolヘッダでCAN encode/decodeを読む。
次に`target_rpm_callback()`、`can_callback()`、`timer_callback()`の順に読むとよい。

## node割り当て

| node | 機構 | CAN ID | target/current |
|---|---|---:|---|
| `vesc_upper_belt_driver` | upperbelt | 51 | `/upperbelt/.../rpm` |
| `vesc_under_belt_driver` | underbelt | 52 | `/underbelt/.../rpm` |
| `vesc_dribble_driver` | dribble | 50 | `/dribble/.../rpm` |

値はYAMLを正とし、実機ID変更時はREADMEとYAMLを同時に確認する。

## command処理

`std_msgs/msg/Int16`の機械RPMを受け、絶対値が`max_rpm`以内なら保存する。
20 ms timerで`rpm_slew_rate`の範囲内で目標へ近づけ、極対数7を掛けてERPMへ
変換し、extended CANのSET_RPMを送る。
SET_RPM IDは`(3 << 8) | controller_id`、DLC 4、dataはbig-endianである。

最後の有効commandから`command_timeout_ms`を超えると0 ERPMを周期送信する。
一度もcommandを受けていない場合はCAN frameを送らない。

## feedback処理

extended CAN、DLC 8、packet ID 9のSTATUS_1だけをdecodeする。controller IDが
自nodeと違うframeは無視する。ERPMを7で割り、四捨五入してInt16へ制限する。

起動または最終feedbackから`feedback_timeout_ms`を超えるとcurrent RPMのpublishを
停止する。0や古い値を代わりに出さない。復帰時はINFOを出して再開する。

## parameter不正

- `controller_id`: 0〜255
- timeout: 1 ms以上
- `max_rpm`: 1〜32767
- `rpm_slew_rate`: 1秒あたりのRPM変化量

違反時は例外でnode起動を失敗させる。RPM commandの上限超過はnodeを止めず、
そのmessageだけをWARN付きで破棄する。

## 調査

targetは出るが回らない場合、CAN ID、上限拒否WARN、SET_RPM frameを確認する。
currentが出ない場合、VESCのSTATUS_1周期送信、ID、feedback timeoutを確認する。
