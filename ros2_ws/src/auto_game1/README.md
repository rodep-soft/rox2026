# auto_game1

`auto_game1` は、Nav2（Navigation2）と連動して自律走行と状態遷移（試合シナリオ制御）を行う ROS 2 パッケージです。

---

## 1. 概要

本パッケージ（`auto_game1_node`）は、試合中の状況に応じた状態遷移（ステートマシン）を管理し、Nav2 に移動目標（Goal Pose / Waypoint）を発行して自律走行を行います。
また、3つの長方形領域を障害物ポリゴン（`PolygonStamped`）として周期的（20Hz）に publish し、Nav2 の `PolygonLayer` 経由で動的な障害物・進入禁止領域として認識・回避させます。

---

## 2. 状態遷移マシン (State Machine)

| ステート名 | 概要・動作 | 次のステートへの遷移条件 |
| :--- | :--- | :--- |
| `AUTO_STOP` | 自動停止状態（全出力停止、Nav2キャンセル）。 | Joy入力 (`auto_stop_toggle_button`) の立ち上がりで再開 |
| `GO_TO_WAYPOINT1` | **Nav2 Action** (`NavigateThroughPoses`) を使用して通過点1へ向かう。 | 通過点1との距離 $\le$ `waypoint1_reach_threshold` [m] |
| `PREPARE_KICK` | 通過点1到達後、独自Twist制御で定速直進。**Kick Action** を送信。 | Kick Action の完了 (`Succeeded`) |
| `GO_TO_GATE_FAR_SIDE` | **Nav2 Action** (`NavigateThroughPoses`) で残りの通過点およびゲート向こう側へ向かう。 | Nav2 到着完了 (`Succeeded`) |
| `FOLLOW_BALL` | ボール追従（拡張用プレースホルダー）。 | 即時移行 |
| `CARRY_BALL_TO_PASS_AREA` | **Nav2 Action** (`NavigateToPose`) でパスエリアへ移動。 | Nav2 到着完了 (`Succeeded`) |
| `RETURN_TO_START` | **Nav2 Action** (`NavigateToPose`) でスタート位置に戻りループ。 | Nav2 到着完了 (`Succeeded`) $\rightarrow$ `GO_TO_WAYPOINT1` |

※ジョイスティック (`/joy`) から `return_to_start_button` が押された場合は、いつでも強制的に `RETURN_TO_START` に遷移します。

---

## 3. 長方形障害物ポリゴンの設定と書き換え方法

ナビゲーションで回避させたい 3つの長方形障害物領域（4頂点 $p_1, p_2, p_3, p_4$）は、[`include/auto_game1/auto_game1_node.hpp`](include/auto_game1/auto_game1_node.hpp) 内の定数配列 `RECTANGLE_OBSTACLES[3]` として定義されています。

### 座標の修正方法
障害物の座標を変更したい場合は、[`include/auto_game1/auto_game1_node.hpp`](include/auto_game1/auto_game1_node.hpp) 内の該当数値を直接書き換えて再ビルドしてください。

```cpp
// include/auto_game1/auto_game1_node.hpp より抜粋

// ナビゲーション回避用の障害物となる3つの長方形の定数座標定義
const RectObstacle RECTANGLE_OBSTACLES[3] = {
  // 長方形 1 (4頂点: p1, p2, p3, p4)
  { {0.5f, 0.5f}, {1.5f, 0.5f}, {1.5f, 1.5f}, {0.5f, 1.5f} },
  // 長方形 2
  { {2.0f, 1.0f}, {3.0f, 1.0f}, {3.0f, 2.0f}, {2.0f, 2.0f} },
  // 長方形 3
  { {1.0f, -1.5f}, {2.0f, -1.5f}, {2.0f, -0.5f}, {1.0f, -0.5f} }
};
```

---

## 4. 通信仕様

### Subscription (受信トピック)
* `/joy` (`sensor_msgs/msg/Joy`): ジョイスティック操作入力（自動停止切替・スタート復帰ボタンの判定）

### Publisher (送信トピック)
* `/mecanum/cmd_vel` (`geometry_msgs/msg/Twist`): 走行速度指令（`robot_controller` または `hardware_driver` 宛）
* `/obstacle_polygon_1` (`geometry_msgs/msg/PolygonStamped`): 長方形障害物 1
* `/obstacle_polygon_2` (`geometry_msgs/msg/PolygonStamped`): 長方形障害物 2
* `/obstacle_polygon_3` (`geometry_msgs/msg/PolygonStamped`): 長方形障害物 3

### Action Client (アクション送信)
* `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`): 単一目標地点へのナビゲーション
* `navigate_through_poses` (`nav2_msgs/action/NavigateThroughPoses`): 複数通過点へのナビゲーション
* `kick` (`auto_game1/action/Kick`): キック機構制御

---

## 5. 設定ファイルおよび起動手順

Launch スクリプトおよび各パラメータファイルは **`robot_bringup`** パッケージ内に集約されています。

* **パラメータファイル**:
  * `robot_bringup/config/auto_game1.yaml` (`auto_game1_node` のパラメータ)
  * `robot_bringup/config/auto_game1_nav2.yaml` (Nav2 コストマップ・PolygonLayer のパラメータ)
* **Launch スクリプト**:
  * `robot_bringup/launch/auto_game1.launch.py`

### ビルド
```bash
cd ~/rox/rox2026/ros2_ws
colcon build --packages-select robot_bringup auto_game1
source install/setup.bash
```

### 起動コマンド
```bash
ros2 launch robot_bringup auto_game1.launch.py
```
