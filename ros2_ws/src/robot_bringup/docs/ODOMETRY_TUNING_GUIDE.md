# 🏎️ メカナムオドメトリ・スケール＆共分散（Covariance）チューニング手順書

このドキュメントは、ロボットの **実測走行データ** から `velocity_scale`（系統誤差補正）と `twist_covariance`（EKF速度共分散）を誰でも数分で数学的に決定するための実践マニュアルです。

---

## 📍 設定対象ファイル
**`ros2_ws/src/robot_bringup/config/odometry.yaml`**

```yaml
mecanum_odometry_node:
  ros__parameters:
    # ① 系統誤差補正（実測スケール）
    velocity_scale_x: 1.0     # 前後スケール
    velocity_scale_y: 1.0     # 左右カニ歩きスケール
    velocity_scale_yaw: 1.0   # 旋回スケール

    # ② 偶然誤差・不確かさ（EKF速度共分散）
    twist_covariance_x: 0.03   # 前後速度の分散 (m/s)^2
    twist_covariance_y: 0.10   # 左右速度の分散 (m/s)^2
    twist_covariance_yaw: 0.15 # 旋回角速度の分散 (rad/s)^2
```

---

## 🧪 ステップ1: 実機で計測データを取る (前後・左右 各5回)

`game1.yaml` の `test_mode: true` を使って、直進 1.0m と 横移動 1.0m をそれぞれ 5回 走行させ、停止位置をメジャーで測ってメモします。

* **前後 (X軸 1.0m指令)**: `[x1, x2, x3, x4, x5]` (例: `[0.41, 0.39, 0.40, 0.42, 0.38]`)
* **左右 (Y軸 1.0m指令)**: `[y1, y2, y3, y4, y5]` (例: `[0.36, 0.32, 0.35, 0.38, 0.34]`)

---

## 🧮 ステップ2: 計算して YAML に反映する

### 1. `velocity_scale` の決め方 (平均値 $\mu$)
$$\text{velocity\_scale\_x} = \frac{x_1 + x_2 + x_3 + x_4 + x_5}{5}$$
$$\text{velocity\_scale\_y} = \frac{y_1 + y_2 + y_3 + y_4 + y_5}{5}$$

* **効果**: 常に同じ割合で縮む・伸びる系統誤差が消え、1.0m 指令で 1.0m ぴったり止まるようになります。

---

### 2. `twist_covariance` の決め方 (分散 $\sigma^2$ と物理目安)
走るたびのバラつき（分散 $\sigma^2$）から決定します：

$$\sigma^2 = \frac{\sum (x_i - \mu)^2}{N - 1}$$

| 軸 | 推奨設定レンジ | 決定の目安 |
| :--- | :---: | :--- |
| **`twist_covariance_x`** | **`0.02 〜 0.05`** | 前後はグリップが強いため小さく設定 (EKFが車輪オドメトリを強く信頼)。 |
| **`twist_covariance_y`** | **`0.08 〜 0.15`** | メカナムは横滑りしやすいため大きめに設定 (EKFが車輪を過信せずAprilTag/IMUを優先)。 |
| **`twist_covariance_yaw`**| **`0.15 〜 0.30`** | 旋回はIMUジャイロに全権を委ねるため高めに設定。 |

---

## 🐍 Python 自動計算ワンライナー (ターミナルで実行)

実測値を下の配列に入れて RDK X5 または PC のターミナルで叩くだけで即座に出力されます：

```bash
python3 -c '
import numpy as np

# ここに実測値(m)を入れる
x_data = [0.41, 0.39, 0.40, 0.42, 0.38]
y_data = [0.36, 0.32, 0.35, 0.38, 0.34]

mu_x, var_x = np.mean(x_data), np.var(x_data, ddof=1)
mu_y, var_y = np.mean(y_data), np.var(y_data, ddof=1)

print("=== odometry.yaml 反映用パラメータ ===")
print(f"velocity_scale_x: {mu_x:.3f}")
print(f"velocity_scale_y: {mu_y:.3f}")
print(f"twist_covariance_x: {max(0.02, round(var_x * 100, 3)):.3f}")
print(f"twist_covariance_y: {max(0.08, round(var_y * 100, 3)):.3f}")
'
```
