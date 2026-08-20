#!/usr/bin/env python3
"""
analyze_ball_threshold.py
-------------------------
ROS 2 bag / ログデータからドリブルローラーの電流値を解析し、
統計的最適閾値（ball_detection_threshold_a, ball_lost_threshold_a）を自動算出するツール。

使い方:
  python3 analyze_ball_threshold.py <rosbag_path_or_folder> [--plot]
"""

import argparse
import os
import sys
import numpy as np


def analyze_current_distribution(currents: np.ndarray, plot: bool = False):
    if len(currents) < 20:
        print(
            f"[ERROR] サンプル数が少なすぎます ({len(currents)} 件)。最低でも20サンプル以上記録してください。"
        )
        return None

    # 外れ値除去 (0.0以下や極端な過電流)
    valid_currents = currents[(currents >= 0.0) & (currents <= 30.0)]
    if len(valid_currents) < 10:
        print("[ERROR] 有効な電流データが存在しません。")
        return None

    # 1. 大津の2値化法 (Otsu's Thresholding) で2峰性の中間境界を大域探索
    hist, bin_edges = np.histogram(valid_currents, bins=100)
    bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2.0
    total = len(valid_currents)

    current_max = 0.0
    otsu_threshold = bin_centers[0]

    weight_b = 0.0
    sum_b = 0.0
    sum_total = np.sum(valid_currents)

    for i in range(len(hist)):
        weight_b += hist[i]
        if weight_b == 0:
            continue
        weight_f = total - weight_b
        if weight_f == 0:
            break

        sum_b += hist[i] * bin_centers[i]
        mean_b = sum_b / weight_b
        mean_f = (sum_total - sum_b) / weight_f

        # クラス間分散
        var_between = weight_b * weight_f * ((mean_b - mean_f) ** 2)
        if var_between > current_max:
            current_max = var_between
            otsu_threshold = bin_centers[i]

    # 2. クラスタごとの統計量 (空転時クラス0 vs ボール保持時クラス1)
    empty_class = valid_currents[valid_currents < otsu_threshold]
    ball_class = valid_currents[valid_currents >= otsu_threshold]

    if len(empty_class) == 0 or len(ball_class) == 0:
        print(
            "[WARN] 1つのクラスしか検出されませんでした (空転のみ、または保持のみのデータ)。"
        )
        mu = np.mean(valid_currents)
        sigma = np.std(valid_currents)
        print(f"全体平均: {mu:.2f} A, 標準偏差: ±{sigma:.2f} A")
        return None

    mu_empty = np.mean(empty_class)
    sigma_empty = np.std(empty_class)
    mu_ball = np.mean(ball_class)
    sigma_ball = np.std(ball_class)

    # 3. 統計的最適閾値の導出
    #   - 解除閾値 (T_lost): 空転時の上位99%点 (mu_empty + 2.0 * sigma_empty)
    #   - 検知閾値 (T_detect): 空転誤検知率 < 0.05% の安全ライン (mu_empty + 3.5 * sigma_empty と大津境界の安全側)
    t_lost = max(0.6, mu_empty + 2.0 * sigma_empty)
    t_detect = max(t_lost + 0.3, min(mu_empty + 3.5 * sigma_empty, otsu_threshold))

    # 安全マージン確保
    if t_detect >= mu_ball:
        t_detect = (mu_empty + mu_ball) / 2.0

    print("\n============================================================")
    print(" 📊 ドリブルローラー電流データ 統計解析レポート")
    print("============================================================")
    print(f" 総サンプル数     : {len(valid_currents):,} 点")
    print(
        f" 🌀 空転時 (No Ball) : 平均 {mu_empty:.3f} A | 標準偏差 ±{sigma_empty:.3f} A (Max: {np.max(empty_class):.2f} A)"
    )
    print(
        f" ⚽ 保持時 (Has Ball): 平均 {mu_ball:.3f} A | 標準偏差 ±{sigma_ball:.3f} A (Min: {np.min(ball_class):.2f} A)"
    )
    print(f" ⚖️ 大津の判別境界  : {otsu_threshold:.3f} A")
    print("------------------------------------------------------------")
    print(" 🎯 推奨 YAML パラメータ (dribble_controller.yaml):")
    print(f"    ball_detection_threshold_a: {t_detect:.2f}   # 検知閾値 [A]")
    print(f"    ball_lost_threshold_a:      {t_lost:.2f}   # 解除閾値 [A]")
    print("============================================================\n")

    # ASCII ヒストグラム表示
    print("📈 電流値 分布ヒストグラム:")
    h_bins = 20
    counts, edges = np.histogram(valid_currents, bins=h_bins)
    max_count = max(counts) if max(counts) > 0 else 1
    for c, e0, e1 in zip(counts, edges[:-1], edges[1:]):
        bar = "█" * int(c / max_count * 40)
        marker = ""
        if e0 <= t_lost < e1:
            marker += " ◄─ T_lost"
        if e0 <= t_detect < e1:
            marker += " ◄─ T_detect"
        print(f" [{e0:4.1f}A - {e1:4.1f}A] {bar:<40} ({c:4d}){marker}")

    if plot:
        try:
            import matplotlib.pyplot as plt

            plt.figure(figsize=(10, 5))
            plt.hist(
                empty_class,
                bins=50,
                alpha=0.6,
                color="blue",
                label=f"Empty (μ={mu_empty:.2f}A, σ={sigma_empty:.2f}A)",
            )
            plt.hist(
                ball_class,
                bins=50,
                alpha=0.6,
                color="red",
                label=f"Has Ball (μ={mu_ball:.2f}A, σ={sigma_ball:.2f}A)",
            )
            plt.axvline(
                t_lost,
                color="orange",
                linestyle="--",
                linewidth=2,
                label=f"T_lost = {t_lost:.2f}A",
            )
            plt.axvline(
                t_detect,
                color="green",
                linestyle="--",
                linewidth=2,
                label=f"T_detect = {t_detect:.2f}A",
            )
            plt.title("Dribble Roller Current Distribution & Optimal Thresholds")
            plt.xlabel("Motor Current [A]")
            plt.ylabel("Count")
            plt.legend()
            plt.grid(True, alpha=0.3)
            plot_file = "dribble_current_distribution.png"
            plt.savefig(plot_file, dpi=150)
            print(f"\n[INFO] プロット画像を保存しました: {plot_file}")
            plt.close()
        except ImportError:
            print(
                "\n[INFO] matplotlib がインストールされていないため画像保存をスキップしました。"
            )

    return t_detect, t_lost


def load_from_rosbag(bag_path: str, logical_id: int = 12):
    currents = []
    print(
        f"[INFO] Loading rosbag from: {bag_path} (Filtering logical_id={logical_id})..."
    )

    # 1. rosbag2_py を利用して読み込み
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        storage_options = StorageOptions(uri=bag_path)
        converter_options = ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )
        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        topics_and_types = reader.get_all_topics_and_types()
        type_map = {t.name: t.type for t in topics_and_types}

        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic in ["/vesc/state", "/vesc/state_array"]:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)

                if hasattr(msg, "logical_id"):
                    if msg.logical_id == logical_id:
                        currents.append(msg.current_a)
                elif hasattr(msg, "actuators"):
                    for act in msg.actuators:
                        if act.logical_id == logical_id:
                            currents.append(act.current_a)

    except Exception as e:
        print(
            f"[WARN] rosbag2_pyでの読み込みに失敗しました ({e})。sqlite3 直接パースを試行します..."
        )
        # 2. sqlite3 直接パースフォールバック
        import glob
        import sqlite3
        import struct

        db_files = glob.glob(os.path.join(bag_path, "*.db3"))
        if not db_files and bag_path.endswith(".db3"):
            db_files = [bag_path]

        if not db_files:
            print(f"[ERROR] db3 ファイルが見つかりませんでした: {bag_path}")
            return np.array([])

        for db_file in db_files:
            conn = sqlite3.connect(db_file)
            cursor = conn.cursor()
            try:
                # topics テーブルから topic_id を特定
                cursor.execute(
                    "SELECT id FROM topics WHERE name = '/vesc/state' OR name = '/vesc/state_array'"
                )
                topic_ids = [row[0] for row in cursor.fetchall()]

                for tid in topic_ids:
                    cursor.execute(
                        "SELECT data FROM messages WHERE topic_id = ?", (tid,)
                    )
                    for (blob,) in cursor.fetchall():
                        # CDR デシリアライズ: ActuatorState (header 4B, logical_id 2B, ... float current_a)
                        # 簡単なヒューリスティック抽出
                        if len(blob) >= 20:
                            # 探索
                            for offset in range(4, len(blob) - 8, 2):
                                lid = struct.unpack_from("<H", blob, offset)[0]
                                if lid == logical_id:
                                    # current_a は position(4B), velocity(4B), current_a(4B)
                                    if offset + 14 <= len(blob):
                                        curr = struct.unpack_from(
                                            "<f", blob, offset + 10
                                        )[0]
                                        if 0.0 <= curr <= 30.0:
                                            currents.append(curr)
                                    break
            finally:
                conn.close()

    return np.array(currents)


def main():
    parser = argparse.ArgumentParser(
        description="Analyze Dribble Current Threshold from ROS 2 Bag"
    )
    parser.add_argument(
        "bag_path", help="Path to ROS 2 bag folder or .db3 file (or CSV file)"
    )
    parser.add_argument(
        "--id", type=int, default=12, help="Logical ID of Dribble Roller (default: 12)"
    )
    parser.add_argument(
        "--plot", action="store_true", help="Save distribution plot image (matplotlib)"
    )
    args = parser.parse_args()

    if args.bag_path.endswith(".csv"):
        data = np.loadtxt(args.bag_path, delimiter=",")
        if data.ndim > 1:
            data = data[:, 0]
        currents = data
    else:
        currents = load_from_rosbag(args.bag_path, logical_id=args.id)

    print(f"[INFO] 取得したサンプル数: {len(currents)} 件")
    analyze_current_distribution(currents, plot=args.plot)


if __name__ == "__main__":
    main()
