#include "game2_shooter/target_selector.hpp"
#include <cmath>

namespace robot_controller
{

double TargetSelector::compute_heading_error(
  double target_x, double target_y,
  double shooter_x, double shooter_y)
{
  const double dx = target_x - shooter_x;
  const double dy = target_y - shooter_y;
  return std::remainder(std::atan2(dy, dx) - M_PI, 2.0 * M_PI);
}

std::vector<TargetCandidate> TargetSelector::build_candidates(
  const std::unordered_map<int, PanelTagInfo> & panel_grid,
  const TargetSelectionConfig & config)
{
  std::vector<TargetCandidate> candidates;

  for (int row = 0; row <= 2; ++row) {
    const PanelTagInfo * p0 = nullptr;
    const PanelTagInfo * p1 = nullptr;
    const PanelTagInfo * p2 = nullptr;

    for (const auto & [id, panel] : panel_grid) {
      if (panel.row == row && !panel.shot_completed && panel.detected) {
        if (panel.col == 0) p0 = &panel;
        else if (panel.col == 1) p1 = &panel;
        else if (panel.col == 2) p2 = &panel;
      }
    }

    if (!p0 && !p1 && !p2) {
      continue;
    }

    if (config.enable_double_panel_midpoint_targeting) {
      // 3枚残存時: 左側2枚(0&1)の中点ペア + 右側1枚(2)の単独候補
      if (p0 && p1 && p2) {
        TargetCandidate c_pair;
        c_pair.row = row;
        c_pair.tag_ids = {p0->tag_id, p1->tag_id};
        c_pair.x = (p0->x + p1->x) * 0.5;
        c_pair.y = (p0->y + p1->y) * 0.5;
        c_pair.z = (p0->z + p1->z) * 0.5;
        c_pair.heading_err = compute_heading_error(c_pair.x, c_pair.y, config.shooter_offset_x, config.shooter_offset_y);
        c_pair.is_pair = true;
        c_pair.desc = "Row " + std::to_string(row) + " Left-Midpoint #" + std::to_string(p0->tag_id) +
          " & #" + std::to_string(p1->tag_id);
        candidates.push_back(c_pair);

        TargetCandidate c_single;
        c_single.row = row;
        c_single.tag_ids = {p2->tag_id};
        c_single.x = p2->x;
        c_single.y = p2->y;
        c_single.z = p2->z;
        c_single.heading_err = compute_heading_error(c_single.x, c_single.y, config.shooter_offset_x, config.shooter_offset_y);
        c_single.is_pair = false;
        c_single.desc = "Row " + std::to_string(row) + " Right-Single Tag #" + std::to_string(p2->tag_id);
        candidates.push_back(c_single);
      } else if (p0 && p1) {
        TargetCandidate c;
        c.row = row;
        c.tag_ids = {p0->tag_id, p1->tag_id};
        c.x = (p0->x + p1->x) * 0.5;
        c.y = (p0->y + p1->y) * 0.5;
        c.z = (p0->z + p1->z) * 0.5;
        c.heading_err = compute_heading_error(c.x, c.y, config.shooter_offset_x, config.shooter_offset_y);
        c.is_pair = true;
        c.desc = "Row " + std::to_string(row) + " Left-Midpoint #" + std::to_string(p0->tag_id) +
          " & #" + std::to_string(p1->tag_id);
        candidates.push_back(c);
      } else if (p1 && p2) {
        TargetCandidate c;
        c.row = row;
        c.tag_ids = {p1->tag_id, p2->tag_id};
        c.x = (p1->x + p2->x) * 0.5;
        c.y = (p1->y + p2->y) * 0.5;
        c.z = (p1->z + p2->z) * 0.5;
        c.heading_err = compute_heading_error(c.x, c.y, config.shooter_offset_x, config.shooter_offset_y);
        c.is_pair = true;
        c.desc = "Row " + std::to_string(row) + " Right-Midpoint #" + std::to_string(p1->tag_id) +
          " & #" + std::to_string(p2->tag_id);
        candidates.push_back(c);
      } else {
        // 単独残存
        const PanelTagInfo * single = p0 ? p0 : (p1 ? p1 : p2);
        if (single) {
          TargetCandidate c;
          c.row = row;
          c.tag_ids = {single->tag_id};
          c.x = single->x;
          c.y = single->y;
          c.z = single->z;
          c.heading_err = compute_heading_error(c.x, c.y, config.shooter_offset_x, config.shooter_offset_y);
          c.is_pair = false;
          c.desc = "Row " + std::to_string(row) + " Single Tag #" + std::to_string(single->tag_id);
          candidates.push_back(c);
        }
      }
    } else {
      // 中点無効時: 全ての未撃破パネルを個別登録
      for (const auto * p : {p0, p1, p2}) {
        if (p) {
          TargetCandidate c;
          c.row = row;
          c.tag_ids = {p->tag_id};
          c.x = p->x;
          c.y = p->y;
          c.z = p->z;
          c.heading_err = compute_heading_error(c.x, c.y, config.shooter_offset_x, config.shooter_offset_y);
          c.is_pair = false;
          c.desc = "Row " + std::to_string(row) + " Single Tag #" + std::to_string(p->tag_id);
          candidates.push_back(c);
        }
      }
    }
  }

  return candidates;
}

std::optional<TargetCandidate> TargetSelector::select_best_target(
  const std::unordered_map<int, PanelTagInfo> & panel_grid,
  int current_active_row,
  double current_target_heading_err,
  const std::vector<int> & current_target_tag_ids,
  const TargetSelectionConfig & config,
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock)
{
  const auto candidates = build_candidates(panel_grid, config);
  if (candidates.empty()) {
    return std::nullopt;
  }

  // 【最優先ルール 1: 垂直スイープ / 旋回0度ロック】
  // 直前と同じ方位 (差が0.08rad以内) に未撃破ターゲットがあれば、旋回0度・RPM変更のみで連続射出
  if (!current_target_tag_ids.empty()) {
    // A. 同じ縦列のペア (2枚抜き) を最優先
    for (const auto & cand : candidates) {
      if (cand.is_pair && std::abs(cand.heading_err - current_target_heading_err) < 0.08) {
        RCLCPP_INFO(
          logger,
          "[VERTICAL LOCK - 2-PANEL PAIR] Same column: %s (Row %d) -> RPM only!",
          cand.desc.c_str(), cand.row);
        return cand;
      }
    }
    // B. 同じ縦列の単独パネル
    for (const auto & cand : candidates) {
      if (!cand.is_pair && std::abs(cand.heading_err - current_target_heading_err) < 0.08) {
        RCLCPP_INFO(
          logger,
          "[VERTICAL LOCK - SINGLE] Same column: %s (Row %d) -> RPM only!",
          cand.desc.c_str(), cand.row);
        return cand;
      }
    }
  }

  // 【最優先ルール 2: 2枚抜き (is_pair = true) の中点を最優先で選択】
  // ペアが存在する限り、単独パネルは狙わず必ず2枚抜きの中点から開始
  double min_pair_turn = 1e9;
  const TargetCandidate * chosen_pair = nullptr;
  for (const auto & cand : candidates) {
    if (cand.is_pair) {
      const double turn_cost = std::abs(cand.heading_err - current_target_heading_err);
      if (turn_cost < min_pair_turn) {
        min_pair_turn = turn_cost;
        chosen_pair = &cand;
      }
    }
  }
  if (chosen_pair) {
    return *chosen_pair;
  }

  // 【最優先ルール 3: ペアが全て倒れた後、残りの単独パネルを最小旋回角で選択】
  double min_turn_angle = 1e9;
  const TargetCandidate * chosen_single = nullptr;
  for (const auto & cand : candidates) {
    const double turn_cost = std::abs(cand.heading_err - current_target_heading_err);
    if (turn_cost < min_turn_angle) {
      min_turn_angle = turn_cost;
      chosen_single = &cand;
    }
  }

  if (chosen_single) {
    return *chosen_single;
  }

  return candidates.front();
}

}  // namespace robot_controller
