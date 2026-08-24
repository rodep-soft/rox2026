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
      // 3枚残存時: 左側2枚(0&1)の中点狙い
      if (p0 && p1 && p2) {
        TargetCandidate c;
        c.row = row;
        c.tag_ids = {p0->tag_id, p1->tag_id};
        c.x = (p0->x + p1->x) * 0.5;
        c.y = (p0->y + p1->y) * 0.5;
        c.z = (p0->z + p1->z) * 0.5;
        c.heading_err = compute_heading_error(c.x, c.y, config.shooter_offset_x, config.shooter_offset_y);
        c.is_pair = true;
        c.desc = "Row " + std::to_string(row) + " Midpoint #" + std::to_string(p0->tag_id) +
          " & #" + std::to_string(p1->tag_id);
        candidates.push_back(c);
      } else if (p0 && p1) {
        TargetCandidate c;
        c.row = row;
        c.tag_ids = {p0->tag_id, p1->tag_id};
        c.x = (p0->x + p1->x) * 0.5;
        c.y = (p0->y + p1->y) * 0.5;
        c.z = (p0->z + p1->z) * 0.5;
        c.heading_err = compute_heading_error(c.x, c.y, config.shooter_offset_x, config.shooter_offset_y);
        c.is_pair = true;
        c.desc = "Row " + std::to_string(row) + " Midpoint #" + std::to_string(p0->tag_id) +
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
        c.desc = "Row " + std::to_string(row) + " Midpoint #" + std::to_string(p1->tag_id) +
          " & #" + std::to_string(p2->tag_id);
        candidates.push_back(c);
      }
    }

    // 単独残存候補 (ペアが作れなかった、または単独のパネル)
    if (candidates.empty() || candidates.back().row != row) {
      const PanelTagInfo * single = p1 ? p1 : (p0 ? p0 : p2);
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

  // 【最優先ルール: 垂直スイープ / 旋回0度ロック】
  // 一度射出した方向と同じ縦列 (heading_err 差が 0.08 rad 以内) に未撃破パネルがあれば、
  // 他の段へ絶対に旋回せず、その列の残存パネルを RPM 変更のみで全て撃ち尽くす (旋回回数最小化)
  if (!current_target_tag_ids.empty()) {
    for (const auto & cand : candidates) {
      if (std::abs(cand.heading_err - current_target_heading_err) < 0.08) {
        RCLCPP_INFO(
          logger,
          "[VERTICAL LOCK - ZERO TURN] Continuing same column: %s (Row %d) -> Changing RPM only!",
          cand.desc.c_str(), cand.row);
        return cand;
      }
    }
  }

  // 【第二優先: 最も近い方位（最小旋回角）の未撃破ターゲットを選択】
  // 現在向いている方位から最も少ない旋回角で狙える縦列・ターゲットへ1回だけ旋回
  double min_turn_angle = 1e9;
  const TargetCandidate * chosen = nullptr;

  for (const auto & cand : candidates) {
    const double turn_cost = std::abs(cand.heading_err - current_target_heading_err);
    if (turn_cost < min_turn_angle) {
      min_turn_angle = turn_cost;
      chosen = &cand;
    }
  }

  if (!chosen) {
    chosen = &candidates.front();
  }

  return *chosen;
}

}  // namespace robot_controller
