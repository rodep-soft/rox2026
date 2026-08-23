#pragma once

#include "game2_shooter/panel_types.hpp"
#include <optional>
#include <vector>

namespace robot_controller
{

struct TargetSelectionConfig
{
  double shooter_offset_x{-0.265};
  double shooter_offset_y{0.0};
  double yaw_tolerance{0.015};
  bool enable_double_panel_midpoint_targeting{true};
  bool prefer_same_row_first{false};
  bool enable_vertical_sweep{true};
  bool enable_nearest_angle_search{true};
};

class TargetSelector
{
public:
  TargetSelector() = default;

  // 全段の残存検出パネルから最適なターゲット候補を抽出・選択
  std::optional<TargetCandidate> select_best_target(
    const std::unordered_map<int, PanelTagInfo> & panel_grid,
    int current_active_row,
    double current_target_heading_err,
    const std::vector<int> & current_target_tag_ids,
    const TargetSelectionConfig & config,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock);

private:
  std::vector<TargetCandidate> build_candidates(
    const std::unordered_map<int, PanelTagInfo> & panel_grid,
    const TargetSelectionConfig & config);

  static double compute_heading_error(
    double target_x, double target_y,
    double shooter_x, double shooter_y);
};

}  // namespace robot_controller
