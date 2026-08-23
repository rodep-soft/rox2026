#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <string>
#include <vector>

namespace robot_controller
{

class FieldVisualizationNode : public rclcpp::Node
{
public:
  explicit FieldVisualizationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("field_visualization_node", options)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/field/markers", rclcpp::QoS(1).reliable().transient_local());

    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&FieldVisualizationNode::publish_field_markers, this));

    RCLCPP_INFO(get_logger(), "FieldVisualizationNode: Publishing EXACT Official Rulebook 2D Field Layout");
  }

private:
  void publish_field_markers()
  {
    visualization_msgs::msg::MarkerArray msg;
    const auto now_stamp = this->now();
    int id = 0;

    // 1. 全体フィールドフロア (Floor: 12.85m x 10.9m) & 白線
    {
      visualization_msgs::msg::Marker floor;
      floor.header.frame_id = map_frame_;
      floor.header.stamp = now_stamp;
      floor.ns = "field_base";
      floor.id = id++;
      floor.type = visualization_msgs::msg::Marker::CUBE;
      floor.action = visualization_msgs::msg::Marker::ADD;
      floor.pose.position.x = 0.0;
      floor.pose.position.y = 0.0;
      floor.pose.position.z = -0.005;
      floor.pose.orientation.w = 1.0;
      floor.scale.x = 12.85;
      floor.scale.y = 10.90;
      floor.scale.z = 0.01;
      floor.color.r = 0.22f;
      floor.color.g = 0.24f;
      floor.color.b = 0.28f;
      floor.color.a = 0.90f;
      msg.markers.push_back(floor);

      // センターライン (縦にフィールドを二分: X=0)
      visualization_msgs::msg::Marker center_line = floor;
      center_line.ns = "field_lines";
      center_line.id = id++;
      center_line.pose.position.z = 0.001;
      center_line.scale.x = 0.05;
      center_line.scale.y = 10.90;
      center_line.scale.z = 0.002;
      center_line.color.r = 1.0f;
      center_line.color.g = 1.0f;
      center_line.color.b = 1.0f;
      center_line.color.a = 0.95f;
      msg.markers.push_back(center_line);
    }

    // 2. コーナーエリア (4隅の正方形)
    {
      const std::vector<std::pair<double, double>> corners = {
        {-6.425 + 0.5,  5.45 - 0.5},
        { 6.425 - 0.5,  5.45 - 0.5},
        {-6.425 + 0.5, -5.45 + 0.5},
        { 6.425 - 0.5, -5.45 + 0.5}
      };
      for (const auto & c : corners) {
        visualization_msgs::msg::Marker corner;
        corner.header.frame_id = map_frame_;
        corner.header.stamp = now_stamp;
        corner.ns = "corner_areas";
        corner.id = id++;
        corner.type = visualization_msgs::msg::Marker::CUBE;
        corner.action = visualization_msgs::msg::Marker::ADD;
        corner.pose.position.x = c.first;
        corner.pose.position.y = c.second;
        corner.pose.position.z = 0.001;
        corner.pose.orientation.w = 1.0;
        corner.scale.x = 1.0;
        corner.scale.y = 1.0;
        corner.scale.z = 0.002;
        corner.color.r = 1.0f;
        corner.color.g = 1.0f;
        corner.color.b = 1.0f;
        corner.color.a = 0.20f;
        msg.markers.push_back(corner);
      }
    }

    // 3. スタートエリア (GAME1 上側スタート / GAME2 手前側スタート)
    {
      // GAME1 上側スタートエリア (自陣: X = -5.50m, Y = 4.50m) - 鮮やかな緑色
      visualization_msgs::msg::Marker g1_start_a;
      g1_start_a.header.frame_id = map_frame_;
      g1_start_a.header.stamp = now_stamp;
      g1_start_a.ns = "start_areas";
      g1_start_a.id = id++;
      g1_start_a.type = visualization_msgs::msg::Marker::CUBE;
      g1_start_a.action = visualization_msgs::msg::Marker::ADD;
      g1_start_a.pose.position.x = -5.50;
      g1_start_a.pose.position.y = 4.50;
      g1_start_a.pose.position.z = 0.005;
      g1_start_a.pose.orientation.w = 1.0;
      g1_start_a.scale.x = 1.00;
      g1_start_a.scale.y = 1.00;
      g1_start_a.scale.z = 0.01;
      g1_start_a.color.r = 0.10f;
      g1_start_a.color.g = 0.90f;
      g1_start_a.color.b = 0.20f;
      g1_start_a.color.a = 0.75f;
      msg.markers.push_back(g1_start_a);

      // GAME1 敵陣上側スタート
      visualization_msgs::msg::Marker g1_start_b = g1_start_a;
      g1_start_b.id = id++;
      g1_start_b.pose.position.x = 5.50;
      g1_start_b.color.r = 0.20f;
      g1_start_b.color.g = 0.50f;
      g1_start_b.color.b = 0.90f;
      msg.markers.push_back(g1_start_b);

      // GAME2 手前側スタートエリア (自陣: X = -3.20m, Y = -0.50m)
      visualization_msgs::msg::Marker start_a = g1_start_a;
      start_a.id = id++;
      start_a.pose.position.x = -3.20;
      start_a.pose.position.y = -0.50;
      start_a.scale.x = 1.20;
      start_a.scale.y = 1.20;
      start_a.color.a = 0.40f;
      msg.markers.push_back(start_a);

      // GAME2 敵陣手前側スタート
      visualization_msgs::msg::Marker start_b = start_a;
      start_b.id = id++;
      start_b.pose.position.x = 3.20;
      start_b.color.r = 0.20f;
      start_b.color.g = 0.50f;
      start_b.color.b = 0.90f;
      msg.markers.push_back(start_b);
    }

    // 4. GAME1 パスエリア (センターライン寄りの縦長ボックス)
    {
      // SIDE A パスエリア (X = -1.3m, Y = 1.5m)
      visualization_msgs::msg::Marker pass_a;
      pass_a.header.frame_id = map_frame_;
      pass_a.header.stamp = now_stamp;
      pass_a.ns = "pass_areas";
      pass_a.id = id++;
      pass_a.type = visualization_msgs::msg::Marker::CUBE;
      pass_a.action = visualization_msgs::msg::Marker::ADD;
      pass_a.pose.position.x = -1.30;
      pass_a.pose.position.y = 1.50;
      pass_a.pose.position.z = 0.005;
      pass_a.pose.orientation.w = 1.0;
      pass_a.scale.x = 0.80;
      pass_a.scale.y = 1.80;
      pass_a.scale.z = 0.01;
      pass_a.color.r = 0.15f;
      pass_a.color.g = 0.85f;
      pass_a.color.b = 0.30f;
      pass_a.color.a = 0.70f;
      msg.markers.push_back(pass_a);

      // SIDE B パスエリア (X = +1.3m, Y = 1.5m)
      visualization_msgs::msg::Marker pass_b = pass_a;
      pass_b.id = id++;
      pass_b.pose.position.x = 1.30;
      pass_b.color.r = 0.20f;
      pass_b.color.g = 0.50f;
      pass_b.color.b = 0.90f;
      msg.markers.push_back(pass_b);
    }

    // 5. GAME1 DFパネル（ゲート 4体）
    // 【上側ゲート】: 自陣・敵陣ともに 横向き (X軸方向にバーが伸びる: |--|)
    // 【中段外側ゲート】: 自陣・敵陣ともに 縦向き (Y軸方向にバーが伸びる: I)
    {
      struct DFGateLayout {
        std::string name;
        double x;
        double y;
        bool is_horizontal; // true: |--| (横向き), false: I (縦向き)
        float r, g, b;
      };

      const std::vector<DFGateLayout> gates = {
        // SIDE A (自陣: 水色/青)
        {"df_a_top",  -3.20, 3.80, true,  0.20f, 0.60f, 0.95f}, // 上部横向き
        {"df_a_mid",  -4.50, 1.50, false, 0.20f, 0.60f, 0.95f}, // パスエリア左横・縦向き

        // SIDE B (敵陣: 橙/赤)
        {"df_b_top",   3.20, 3.80, true,  0.95f, 0.55f, 0.15f}, // 上部横向き
        {"df_b_mid",   4.50, 1.50, false, 0.95f, 0.55f, 0.15f}  // パスエリア右横・縦向き
      };

      for (const auto & g : gates) {
        const double half_gap = 0.30;
        double p1_x = g.x, p1_y = g.y;
        double p2_x = g.x, p2_y = g.y;
        double bar_sx = 0.05, bar_sy = 0.65;

        if (g.is_horizontal) {
          // 横向き |--| : 支柱は左右 (X方向) に並ぶ
          p1_x += half_gap;
          p2_x -= half_gap;
          bar_sx = 0.65;
          bar_sy = 0.05;
        } else {
          // 縦向き I : 支柱は上下 (Y方向) に並ぶ
          p1_y += half_gap;
          p2_y -= half_gap;
          bar_sx = 0.05;
          bar_sy = 0.65;
        }

        // 支柱 1
        visualization_msgs::msg::Marker p1;
        p1.header.frame_id = map_frame_;
        p1.header.stamp = now_stamp;
        p1.ns = "df_gates";
        p1.id = id++;
        p1.type = visualization_msgs::msg::Marker::CUBE;
        p1.action = visualization_msgs::msg::Marker::ADD;
        p1.pose.position.x = p1_x;
        p1.pose.position.y = p1_y;
        p1.pose.position.z = 0.40;
        p1.pose.orientation.w = 1.0;
        p1.scale.x = 0.06;
        p1.scale.y = 0.06;
        p1.scale.z = 0.80;
        p1.color.r = g.r;
        p1.color.g = g.g;
        p1.color.b = g.b;
        p1.color.a = 1.0f;
        msg.markers.push_back(p1);

        // 支柱 2
        visualization_msgs::msg::Marker p2 = p1;
        p2.id = id++;
        p2.pose.position.x = p2_x;
        p2.pose.position.y = p2_y;
        msg.markers.push_back(p2);

        // 上部バー / パネル
        visualization_msgs::msg::Marker bar;
        bar.header.frame_id = map_frame_;
        bar.header.stamp = now_stamp;
        bar.ns = "df_gates";
        bar.id = id++;
        bar.type = visualization_msgs::msg::Marker::CUBE;
        bar.action = visualization_msgs::msg::Marker::ADD;
        bar.pose.position.x = g.x;
        bar.pose.position.y = g.y;
        bar.pose.position.z = 0.70;
        bar.pose.orientation.w = 1.0;
        bar.scale.x = bar_sx;
        bar.scale.y = bar_sy;
        bar.scale.z = 0.20;
        bar.color = p1.color;
        msg.markers.push_back(bar);
      }
    }

    // 6. 手前側の GAME2 シュートパネル & GAME3 ゴールエリア (下辺: Y = -5.45m)
    {
      // GAME3 ゴールエリア (中央下)
      visualization_msgs::msg::Marker g3_goal;
      g3_goal.header.frame_id = map_frame_;
      g3_goal.header.stamp = now_stamp;
      g3_goal.ns = "goals";
      g3_goal.id = id++;
      g3_goal.type = visualization_msgs::msg::Marker::CUBE;
      g3_goal.action = visualization_msgs::msg::Marker::ADD;
      g3_goal.pose.position.x = 0.0;
      g3_goal.pose.position.y = -5.45;
      g3_goal.pose.position.z = 0.25;
      g3_goal.pose.orientation.w = 1.0;
      g3_goal.scale.x = 1.60;
      g3_goal.scale.y = 0.40;
      g3_goal.scale.z = 0.50;
      g3_goal.color.r = 1.0f;
      g3_goal.color.g = 0.85f;
      g3_goal.color.b = 0.10f;
      g3_goal.color.a = 0.90f;
      msg.markers.push_back(g3_goal);

      // GAME2 シュートパネル (自陣下: X = -3.2m, 敵陣下: X = +3.2m)
      const std::vector<double> g2_xs = {-3.20, 3.20};
      for (const double g2_x : g2_xs) {
        for (int r = 0; r < 3; ++r) {
          for (int c = 0; c < 3; ++c) {
            visualization_msgs::msg::Marker panel;
            panel.header.frame_id = map_frame_;
            panel.header.stamp = now_stamp;
            panel.ns = "game2_panels";
            panel.id = id++;
            panel.type = visualization_msgs::msg::Marker::CUBE;
            panel.action = visualization_msgs::msg::Marker::ADD;
            panel.pose.position.x = g2_x + (c - 1) * 0.32;
            panel.pose.position.y = -5.45;
            panel.pose.position.z = 0.20 + r * 0.32;
            panel.pose.orientation.w = 1.0;
            panel.scale.x = 0.28;
            panel.scale.y = 0.04;
            panel.scale.z = 0.28;
            panel.color.r = 0.85f;
            panel.color.g = 0.20f;
            panel.color.b = 0.20f;
            panel.color.a = 0.90f;
            msg.markers.push_back(panel);
          }
        }
      }
    }

    marker_pub_->publish(msg);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string map_frame_{"map"};
};

}  // namespace robot_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_controller::FieldVisualizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
