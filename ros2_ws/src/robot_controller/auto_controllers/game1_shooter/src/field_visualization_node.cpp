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

    RCLCPP_INFO(get_logger(), "FieldVisualizationNode: Publishing EXACT Onshape CAD Field Positions");
  }

private:
  void publish_field_markers()
  {
    visualization_msgs::msg::MarkerArray msg;
    const auto now_stamp = this->now();
    int id = 0;

    // 1. 全体フロア (Floor Sheet: 12.85m x 10.9m) & センターライン
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
      floor.color.g = 0.25f;
      floor.color.b = 0.28f;
      floor.color.a = 0.90f;
      msg.markers.push_back(floor);

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

    // 2. スタート範囲 (Onshape CAD: X = +/-3.43m, Y = 0.75m)
    {
      // Side A (自陣: X = -3.43m)
      visualization_msgs::msg::Marker start_a;
      start_a.header.frame_id = map_frame_;
      start_a.header.stamp = now_stamp;
      start_a.ns = "start_areas";
      start_a.id = id++;
      start_a.type = visualization_msgs::msg::Marker::CUBE;
      start_a.action = visualization_msgs::msg::Marker::ADD;
      start_a.pose.position.x = -3.43;
      start_a.pose.position.y = 0.75;
      start_a.pose.position.z = 0.005;
      start_a.pose.orientation.w = 1.0;
      start_a.scale.x = 0.80;
      start_a.scale.y = 0.80;
      start_a.scale.z = 0.01;
      start_a.color.r = 0.10f;
      start_a.color.g = 0.85f;
      start_a.color.b = 0.25f;
      start_a.color.a = 0.70f;
      msg.markers.push_back(start_a);

      // Side B (敵陣: X = +3.43m)
      visualization_msgs::msg::Marker start_b = start_a;
      start_b.id = id++;
      start_b.pose.position.x = 3.43;
      start_b.color.r = 0.20f;
      start_b.color.g = 0.50f;
      start_b.color.b = 0.90f;
      msg.markers.push_back(start_b);
    }

    // 3. パスエリア (Onshape CAD: X = +/-1.316m, Y = 1.641m, Size: 1.718m x 0.768m)
    {
      visualization_msgs::msg::Marker pass_a;
      pass_a.header.frame_id = map_frame_;
      pass_a.header.stamp = now_stamp;
      pass_a.ns = "pass_areas";
      pass_a.id = id++;
      pass_a.type = visualization_msgs::msg::Marker::CUBE;
      pass_a.action = visualization_msgs::msg::Marker::ADD;
      pass_a.pose.position.x = -1.316;
      pass_a.pose.position.y = 1.641;
      pass_a.pose.position.z = 0.005;
      pass_a.pose.orientation.w = 1.0;
      pass_a.scale.x = 0.768;
      pass_a.scale.y = 1.718;
      pass_a.scale.z = 0.01;
      pass_a.color.r = 0.15f;
      pass_a.color.g = 0.85f;
      pass_a.color.b = 0.30f;
      pass_a.color.a = 0.70f;
      msg.markers.push_back(pass_a);

      visualization_msgs::msg::Marker pass_b = pass_a;
      pass_b.id = id++;
      pass_b.pose.position.x = 1.316;
      pass_b.color.r = 0.20f;
      pass_b.color.g = 0.50f;
      pass_b.color.b = 0.90f;
      msg.markers.push_back(pass_b);
    }

    // 4. ディフェンダーパネル（4基のゲート: Onshape CAD exact poses）
    {
      struct GateCADSpec {
        std::string name;
        double x;
        double y;
        bool is_vertical; // vertical: along Y-axis, horizontal: along X-axis
        float r, g, b;
      };

      const std::vector<GateCADSpec> gates = {
        // 自陣 1: X = -4.475, Y = 1.89 (横向き)
        {"gate_a_1", -4.475, 1.89, false, 0.20f, 0.50f, 0.95f},
        // 自陣 2: X = -3.440, Y = 3.875 (縦向き)
        {"gate_a_2", -3.440, 3.875, true,  0.20f, 0.50f, 0.95f},
        // 敵陣 1: X = +4.475, Y = 1.89 (横向き)
        {"gate_b_1",  4.475, 1.89, false, 0.95f, 0.30f, 0.20f},
        // 敵陣 2: X = +2.890, Y = 3.875 (縦向き)
        {"gate_b_2",  2.890, 3.875, true,  0.95f, 0.30f, 0.20f}
      };

      for (const auto & g : gates) {
        const double offset = 0.275;
        double p1_x = g.x, p1_y = g.y;
        double p2_x = g.x, p2_y = g.y;
        double bar_sx = 0.05, bar_sy = 0.55;

        if (g.is_vertical) {
          p1_y += offset;
          p2_y -= offset;
          bar_sx = 0.05;
          bar_sy = 0.55;
        } else {
          p1_x += offset;
          p2_x -= offset;
          bar_sx = 0.55;
          bar_sy = 0.05;
        }

        // 支柱 1
        visualization_msgs::msg::Marker p1;
        p1.header.frame_id = map_frame_;
        p1.header.stamp = now_stamp;
        p1.ns = "defender_gates";
        p1.id = id++;
        p1.type = visualization_msgs::msg::Marker::CUBE;
        p1.action = visualization_msgs::msg::Marker::ADD;
        p1.pose.position.x = p1_x;
        p1.pose.position.y = p1_y;
        p1.pose.position.z = 0.40;
        p1.pose.orientation.w = 1.0;
        p1.scale.x = 0.05;
        p1.scale.y = 0.05;
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

        // 上部バー
        visualization_msgs::msg::Marker bar;
        bar.header.frame_id = map_frame_;
        bar.header.stamp = now_stamp;
        bar.ns = "defender_gates";
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

    // 5. Game 2 ターゲットパネル (Onshape CAD: Y = 5.525m)
    {
      const std::vector<double> g2_centers = {-3.23, 3.22};
      for (const double g2_x : g2_centers) {
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
            panel.pose.position.y = 5.525;
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
