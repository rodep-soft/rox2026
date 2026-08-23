#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ament_index_python/packages.hpp>
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
    field_side_ = declare_parameter<std::string>("field_side", "left");
    mirror_y_ = (field_side_ == "right" || field_side_ == "blue") ? -1.0 : 1.0;

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/field/markers", rclcpp::QoS(1).reliable().transient_local());

    // 1 Hz で公式SDF/CAD準拠の立体フィールドマーカーを常時配信
    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&FieldVisualizationNode::publish_field_markers, this));

    RCLCPP_INFO(
      get_logger(),
      "FieldVisualizationNode initialized with Official SDF Field Layout (Side: %s, Frame: %s)",
      field_side_.c_str(), map_frame_.c_str());
  }

private:
  void publish_field_markers()
  {
    visualization_msgs::msg::MarkerArray msg;
    const auto now_stamp = this->now();
    int id = 0;

    // 1. 公式フロアシート (Floor Sheet: 12.85m x 10.9m)
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
      floor.pose.position.z = -0.01;
      floor.pose.orientation.w = 1.0;
      floor.scale.x = 12.85;
      floor.scale.y = 10.90;
      floor.scale.z = 0.01;
      floor.color.r = 0.20f;
      floor.color.g = 0.22f;
      floor.color.b = 0.25f;
      floor.color.a = 0.85f;
      msg.markers.push_back(floor);

      // センターライン
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
      center_line.color.a = 0.9f;
      msg.markers.push_back(center_line);
    }

    // 2. 公式パスエリア (Pass Areas: 0.768m x 1.718m)
    {
      // SIDE A (Left: X = -1.5m)
      visualization_msgs::msg::Marker pass_a;
      pass_a.header.frame_id = map_frame_;
      pass_a.header.stamp = now_stamp;
      pass_a.ns = "pass_areas";
      pass_a.id = id++;
      pass_a.type = visualization_msgs::msg::Marker::CUBE;
      pass_a.action = visualization_msgs::msg::Marker::ADD;
      pass_a.pose.position.x = -1.5;
      pass_a.pose.position.y = 0.0;
      pass_a.pose.position.z = 0.002;
      pass_a.pose.orientation.w = 1.0;
      pass_a.scale.x = 0.768;
      pass_a.scale.y = 1.718;
      pass_a.scale.z = 0.002;
      pass_a.color.r = 0.1f;
      pass_a.color.g = 0.8f;
      pass_a.color.b = 0.3f;
      pass_a.color.a = 0.5f;
      msg.markers.push_back(pass_a);

      // SIDE B (Right: X = +1.5m)
      visualization_msgs::msg::Marker pass_b = pass_a;
      pass_b.id = id++;
      pass_b.pose.position.x = 1.5;
      pass_b.color.r = 0.2f;
      pass_b.color.g = 0.5f;
      pass_b.color.b = 0.9f;
      msg.markers.push_back(pass_b);
    }

    // 3. 公式 Nutmeg Gates (SDF exact: 4基, X: +/-4.6m, Y: +/-4.025m)
    {
      struct GateSpec {
        std::string name;
        double x;
        double y;
        bool is_own;
      };

      const std::vector<GateSpec> gates = {
        {"gate_a_top",    -4.6,  4.025, true},
        {"gate_a_bottom", -4.6, -4.025, true},
        {"gate_b_top",     4.6,  4.025, false},
        {"gate_b_bottom",  4.6, -4.025, false}
      };

      for (const auto & g : gates) {
        // 支柱 1 (+Y 側)
        visualization_msgs::msg::Marker post1;
        post1.header.frame_id = map_frame_;
        post1.header.stamp = now_stamp;
        post1.ns = "nutmeg_gates";
        post1.id = id++;
        post1.type = visualization_msgs::msg::Marker::CUBE;
        post1.action = visualization_msgs::msg::Marker::ADD;
        post1.pose.position.x = g.x;
        post1.pose.position.y = g.y + 0.275;
        post1.pose.position.z = 0.40;
        post1.pose.orientation.w = 1.0;
        post1.scale.x = 0.05;
        post1.scale.y = 0.05;
        post1.scale.z = 0.80;
        post1.color.r = g.is_own ? 0.95f : 0.4f;
        post1.color.g = g.is_own ? 0.55f : 0.4f;
        post1.color.b = g.is_own ? 0.10f : 0.4f;
        post1.color.a = 0.95f;
        msg.markers.push_back(post1);

        // 支柱 2 (-Y 側)
        visualization_msgs::msg::Marker post2 = post1;
        post2.id = id++;
        post2.pose.position.y = g.y - 0.275;
        msg.markers.push_back(post2);

        // 上部パネルバー (Top Panel: 0.05m x 0.55m x 0.2m, Z=0.7m)
        visualization_msgs::msg::Marker top_panel;
        top_panel.header.frame_id = map_frame_;
        top_panel.header.stamp = now_stamp;
        top_panel.ns = "nutmeg_gates";
        top_panel.id = id++;
        top_panel.type = visualization_msgs::msg::Marker::CUBE;
        top_panel.action = visualization_msgs::msg::Marker::ADD;
        top_panel.pose.position.x = g.x;
        top_panel.pose.position.y = g.y;
        top_panel.pose.position.z = 0.70;
        top_panel.pose.orientation.w = 1.0;
        top_panel.scale.x = 0.05;
        top_panel.scale.y = 0.55;
        top_panel.scale.z = 0.20;
        top_panel.color = post1.color;
        msg.markers.push_back(top_panel);
      }
    }

    // 4. 公式ゴール (Goals: Side A @ X=-6.425m, Side B @ X=+6.425m)
    {
      visualization_msgs::msg::Marker goal_a;
      goal_a.header.frame_id = map_frame_;
      goal_a.header.stamp = now_stamp;
      goal_a.ns = "goals";
      goal_a.id = id++;
      goal_a.type = visualization_msgs::msg::Marker::CUBE;
      goal_a.action = visualization_msgs::msg::Marker::ADD;
      goal_a.pose.position.x = -6.425;
      goal_a.pose.position.y = 0.0;
      goal_a.pose.position.z = 0.715;
      goal_a.pose.orientation.w = 1.0;
      goal_a.scale.x = 0.05;
      goal_a.scale.y = 1.60;
      goal_a.scale.z = 1.43;
      goal_a.color.r = 1.0f;
      goal_a.color.g = 0.84f;
      goal_a.color.b = 0.0f;
      goal_a.color.a = 0.8f;
      msg.markers.push_back(goal_a);

      visualization_msgs::msg::Marker goal_b = goal_a;
      goal_b.id = id++;
      goal_b.pose.position.x = 6.425;
      goal_b.pose.position.z = 0.525;
      goal_b.scale.z = 1.05;
      msg.markers.push_back(goal_b);
    }

    // 5. 公式外壁 (Boundary Walls: 0.5m High)
    {
      const std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> walls = {
        {{0.0, 5.45}, {12.85, 0.05}},   // North
        {{0.0, -5.45}, {12.85, 0.05}},  // South
        {{6.425, 0.0}, {0.05, 10.90}},  // East
        {{-6.425, 0.0}, {0.05, 10.90}}  // West
      };

      for (const auto & w : walls) {
        visualization_msgs::msg::Marker wall;
        wall.header.frame_id = map_frame_;
        wall.header.stamp = now_stamp;
        wall.ns = "boundary_walls";
        wall.id = id++;
        wall.type = visualization_msgs::msg::Marker::CUBE;
        wall.action = visualization_msgs::msg::Marker::ADD;
        wall.pose.position.x = w.first.first;
        wall.pose.position.y = w.first.second;
        wall.pose.position.z = 0.25;
        wall.pose.orientation.w = 1.0;
        wall.scale.x = w.second.first;
        wall.scale.y = w.second.second;
        wall.scale.z = 0.50;
        wall.color.r = 0.6f;
        wall.color.g = 0.6f;
        wall.color.b = 0.65f;
        wall.color.a = 0.5f;
        msg.markers.push_back(wall);
      }
    }

    marker_pub_->publish(msg);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string map_frame_{"map"};
  std::string field_side_{"left"};
  double mirror_y_{1.0};
};

}  // namespace robot_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_controller::FieldVisualizationNode)
