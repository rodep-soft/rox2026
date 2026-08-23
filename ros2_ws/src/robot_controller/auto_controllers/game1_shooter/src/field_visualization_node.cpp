#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <cmath>
#include <string>

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

    // 1 Hz でフィールドマーカーを常時配信 (Foxglove / RViz2 向け)
    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&FieldVisualizationNode::publish_field_markers, this));

    RCLCPP_INFO(
      get_logger(),
      "FieldVisualizationNode initialized. Publishing /field/markers (Side: %s, Mirror: %.0f)",
      field_side_.c_str(), mirror_y_);
  }

private:
  void publish_field_markers()
  {
    visualization_msgs::msg::MarkerArray msg;
    const auto now_stamp = this->now();

    int id = 0;

    // 1. フィールド外枠・床面 (Field Floor)
    {
      visualization_msgs::msg::Marker floor;
      floor.header.frame_id = map_frame_;
      floor.header.stamp = now_stamp;
      floor.ns = "field_boundary";
      floor.id = id++;
      floor.type = visualization_msgs::msg::Marker::CUBE;
      floor.action = visualization_msgs::msg::Marker::ADD;
      floor.pose.position.x = 2.5;
      floor.pose.position.y = 0.0;
      floor.pose.position.z = -0.01;
      floor.pose.orientation.w = 1.0;
      floor.scale.x = 6.0;
      floor.scale.y = 4.0;
      floor.scale.z = 0.01;
      floor.color.r = 0.15f;
      floor.color.g = 0.18f;
      floor.color.b = 0.22f;
      floor.color.a = 0.8f;
      msg.markers.push_back(floor);
    }

    // 2. スタートエリア枠 (Start Box)
    {
      visualization_msgs::msg::Marker start_box;
      start_box.header.frame_id = map_frame_;
      start_box.header.stamp = now_stamp;
      start_box.ns = "game1_start_area";
      start_box.id = id++;
      start_box.type = visualization_msgs::msg::Marker::CUBE;
      start_box.action = visualization_msgs::msg::Marker::ADD;
      start_box.pose.position.x = 0.0;
      start_box.pose.position.y = 0.0;
      start_box.pose.position.z = 0.005;
      start_box.pose.orientation.w = 1.0;
      start_box.scale.x = 0.8;
      start_box.scale.y = 0.8;
      start_box.scale.z = 0.01;
      start_box.color.r = 0.2f;
      start_box.color.g = 0.7f;
      start_box.color.b = 0.3f;
      start_box.color.a = 0.5f;
      msg.markers.push_back(start_box);
    }

    // 3. ゲート (Gate: 左右支柱 & 上部バー)
    {
      const double gate_x = 1.78;
      const double gate_y = 0.0 * mirror_y_;

      // 支柱 A (Left post)
      visualization_msgs::msg::Marker post_l;
      post_l.header.frame_id = map_frame_;
      post_l.header.stamp = now_stamp;
      post_l.ns = "gate_structure";
      post_l.id = id++;
      post_l.type = visualization_msgs::msg::Marker::CYLINDER;
      post_l.action = visualization_msgs::msg::Marker::ADD;
      post_l.pose.position.x = gate_x;
      post_l.pose.position.y = gate_y + 0.35;
      post_l.pose.position.z = 0.25;
      post_l.pose.orientation.w = 1.0;
      post_l.scale.x = 0.04;
      post_l.scale.y = 0.04;
      post_l.scale.z = 0.50;
      post_l.color.r = 0.9f;
      post_l.color.g = 0.6f;
      post_l.color.b = 0.1f;
      post_l.color.a = 1.0f;
      msg.markers.push_back(post_l);

      // 支柱 B (Right post)
      visualization_msgs::msg::Marker post_r = post_l;
      post_r.id = id++;
      post_r.pose.position.y = gate_y - 0.35;
      msg.markers.push_back(post_r);

      // 上部バー (Top Bar)
      visualization_msgs::msg::Marker bar;
      bar.header.frame_id = map_frame_;
      bar.header.stamp = now_stamp;
      bar.ns = "gate_structure";
      bar.id = id++;
      bar.type = visualization_msgs::msg::Marker::CUBE;
      bar.action = visualization_msgs::msg::Marker::ADD;
      bar.pose.position.x = gate_x;
      bar.pose.position.y = gate_y;
      bar.pose.position.z = 0.50;
      bar.pose.orientation.w = 1.0;
      bar.scale.x = 0.06;
      bar.scale.y = 0.74;
      bar.scale.z = 0.04;
      bar.color.r = 0.9f;
      bar.color.g = 0.6f;
      bar.color.b = 0.1f;
      bar.color.a = 1.0f;
      msg.markers.push_back(bar);
    }

    // 4. パスエリア枠 (Pass Area Box: 手書き作戦図の枠)
    {
      visualization_msgs::msg::Marker pass_box;
      pass_box.header.frame_id = map_frame_;
      pass_box.header.stamp = now_stamp;
      pass_box.ns = "game1_pass_area";
      pass_box.id = id++;
      pass_box.type = visualization_msgs::msg::Marker::CUBE;
      pass_box.action = visualization_msgs::msg::Marker::ADD;
      pass_box.pose.position.x = 3.5;
      pass_box.pose.position.y = 0.8 * mirror_y_;
      pass_box.pose.position.z = 0.005;
      pass_box.pose.orientation.w = 1.0;
      pass_box.scale.x = 1.5;
      pass_box.scale.y = 1.0;
      pass_box.scale.z = 0.01;
      pass_box.color.r = 0.2f;
      pass_box.color.g = 0.4f;
      pass_box.color.b = 0.9f;
      pass_box.color.a = 0.4f;
      msg.markers.push_back(pass_box);
    }

    // 5. Game 2 AprilTag 3x3 ターゲットパネル群 (Target Board)
    {
      const double target_wall_x = -4.0; // 後方4m位置
      const double col_pitch = 0.40;
      const double row_pitch = 0.43;
      const double base_z = 0.50;

      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          visualization_msgs::msg::Marker panel;
          panel.header.frame_id = map_frame_;
          panel.header.stamp = now_stamp;
          panel.ns = "game2_target_panels";
          panel.id = id++;
          panel.type = visualization_msgs::msg::Marker::CUBE;
          panel.action = visualization_msgs::msg::Marker::ADD;
          panel.pose.position.x = target_wall_x;
          panel.pose.position.y = (c - 1) * col_pitch;
          panel.pose.position.z = base_z + r * row_pitch;
          panel.pose.orientation.w = 1.0;
          panel.scale.x = 0.02;
          panel.scale.y = 0.25;
          panel.scale.z = 0.25;
          panel.color.r = 0.9f;
          panel.color.g = 0.2f;
          panel.color.b = 0.2f;
          panel.color.a = 0.9f;
          msg.markers.push_back(panel);
        }
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
