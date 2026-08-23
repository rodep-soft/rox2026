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
    field_side_ = declare_parameter<std::string>("field_side", "left");
    mirror_y_ = (field_side_ == "right" || field_side_ == "blue") ? -1.0 : 1.0;

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/field/markers", rclcpp::QoS(1).reliable().transient_local());

    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&FieldVisualizationNode::publish_field_markers, this));

    RCLCPP_INFO(get_logger(), "FieldVisualizationNode: Publishing Clean 3D Solid Markers");
  }

private:
  void publish_field_markers()
  {
    visualization_msgs::msg::MarkerArray msg;
    const auto now_stamp = this->now();
    int id = 0;

    // 1. 全体フィールドフロア (Floor: 6.0m x 4.0m)
    {
      visualization_msgs::msg::Marker floor;
      floor.header.frame_id = map_frame_;
      floor.header.stamp = now_stamp;
      floor.ns = "field_floor";
      floor.id = id++;
      floor.type = visualization_msgs::msg::Marker::CUBE;
      floor.action = visualization_msgs::msg::Marker::ADD;
      floor.pose.position.x = 2.5;
      floor.pose.position.y = 1.0 * mirror_y_;
      floor.pose.position.z = -0.01;
      floor.pose.orientation.w = 1.0;
      floor.scale.x = 6.0;
      floor.scale.y = 4.5;
      floor.scale.z = 0.01;
      floor.color.r = 0.22f;
      floor.color.g = 0.24f;
      floor.color.b = 0.28f;
      floor.color.a = 0.90f;
      msg.markers.push_back(floor);

      // センターライン
      visualization_msgs::msg::Marker center_line = floor;
      center_line.ns = "field_lines";
      center_line.id = id++;
      center_line.pose.position.y = -1.0 * mirror_y_;
      center_line.pose.position.z = 0.001;
      center_line.scale.x = 6.0;
      center_line.scale.y = 0.05;
      center_line.scale.z = 0.002;
      center_line.color.r = 1.0f;
      center_line.color.g = 1.0f;
      center_line.color.b = 1.0f;
      center_line.color.a = 0.95f;
      msg.markers.push_back(center_line);
    }

    // 2. スタート枠 (0,0) - 800mm x 800mm (緑色)
    {
      visualization_msgs::msg::Marker start_box;
      start_box.header.frame_id = map_frame_;
      start_box.header.stamp = now_stamp;
      start_box.ns = "game1_start_box";
      start_box.id = id++;
      start_box.type = visualization_msgs::msg::Marker::CUBE;
      start_box.action = visualization_msgs::msg::Marker::ADD;
      start_box.pose.position.x = 0.0;
      start_box.pose.position.y = 0.0;
      start_box.pose.position.z = 0.005;
      start_box.pose.orientation.w = 1.0;
      start_box.scale.x = 0.80;
      start_box.scale.y = 0.80;
      start_box.scale.z = 0.01;
      start_box.color.r = 0.15f;
      start_box.color.g = 0.85f;
      start_box.color.b = 0.25f;
      start_box.color.a = 0.65f;
      msg.markers.push_back(start_box);
    }

    // 3. パスエリア枠 (青色・外枠とエリア床)
    {
      visualization_msgs::msg::Marker pass_box;
      pass_box.header.frame_id = map_frame_;
      pass_box.header.stamp = now_stamp;
      pass_box.ns = "game1_pass_box";
      pass_box.id = id++;
      pass_box.type = visualization_msgs::msg::Marker::CUBE;
      pass_box.action = visualization_msgs::msg::Marker::ADD;
      pass_box.pose.position.x = 3.50;
      pass_box.pose.position.y = 1.50 * mirror_y_;
      pass_box.pose.position.z = 0.005;
      pass_box.pose.orientation.w = 1.0;
      pass_box.scale.x = 1.718;
      pass_box.scale.y = 0.768;
      pass_box.scale.z = 0.01;
      pass_box.color.r = 0.15f;
      pass_box.color.g = 0.45f;
      pass_box.color.b = 0.95f;
      pass_box.color.a = 0.65f;
      msg.markers.push_back(pass_box);
    }

    // 4. ディフェンダーパネル（ゲート 2体）
    // 第1ゲート: 前方 1.78m
    // 第2ゲート: 前方 2.80m, 右側 0.6m
    {
      struct GatePos {
        double x;
        double y;
        bool is_active;
      };

      const std::vector<GatePos> gates = {
        {1.78, 0.0 * mirror_y_, true},
        {2.80, 0.6 * mirror_y_, false}
      };

      for (const auto & g : gates) {
        // 支柱 1 (+Y)
        visualization_msgs::msg::Marker post1;
        post1.header.frame_id = map_frame_;
        post1.header.stamp = now_stamp;
        post1.ns = "defender_panels";
        post1.id = id++;
        post1.type = visualization_msgs::msg::Marker::CYLINDER;
        post1.action = visualization_msgs::msg::Marker::ADD;
        post1.pose.position.x = g.x;
        post1.pose.position.y = g.y + 0.35;
        post1.pose.position.z = 0.25;
        post1.pose.orientation.w = 1.0;
        post1.scale.x = 0.05;
        post1.scale.y = 0.05;
        post1.scale.z = 0.50;
        post1.color.r = g.is_active ? 0.95f : 0.45f;
        post1.color.g = g.is_active ? 0.55f : 0.45f;
        post1.color.b = g.is_active ? 0.10f : 0.45f;
        post1.color.a = 1.0f;
        msg.markers.push_back(post1);

        // 支柱 2 (-Y)
        visualization_msgs::msg::Marker post2 = post1;
        post2.id = id++;
        post2.pose.position.y = g.y - 0.35;
        msg.markers.push_back(post2);

        // 上部バー / パネル頭部
        visualization_msgs::msg::Marker bar;
        bar.header.frame_id = map_frame_;
        bar.header.stamp = now_stamp;
        bar.ns = "defender_panels";
        bar.id = id++;
        bar.type = visualization_msgs::msg::Marker::CUBE;
        bar.action = visualization_msgs::msg::Marker::ADD;
        bar.pose.position.x = g.x;
        bar.pose.position.y = g.y;
        bar.pose.position.z = 0.50;
        bar.pose.orientation.w = 1.0;
        bar.scale.x = 0.08;
        bar.scale.y = 0.76;
        bar.scale.z = 0.12;
        bar.color = post1.color;
        msg.markers.push_back(bar);
      }
    }

    // 5. Game 2 ターゲットパネル（9枚の赤い的マス）
    {
      const double g2_center_x = 4.5;
      const double g2_y = -0.5 * mirror_y_;
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          visualization_msgs::msg::Marker panel;
          panel.header.frame_id = map_frame_;
          panel.header.stamp = now_stamp;
          panel.ns = "game2_panels";
          panel.id = id++;
          panel.type = visualization_msgs::msg::Marker::CUBE;
          panel.action = visualization_msgs::msg::Marker::ADD;
          panel.pose.position.x = g2_center_x;
          panel.pose.position.y = g2_y + (c - 1) * 0.32;
          panel.pose.position.z = 0.20 + r * 0.32;
          panel.pose.orientation.w = 1.0;
          panel.scale.x = 0.04;
          panel.scale.y = 0.28;
          panel.scale.z = 0.28;
          panel.color.r = 0.85f;
          panel.color.g = 0.20f;
          panel.color.b = 0.20f;
          panel.color.a = 0.90f;
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_controller::FieldVisualizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
