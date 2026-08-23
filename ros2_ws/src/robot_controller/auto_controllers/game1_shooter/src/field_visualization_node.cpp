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

    RCLCPP_INFO(get_logger(), "FieldVisualizationNode: Publishing 100%% EXACT SDF Field Dimensions");
  }

private:
  void publish_field_markers()
  {
    visualization_msgs::msg::MarkerArray msg;
    const auto now_stamp = this->now();
    int id = 0;

    // 1. Floor (12.85m x 10.9m) & Center Line
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

    // 2. Pass Areas (SDF: X = +/-1.5, Y = 0.0, Size: 0.768m x 1.718m)
    {
      // SIDE A Pass Area (自陣: X = -1.5m)
      visualization_msgs::msg::Marker pass_a;
      pass_a.header.frame_id = map_frame_;
      pass_a.header.stamp = now_stamp;
      pass_a.ns = "pass_areas";
      pass_a.id = id++;
      pass_a.type = visualization_msgs::msg::Marker::CUBE;
      pass_a.action = visualization_msgs::msg::Marker::ADD;
      pass_a.pose.position.x = -1.50;
      pass_a.pose.position.y = 0.0;
      pass_a.pose.position.z = 0.005;
      pass_a.pose.orientation.w = 1.0;
      pass_a.scale.x = 0.768;
      pass_a.scale.y = 1.718;
      pass_a.scale.z = 0.01;
      pass_a.color.r = 0.10f;
      pass_a.color.g = 0.85f;
      pass_a.color.b = 0.30f;
      pass_a.color.a = 0.70f;
      msg.markers.push_back(pass_a);

      // SIDE B Pass Area (敵陣: X = +1.5m)
      visualization_msgs::msg::Marker pass_b = pass_a;
      pass_b.id = id++;
      pass_b.pose.position.x = 1.50;
      pass_b.color.r = 0.20f;
      pass_b.color.g = 0.50f;
      pass_b.color.b = 0.90f;
      msg.markers.push_back(pass_b);
    }

    // 3. Penalty Areas (SDF: X = +/-5.625, Y = 0.0, Size: 1.6m x 4.95m)
    {
      visualization_msgs::msg::Marker pen_a;
      pen_a.header.frame_id = map_frame_;
      pen_a.header.stamp = now_stamp;
      pen_a.ns = "penalty_areas";
      pen_a.id = id++;
      pen_a.type = visualization_msgs::msg::Marker::CUBE;
      pen_a.action = visualization_msgs::msg::Marker::ADD;
      pen_a.pose.position.x = -5.625;
      pen_a.pose.position.y = 0.0;
      pen_a.pose.position.z = 0.002;
      pen_a.pose.orientation.w = 1.0;
      pen_a.scale.x = 1.60;
      pen_a.scale.y = 4.95;
      pen_a.scale.z = 0.002;
      pen_a.color.r = 1.0f;
      pen_a.color.g = 1.0f;
      pen_a.color.b = 1.0f;
      pen_a.color.a = 0.25f;
      msg.markers.push_back(pen_a);

      visualization_msgs::msg::Marker pen_b = pen_a;
      pen_b.id = id++;
      pen_b.pose.position.x = 5.625;
      msg.markers.push_back(pen_b);
    }

    // 4. Nutmeg Gates (SDF exact: 4 gates, X: +/-4.6, Y: +/-4.025, 柱間隔 0.55m, 高さ 0.8m)
    {
      struct SDFGateSpec {
        std::string name;
        double x;
        double y;
        float r, g, b;
      };

      const std::vector<SDFGateSpec> sdf_gates = {
        {"gate_a_top",    -4.60,  4.025, 0.20f, 0.50f, 0.95f}, // Side A (Blue)
        {"gate_a_bottom", -4.60, -4.025, 0.20f, 0.50f, 0.95f}, // Side A (Blue)
        {"gate_b_top",     4.60,  4.025, 0.95f, 0.30f, 0.20f}, // Side B (Red)
        {"gate_b_bottom",  4.60, -4.025, 0.95f, 0.30f, 0.20f}  // Side B (Red)
      };

      for (const auto & g : sdf_gates) {
        // 支柱 1 (+Y: offset +0.275m)
        visualization_msgs::msg::Marker p1;
        p1.header.frame_id = map_frame_;
        p1.header.stamp = now_stamp;
        p1.ns = "nutmeg_gates";
        p1.id = id++;
        p1.type = visualization_msgs::msg::Marker::CUBE;
        p1.action = visualization_msgs::msg::Marker::ADD;
        p1.pose.position.x = g.x;
        p1.pose.position.y = g.y + 0.275;
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

        // 支柱 2 (-Y: offset -0.275m)
        visualization_msgs::msg::Marker p2 = p1;
        p2.id = id++;
        p2.pose.position.y = g.y - 0.275;
        msg.markers.push_back(p2);

        // 上部パネルバー (pose: 0 0 0.7, size: 0.05 0.55 0.2)
        visualization_msgs::msg::Marker bar;
        bar.header.frame_id = map_frame_;
        bar.header.stamp = now_stamp;
        bar.ns = "nutmeg_gates";
        bar.id = id++;
        bar.type = visualization_msgs::msg::Marker::CUBE;
        bar.action = visualization_msgs::msg::Marker::ADD;
        bar.pose.position.x = g.x;
        bar.pose.position.y = g.y;
        bar.pose.position.z = 0.70;
        bar.pose.orientation.w = 1.0;
        bar.scale.x = 0.05;
        bar.scale.y = 0.55;
        bar.scale.z = 0.20;
        bar.color = p1.color;
        msg.markers.push_back(bar);
      }
    }

    // 5. Goals (SDF exact: X = -6.425m / +6.425m)
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
      goal_a.color.g = 0.85f;
      goal_a.color.b = 0.10f;
      goal_a.color.a = 0.90f;
      msg.markers.push_back(goal_a);

      visualization_msgs::msg::Marker goal_b = goal_a;
      goal_b.id = id++;
      goal_b.pose.position.x = 6.425;
      goal_b.pose.position.z = 0.525;
      goal_b.scale.z = 1.05;
      msg.markers.push_back(goal_b);
    }

    // 6. Boundary Walls (SDF exact: 12.85m x 10.90m, Height: 0.5m)
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
        wall.color.r = 0.70f;
        wall.color.g = 0.70f;
        wall.color.b = 0.75f;
        wall.color.a = 0.40f;
        msg.markers.push_back(wall);
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
