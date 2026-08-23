#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include <string>
#include <cmath>

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
      std::chrono::milliseconds(1000),
      std::bind(&FieldVisualizationNode::publish_field_markers, this));

    RCLCPP_INFO(get_logger(), "FieldVisualizationNode: Publishing verified field markers and AprilTags");
  }

private:
  void publish_field_markers()
  {
    visualization_msgs::msg::MarkerArray msg;
    const auto now_stamp = this->now();
    int32_t id = 0;

    // 1. フィールド床面
    {
      visualization_msgs::msg::Marker floor;
      floor.header.frame_id = map_frame_;
      floor.header.stamp = now_stamp;
      floor.ns = "field_floor";
      floor.id = id++;
      floor.type = visualization_msgs::msg::Marker::CUBE;
      floor.action = visualization_msgs::msg::Marker::ADD;
      floor.pose.position.x = 0.0;
      floor.pose.position.y = 0.0;
      floor.pose.position.z = -0.01;
      floor.pose.orientation.w = 1.0;
      floor.scale.x = 12.85;
      floor.scale.y = 10.90;
      floor.scale.z = 0.02;
      floor.color.r = 0.10f;
      floor.color.g = 0.12f;
      floor.color.b = 0.16f;
      floor.color.a = 0.98f;
      msg.markers.push_back(floor);

      visualization_msgs::msg::Marker cline;
      cline.header.frame_id = map_frame_;
      cline.header.stamp = now_stamp;
      cline.ns = "center_line";
      cline.id = id++;
      cline.type = visualization_msgs::msg::Marker::CUBE;
      cline.action = visualization_msgs::msg::Marker::ADD;
      cline.pose.position.x = 0.0;
      cline.pose.position.y = 0.0;
      cline.pose.position.z = 0.002;
      cline.pose.orientation.w = 1.0;
      cline.scale.x = 0.08;
      cline.scale.y = 10.90;
      cline.scale.z = 0.005;
      cline.color.r = 0.95f;
      cline.color.g = 0.98f;
      cline.color.b = 1.00f;
      cline.color.a = 0.95f;
      msg.markers.push_back(cline);
    }

    // 2. コーナーエリア (4隅)
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
        corner.color.r = 0.40f;
        corner.color.g = 0.50f;
        corner.color.b = 0.60f;
        corner.color.a = 0.25f;
        msg.markers.push_back(corner);
      }
    }

    // 3. スタートエリア
    {
      visualization_msgs::msg::Marker g1_start_a;
      g1_start_a.header.frame_id = map_frame_;
      g1_start_a.header.stamp = now_stamp;
      g1_start_a.ns = "start_areas";
      g1_start_a.id = id++;
      g1_start_a.type = visualization_msgs::msg::Marker::CUBE;
      g1_start_a.action = visualization_msgs::msg::Marker::ADD;
      g1_start_a.pose.position.x = -5.925;
      g1_start_a.pose.position.y = 4.950;
      g1_start_a.pose.position.z = 0.005;
      g1_start_a.pose.orientation.w = 1.0;
      g1_start_a.scale.x = 1.00;
      g1_start_a.scale.y = 1.00;
      g1_start_a.scale.z = 0.01;
      g1_start_a.color.r = 0.00f;
      g1_start_a.color.g = 0.95f;
      g1_start_a.color.b = 0.45f;
      g1_start_a.color.a = 0.70f;
      msg.markers.push_back(g1_start_a);

      visualization_msgs::msg::Marker g1_start_b = g1_start_a;
      g1_start_b.id = id++;
      g1_start_b.pose.position.x = 5.925;
      g1_start_b.color.r = 0.00f;
      g1_start_b.color.g = 0.70f;
      g1_start_b.color.b = 1.00f;
      msg.markers.push_back(g1_start_b);

      visualization_msgs::msg::Marker start_a = g1_start_a;
      start_a.id = id++;
      start_a.pose.position.x = -3.43;
      start_a.pose.position.y = -0.75;
      start_a.scale.x = 1.20;
      start_a.scale.y = 1.20;
      start_a.color.r = 0.00f;
      start_a.color.g = 0.85f;
      start_a.color.b = 0.60f;
      start_a.color.a = 0.65f;
      msg.markers.push_back(start_a);

      visualization_msgs::msg::Marker start_b = start_a;
      start_b.id = id++;
      start_b.pose.position.x = 3.43;
      start_b.color.r = 0.20f;
      start_b.color.g = 0.50f;
      start_b.color.b = 1.00f;
      msg.markers.push_back(start_b);
    }

    // 4. GAME1 パスエリア
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
      pass_a.color.r = 0.05f;
      pass_a.color.g = 0.90f;
      pass_a.color.b = 0.55f;
      pass_a.color.a = 0.80f;
      msg.markers.push_back(pass_a);

      visualization_msgs::msg::Marker pass_b = pass_a;
      pass_b.id = id++;
      pass_b.pose.position.x = 1.316;
      pass_b.color.r = 0.05f;
      pass_b.color.g = 0.65f;
      pass_b.color.b = 1.00f;
      msg.markers.push_back(pass_b);
    }

    // 5. GAME1 DFゲート構造体 (SDF CAD メッシュの frame pose に完全一致)
    {
      struct DFGateDef {
        double p1_x, p1_y, p2_x, p2_y, bar_x, bar_y;
        double bar_sx, bar_sy;
        float r, g, b;
      };

      const std::vector<DFGateDef> gates = {
        // 自陣 縦向きゲート (X = -4.475)
        {-4.375, 1.830, -4.375, 2.490, -4.475, 2.160, 0.06, 0.66, 0.10f, 0.75f, 1.00f},
        // 自陣 横向きゲート (Y = 3.880)
        {-3.500, 3.775, -2.840, 3.775, -3.170, 3.880, 0.66, 0.06, 0.10f, 0.75f, 1.00f},
        // 敵陣 縦向きゲート (X = 4.475)
        { 4.575, 1.830,  4.575, 2.490,  4.475, 2.160, 0.06, 0.66, 1.00f, 0.55f, 0.10f},
        // 敵陣 横向きゲート (Y = 3.880)
        { 2.830, 3.775,  3.490, 3.775,  3.160, 3.880, 0.66, 0.06, 1.00f, 0.55f, 0.10f}
      };

      for (const auto & g : gates) {
        visualization_msgs::msg::Marker p1;
        p1.header.frame_id = map_frame_;
        p1.header.stamp = now_stamp;
        p1.ns = "df_gates";
        p1.id = id++;
        p1.type = visualization_msgs::msg::Marker::CUBE;
        p1.action = visualization_msgs::msg::Marker::ADD;
        p1.pose.position.x = g.p1_x;
        p1.pose.position.y = g.p1_y;
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

        visualization_msgs::msg::Marker p2 = p1;
        p2.id = id++;
        p2.pose.position.x = g.p2_x;
        p2.pose.position.y = g.p2_y;
        msg.markers.push_back(p2);

        visualization_msgs::msg::Marker top_bar = p1;
        top_bar.id = id++;
        top_bar.pose.position.x = g.bar_x;
        top_bar.pose.position.y = g.bar_y;
        top_bar.pose.position.z = 0.80;
        top_bar.scale.x = g.bar_sx;
        top_bar.scale.y = g.bar_sy;
        top_bar.scale.z = 0.06;
        msg.markers.push_back(top_bar);
      }
    }

    // 6. GAME3 ゴール構造体
    {
      visualization_msgs::msg::Marker g3_top_bar;
      g3_top_bar.header.frame_id = map_frame_;
      g3_top_bar.header.stamp = now_stamp;
      g3_top_bar.ns = "game3_goal_structure";
      g3_top_bar.id = id++;
      g3_top_bar.type = visualization_msgs::msg::Marker::CUBE;
      g3_top_bar.action = visualization_msgs::msg::Marker::ADD;
      g3_top_bar.pose.position.x = 0.10;
      g3_top_bar.pose.position.y = -5.510;
      g3_top_bar.pose.position.z = 0.850;
      g3_top_bar.pose.orientation.w = 1.0;
      g3_top_bar.scale.x = 1.70;
      g3_top_bar.scale.y = 0.06;
      g3_top_bar.scale.z = 0.06;
      g3_top_bar.color.r = 1.00f;
      g3_top_bar.color.g = 0.85f;
      g3_top_bar.color.b = 0.10f;
      g3_top_bar.color.a = 1.00f;
      msg.markers.push_back(g3_top_bar);

      visualization_msgs::msg::Marker g3_left_post = g3_top_bar;
      g3_left_post.id = id++;
      g3_left_post.pose.position.x = -0.750;
      g3_left_post.pose.position.z = 0.425;
      g3_left_post.scale.x = 0.06;
      g3_left_post.scale.z = 0.85;
      msg.markers.push_back(g3_left_post);

      visualization_msgs::msg::Marker g3_right_post = g3_top_bar;
      g3_right_post.id = id++;
      g3_right_post.pose.position.x = 0.950;
      g3_right_post.pose.position.z = 0.425;
      g3_right_post.scale.x = 0.06;
      g3_right_post.scale.z = 0.85;
      msg.markers.push_back(g3_right_post);

      visualization_msgs::msg::Marker g3_left_rail = g3_top_bar;
      g3_left_rail.id = id++;
      g3_left_rail.pose.position.x = -0.750;
      g3_left_rail.pose.position.y = -5.985;
      g3_left_rail.pose.position.z = 0.050;
      g3_left_rail.scale.x = 0.05;
      g3_left_rail.scale.y = 0.95;
      g3_left_rail.scale.z = 0.08;
      g3_left_rail.color.r = 0.85f;
      g3_left_rail.color.g = 0.85f;
      g3_left_rail.color.b = 0.90f;
      msg.markers.push_back(g3_left_rail);

      visualization_msgs::msg::Marker g3_right_rail = g3_left_rail;
      g3_right_rail.id = id++;
      g3_right_rail.pose.position.x = 0.950;
      msg.markers.push_back(g3_right_rail);
    }

    // 7. GAME2 3x3 シュートパネル構造体 (Z: 0.27m, 0.73m, 1.19m)
    {
      const std::vector<double> g2_xs = {-3.23, 3.63};
      for (const double g2_x : g2_xs) {
        for (int r = 0; r < 3; ++r) {
          for (int c = 0; c < 3; ++c) {
            visualization_msgs::msg::Marker panel;
            panel.header.frame_id = map_frame_;
            panel.header.stamp = now_stamp;
            panel.ns = "game2_shoot_panels";
            panel.id = id++;
            panel.type = visualization_msgs::msg::Marker::CUBE;
            panel.action = visualization_msgs::msg::Marker::ADD;
            panel.pose.position.x = g2_x + (c - 1) * 0.41;
            panel.pose.position.y = -5.525;
            panel.pose.position.z = 0.27 + r * 0.46;
            panel.pose.orientation.w = 1.0;
            panel.scale.x = 0.36;
            panel.scale.y = 0.03;
            panel.scale.z = 0.36;
            panel.color.r = 0.85f;
            panel.color.g = 0.20f;
            panel.color.b = 0.25f;
            panel.color.a = 0.92f;
            msg.markers.push_back(panel);
          }
        }
      }
    }

    // 8. 全27枚の公式 AprilTag (前repo / 正しいCAD joint pose 完全一致版)
    {
      struct TagData {
        int id;
        double x, y, z, normal_yaw;
      };
      const std::vector<TagData> all_tags = {
        // 四隅・外壁
        // 自陣 (SIDE A):
        {0,  -5.920,  5.490, 0.300,  1.5708},   // 上壁
        {1,  -6.500,  4.920, 0.300,  3.1416},   // 左上壁
        {2,  -6.490, -4.920, 0.300,  3.1416},   // 左下壁
        {3,  -5.920, -5.500, 0.300, -1.5708},   // 下壁
        // 敵陣 (SIDE B):
        {0,   5.920,  5.490, 0.300,  1.5708},
        {1,   6.500,  4.920, 0.300,  0.0000},
        {2,   6.490, -4.920, 0.300,  0.0000},
        {3,   5.920, -5.500, 0.300, -1.5708},

        // 横向きゲート (SIDE A)
        {4,  -3.265,  3.910, 0.802,  1.5708},   // バー表 (北向き)
        {6,  -3.265,  3.850, 0.802, -1.5708},   // バー裏 (南向き)
        {7,  -3.500,  3.775, 0.400,  3.1416},   // 左支柱 (西向き)
        {5,  -2.840,  3.775, 0.400,  0.0000},   // 右支柱 (東向き)

        // 横向きゲート (SIDE B)
        {4,   3.065,  3.910, 0.802,  1.5708},
        {6,   3.065,  3.850, 0.802, -1.5708},
        {5,   2.830,  3.775, 0.400,  3.1416},
        {7,   3.490,  3.775, 0.400,  0.0000},

        // 縦向きゲート (SIDE A)
        {9,  -4.375,  2.490, 0.400,  1.5708},   // 上支柱 (北向き)
        {11, -4.375,  1.830, 0.400, -1.5708},   // 下支柱 (南向き)
        {8,  -4.510,  2.065, 0.802,  3.1416},   // 左バー (西向き)
        {10, -4.450,  2.065, 0.802,  0.0000},   // 右バー (東向き)

        // 縦向きゲート (SIDE B)
        {9,   4.575,  2.490, 0.400,  1.5708},
        {11,  4.575,  1.830, 0.400, -1.5708},
        {10,  4.440,  2.065, 0.802,  3.1416},
        {8,   4.500,  2.065, 0.802,  0.0000},

        // センターラインポール
        {12, -0.020,  2.450, 0.400,  3.1416},
        {13, -0.020,  0.650, 0.400,  3.1416},

        // GAME2 3x3 シュートパネル (Z: 0.27m / 0.73m / 1.19m)
        {16, -3.640, -5.525, 1.190,  1.5708},
        {15, -3.230, -5.525, 1.190,  1.5708},
        {14, -2.820, -5.525, 1.190,  1.5708},
        {19, -3.640, -5.525, 0.730,  1.5708},
        {18, -3.230, -5.525, 0.730,  1.5708},
        {17, -2.820, -5.525, 0.730,  1.5708},
        {22, -3.640, -5.525, 0.270,  1.5708},
        {21, -3.230, -5.525, 0.270,  1.5708},
        {20, -2.820, -5.525, 0.270,  1.5708},

        // GAME3 ゴールエリア
        {24, -0.750, -5.510, 0.850,  1.5708},
        {23,  0.950, -5.510, 0.850,  1.5708},
        {26, -0.750, -5.510, 0.120,  1.5708},
        {25,  0.950, -5.510, 0.120,  1.5708}
      };

      for (const auto & tag : all_tags) {
        const double forward_offset = 0.025;
        const double nx = std::cos(tag.normal_yaw);
        const double ny = std::sin(tag.normal_yaw);
        const double px = tag.x + forward_offset * nx;
        const double py = tag.y + forward_offset * ny;

        // 1. Tag プレート本体 (黒外枠)
        visualization_msgs::msg::Marker tag_plate;
        tag_plate.header.frame_id = map_frame_;
        tag_plate.header.stamp = now_stamp;
        tag_plate.ns = "apriltags_plate";
        tag_plate.id = id++;
        tag_plate.type = visualization_msgs::msg::Marker::CUBE;
        tag_plate.action = visualization_msgs::msg::Marker::ADD;
        tag_plate.pose.position.x = px;
        tag_plate.pose.position.y = py;
        tag_plate.pose.position.z = tag.z;
        tag_plate.pose.orientation.z = std::sin(tag.normal_yaw / 2.0);
        tag_plate.pose.orientation.w = std::cos(tag.normal_yaw / 2.0);
        tag_plate.scale.x = 0.012;
        tag_plate.scale.y = 0.18;
        tag_plate.scale.z = 0.18;
        tag_plate.color.r = 0.05f;
        tag_plate.color.g = 0.05f;
        tag_plate.color.b = 0.07f;
        tag_plate.color.a = 0.95f;
        msg.markers.push_back(tag_plate);

        // 2. 白背景インナー枠
        visualization_msgs::msg::Marker tag_inner;
        tag_inner.header.frame_id = map_frame_;
        tag_inner.header.stamp = now_stamp;
        tag_inner.ns = "apriltags_inner";
        tag_inner.id = id++;
        tag_inner.type = visualization_msgs::msg::Marker::CUBE;
        tag_inner.action = visualization_msgs::msg::Marker::ADD;
        tag_inner.pose.position.x = px + 0.008 * nx;
        tag_inner.pose.position.y = py + 0.008 * ny;
        tag_inner.pose.position.z = tag.z;
        tag_inner.pose.orientation = tag_plate.pose.orientation;
        tag_inner.scale.x = 0.008;
        tag_inner.scale.y = 0.14;
        tag_inner.scale.z = 0.14;
        tag_inner.color.r = 0.95f;
        tag_inner.color.g = 0.95f;
        tag_inner.color.b = 0.98f;
        tag_inner.color.a = 0.95f;
        msg.markers.push_back(tag_inner);

        // 3. 頭上に浮かぶネオンシアン HUD バッジ (#00 〜 #26)
        visualization_msgs::msg::Marker hud_badge;
        hud_badge.header.frame_id = map_frame_;
        hud_badge.header.stamp = now_stamp;
        hud_badge.ns = "apriltags_floating_hud";
        hud_badge.id = id++;
        hud_badge.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        hud_badge.action = visualization_msgs::msg::Marker::ADD;
        hud_badge.pose.position.x = tag.x + 0.15 * nx;
        hud_badge.pose.position.y = tag.y + 0.15 * ny;
        hud_badge.pose.position.z = tag.z + 0.18;
        hud_badge.scale.z = 0.13;
        hud_badge.color.r = 0.00f;
        hud_badge.color.g = 1.00f;
        hud_badge.color.b = 0.85f;
        hud_badge.color.a = 1.00f;
        char hud_buf[32];
        std::snprintf(hud_buf, sizeof(hud_buf), "#%02d", tag.id);
        hud_badge.text = hud_buf;
        msg.markers.push_back(hud_badge);
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
