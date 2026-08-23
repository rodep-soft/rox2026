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

    RCLCPP_INFO(get_logger(), "FieldVisualizationNode: Publishing field layout and elevated HUD AprilTags to /field/markers");
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
      start_a.pose.position.x = -3.20;
      start_a.pose.position.y = -0.50;
      start_a.scale.x = 1.20;
      start_a.scale.y = 1.20;
      start_a.color.r = 0.00f;
      start_a.color.g = 0.85f;
      start_a.color.b = 0.60f;
      start_a.color.a = 0.65f;
      msg.markers.push_back(start_a);

      visualization_msgs::msg::Marker start_b = start_a;
      start_b.id = id++;
      start_b.pose.position.x = 3.20;
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
      pass_a.pose.position.x = -1.30;
      pass_a.pose.position.y = 1.50;
      pass_a.pose.position.z = 0.005;
      pass_a.pose.orientation.w = 1.0;
      pass_a.scale.x = 0.80;
      pass_a.scale.y = 2.00;
      pass_a.scale.z = 0.01;
      pass_a.color.r = 0.05f;
      pass_a.color.g = 0.90f;
      pass_a.color.b = 0.55f;
      pass_a.color.a = 0.80f;
      msg.markers.push_back(pass_a);

      visualization_msgs::msg::Marker pass_b = pass_a;
      pass_b.id = id++;
      pass_b.pose.position.x = 1.30;
      pass_b.color.r = 0.05f;
      pass_b.color.g = 0.65f;
      pass_b.color.b = 1.00f;
      msg.markers.push_back(pass_b);
    }

    // 5. GAME1 DFゲート
    {
      struct DFGateLayout {
        std::string name;
        double x;
        double y;
        bool is_horizontal;
        float r, g, b;
      };

      const std::vector<DFGateLayout> gates = {
        {"df_a_top",  -3.20, 3.80, true,  0.10f, 0.75f, 1.00f},
        {"df_a_mid",  -4.50, 1.50, false, 0.10f, 0.75f, 1.00f},
        {"df_b_top",   3.20, 3.80, true,  1.00f, 0.55f, 0.10f},
        {"df_b_mid",   4.50, 1.50, false, 1.00f, 0.55f, 0.10f}
      };

      for (const auto & g : gates) {
        const double half_gap = 0.30;
        double p1_x = g.x, p1_y = g.y;
        double p2_x = g.x, p2_y = g.y;
        double bar_sx = 0.05, bar_sy = 0.65;

        if (g.is_horizontal) {
          p1_x += half_gap;
          p2_x -= half_gap;
          bar_sx = 0.65;
          bar_sy = 0.05;
        } else {
          p1_y += half_gap;
          p2_y -= half_gap;
          bar_sx = 0.05;
          bar_sy = 0.65;
        }

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

        visualization_msgs::msg::Marker p2 = p1;
        p2.id = id++;
        p2.pose.position.x = p2_x;
        p2.pose.position.y = p2_y;
        msg.markers.push_back(p2);

        visualization_msgs::msg::Marker top_bar = p1;
        top_bar.id = id++;
        top_bar.pose.position.x = g.x;
        top_bar.pose.position.y = g.y;
        top_bar.pose.position.z = 0.80;
        top_bar.scale.x = bar_sx;
        top_bar.scale.y = bar_sy;
        top_bar.scale.z = 0.06;
        msg.markers.push_back(top_bar);
      }
    }

    // 6. 全27枚の公式 AprilTag 3D マーカー (法線オフセット + 優先オーバーレイ表示)
    {
      struct TagData {
        int id;
        double x, y, z, normal_yaw;
      };
      const std::vector<TagData> all_tags = {
        // コーナー・外壁
        {0,  -6.495, -5.020, 0.420,  0.000},   // 左壁
        {1,  -6.020, -5.495, 0.420,  1.571},   // 下壁
        {2,  -6.500,  4.920, 0.320,  0.000},   // 左壁
        {3,  -6.020,  5.495, 0.420, -1.571},   // 上壁

        // GAME1 縦向きDFゲート
        {4,  -4.505,  2.165, 0.922,  0.000},   // バー表
        {5,  -4.445,  2.165, 0.922,  3.142},   // バー裏
        {6,  -4.475,  2.495, 0.122, -1.571},   // 上支柱
        {7,  -4.475,  1.835, 0.122,  1.571},   // 下支柱

        // GAME1 上側横向きDFゲート
        {8,  -3.165,  3.905, 0.922, -1.571},   // バー表
        {9,  -3.495,  3.875, 0.122,  0.000},   // 左支柱
        {10, -2.835,  3.875, 0.122,  3.142},   // 右支柱
        {11, -3.165,  3.845, 0.922,  1.571},   // バー裏

        // センターラインポール
        {12,  0.015,  0.750, 0.422,  3.142},
        {13,  0.015,  2.550, 0.422,  3.142},

        // GAME2 3x3 シュートパネル (上段 14,15,16 / 中段 17,18,19 / 下段 20,21,22)
        {14, -3.640, -5.525, 1.190,  1.571},
        {15, -3.230, -5.525, 1.190,  1.571},
        {16, -2.820, -5.525, 1.190,  1.571},
        {17, -3.640, -5.525, 0.730,  1.571},
        {18, -3.230, -5.525, 0.730,  1.571},
        {19, -2.820, -5.525, 0.730,  1.571},
        {20, -3.640, -5.525, 0.270,  1.571},
        {21, -3.230, -5.525, 0.270,  1.571},
        {22, -2.820, -5.525, 0.270,  1.571},

        // GAME3 ゴールエリアポスト
        {23,  0.850, -5.505, 0.120,  1.571},
        {24,  0.850, -5.505, 0.970,  1.571},
        {25, -0.850, -5.505, 0.120,  1.571},
        {26, -0.850, -5.505, 0.970,  1.571}
      };

      for (const auto & tag : all_tags) {
        // 法線ベクトル方向に 0.06m 前面へオフセット（メッシュの奥に埋もれるのを完全に防止）
        const double forward_offset = 0.06;
        const double nx = std::cos(tag.normal_yaw);
        const double ny = std::sin(tag.normal_yaw);
        const double px = tag.x + forward_offset * nx;
        const double py = tag.y + forward_offset * ny;

        // 1. Tag プレート本体 (前面オフセット)
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
        tag_plate.scale.x = 0.015;
        tag_plate.scale.y = 0.18;
        tag_plate.scale.z = 0.18;
        tag_plate.color.r = 0.05f;
        tag_plate.color.g = 0.05f;
        tag_plate.color.b = 0.07f;
        tag_plate.color.a = 1.0f;
        msg.markers.push_back(tag_plate);

        // 2. HUD バックプレートバッジ (Tag プレートのさらに 0.03m 前面 & 上部)
        visualization_msgs::msg::Marker badge;
        badge.header.frame_id = map_frame_;
        badge.header.stamp = now_stamp;
        badge.ns = "hud_badges";
        badge.id = id++;
        badge.type = visualization_msgs::msg::Marker::CUBE;
        badge.action = visualization_msgs::msg::Marker::ADD;
        badge.pose.position.x = px + 0.03 * nx;
        badge.pose.position.y = py + 0.03 * ny;
        badge.pose.position.z = tag.z + 0.18; // メッシュ上端から完全に浮上
        badge.pose.orientation = tag_plate.pose.orientation;
        badge.scale.x = 0.012;
        badge.scale.y = 0.26;
        badge.scale.z = 0.09;
        badge.color.r = 0.02f;
        badge.color.g = 0.10f;
        badge.color.b = 0.20f;
        badge.color.a = 0.92f;
        msg.markers.push_back(badge);

        // 3. 最前面 HUD テキストラベル (最優先でくっきり描画)
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = map_frame_;
        text_marker.header.stamp = now_stamp;
        text_marker.ns = "apriltag_hud_text";
        text_marker.id = id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = px + 0.05 * nx;
        text_marker.pose.position.y = py + 0.05 * ny;
        text_marker.pose.position.z = tag.z + 0.18;
        text_marker.scale.z = 0.14; // くっきり大型フォント
        text_marker.color.r = 0.00f;
        text_marker.color.g = 1.00f;
        text_marker.color.b = 0.85f; // ネオンエメラルドシアン
        text_marker.color.a = 1.00f;

        char buf[32];
        std::snprintf(buf, sizeof(buf), "[ %02d ]", tag.id);
        text_marker.text = buf;
        msg.markers.push_back(text_marker);
      }
    }

    // 7. GAME3 ゴールエリア & GAME2 パネル (3x3)
    {
      visualization_msgs::msg::Marker g3_goal;
      g3_goal.header.frame_id = map_frame_;
      g3_goal.header.stamp = now_stamp;
      g3_goal.ns = "game3_goal";
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
      g3_goal.color.r = 1.00f;
      g3_goal.color.g = 0.85f;
      g3_goal.color.b = 0.15f;
      g3_goal.color.a = 0.90f;
      msg.markers.push_back(g3_goal);

      const std::vector<double> g2_xs = {-3.23, 3.23};
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
            panel.pose.position.x = g2_x + (c - 1) * 0.41;
            panel.pose.position.y = -5.45;
            panel.pose.position.z = 0.27 + r * 0.46;
            panel.pose.orientation.w = 1.0;
            panel.scale.x = 0.35;
            panel.scale.y = 0.04;
            panel.scale.z = 0.35;
            panel.color.r = 0.90f;
            panel.color.g = 0.20f;
            panel.color.b = 0.25f;
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
