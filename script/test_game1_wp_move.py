#!/usr/bin/env python3
import time
import math
import rclcpp
from rclcpp.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class TestGame1WpMoveNode(Node):
    def __init__(self):
        super().__init__('test_game1_wp_move')

        # パラメータ宣言
        self.declare_parameter('target_x', 1.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('kp_linear', 2.0)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('max_linear_vel', 2.0)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('pos_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance', 0.05)
        self.declare_parameter('timeout_sec', 15.0)

        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.target_yaw = self.get_parameter('target_yaw').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.pos_tolerance = self.get_parameter('pos_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value
        self.timeout_sec = self.get_parameter('timeout_sec').value

        # パブリッシャー & サブスクライバー
        self.cmd_vel_pub = self.create_publisher(Twist, '/drive/cmd_vel', 10)
        self.completed_pub = self.create_publisher(Bool, '/game1/wp_test/completed', 10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        self.start_time = self.get_clock().now()

        # 20Hz 制御ループ
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(
            f"TestGame1WpMove initialized. Target Relative WP: ({self.target_x:.2f}, {self.target_y:.2f}, {self.target_yaw:.2f} rad)"
        )

    def odom_callback(self, msg: Odometry):
        # Quaternion -> Yaw 変換
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        raw_yaw = math.atan2(siny_cosp, cosy_cosp)

        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = raw_x
            self.start_y = raw_y
            self.start_yaw = raw_yaw
            self.start_time = self.get_clock().now()
            self.get_logger().info(f"Start pose zero-reset at X:{self.start_x:.3f}, Y:{self.start_y:.3f}, Yaw:{self.start_yaw:.3f}")

        # スタート時からの相対座標
        dx_raw = raw_x - self.start_x
        dy_raw = raw_y - self.start_y
        
        # スタート時の向きに合わせて回転変換
        cos_yaw = math.cos(-self.start_yaw)
        sin_yaw = math.sin(-self.start_yaw)
        self.current_x = dx_raw * cos_yaw - dy_raw * sin_yaw
        self.current_y = dx_raw * sin_yaw + dy_raw * cos_yaw
        self.current_yaw = math.atan2(math.sin(raw_yaw - self.start_yaw), math.cos(raw_yaw - self.start_yaw))

        self.odom_received = True

    def control_loop(self):
        if not self.odom_received:
            self.get_logger().info('Waiting for /odometry/filtered...', throttle_duration_sec=2.0)
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        # EKF 自己位置からの位置誤差・角度誤差
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dist_err = math.hypot(dx, dy)

        yaw_err = math.atan2(math.sin(self.target_yaw - self.current_yaw),
                             math.cos(self.target_yaw - self.current_yaw))

        # 到達またはタイムアウトの判定
        if (dist_err <= self.pos_tolerance and abs(yaw_err) <= self.yaw_tolerance) or elapsed > self.timeout_sec:
            # 停止
            self.cmd_vel_pub.publish(Twist())
            
            comp_msg = Bool()
            comp_msg.data = True
            self.completed_pub.publish(comp_msg)

            if elapsed > self.timeout_sec:
                self.get_logger().warn(
                    f"Target WP TIMED OUT after {elapsed:.1f}s. Final Pos: ({self.current_x:.3f}, {self.current_y:.3f}, {self.current_yaw:.3f})"
                )
            else:
                self.get_logger().info(
                    f"Reached Target WP successfully! Pos: ({self.current_x:.3f}, {self.current_y:.3f}, {self.current_yaw:.3f})"
                )
            
            self.timer.cancel()
            return

        # ホロノミック全方位制御指令値計算（Field-Oriented -> Body-Centric）
        cmd = Twist()
        if dist_err > 1e-4:
            target_speed = min(self.max_linear_vel, self.kp_linear * dist_err)
            vx_world = target_speed * (dx / dist_err)
            vy_world = target_speed * (dy / dist_err)

            # 現在のYaw角を使って車体座標系へ回転変換
            cos_curr = math.cos(self.current_yaw)
            sin_curr = math.sin(self.current_yaw)
            cmd.linear.x = cos_curr * vx_world + sin_curr * vy_world
            cmd.linear.y = -sin_curr * vx_world + cos_curr * vy_world

        cmd.angular.z = max(min(self.kp_angular * yaw_err, self.max_angular_vel), -self.max_angular_vel)

        self.cmd_vel_pub.publish(cmd)

        self.get_logger().info(
            f"[Moving] Current: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_yaw:.2f}) -> "
            f"Err: dist={dist_err:.3f}m, yaw={yaw_err:.3f}rad",
            throttle_duration_sec=1.0
        )

def main(args=None):
    rclcpp.init(args=args)
    node = TestGame1WpMoveNode()
    try:
        rclcpp.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclcpp.shutdown()

if __name__ == '__main__':
    main()
