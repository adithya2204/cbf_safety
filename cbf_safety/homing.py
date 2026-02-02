#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class HomingNode(Node):
    def __init__(self):
        super().__init__('homing_node')

        # --- HOME POSITION ---
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_theta = 0.0

        # --- CONTROL GAINS ---
        self.k_lin = 0.3
        self.k_ang = 1.2

        # --- TOLERANCES ---
        self.dist_tol = 0.05      # meters
        self.angle_tol = 0.05     # radians

        # --- STATE ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # --- ROS COMM ---
        self.sub = self.create_subscription(
            Odometry,
            '/my_odom',
            self.odom_cb,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info("Homing node activated 🚀")

    def odom_cb(self, msg):
        # --- EXTRACT POSE ---
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.theta = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

        # --- ERROR COMPUTATION ---
        dx = self.home_x - self.x
        dy = self.home_y - self.y
        dist_error = math.hypot(dx, dy)

        target_theta = math.atan2(dy, dx)
        ang_error = self.normalize_angle(target_theta - self.theta)

        cmd = Twist()

        # --- CONTROL LOGIC ---
        if dist_error > self.dist_tol:
            # Rotate first
            if abs(ang_error) > self.angle_tol:
                cmd.angular.z = self.k_ang * ang_error
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = self.k_lin * dist_error
                cmd.angular.z = 0.0
        else:
            # Final orientation correction
            final_ang_error = self.normalize_angle(self.home_theta - self.theta)
            if abs(final_ang_error) > self.angle_tol:
                cmd.angular.z = self.k_ang * final_ang_error
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info("🏠 Home reached!")
                self.pub.publish(cmd)
                return

        self.pub.publish(cmd)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main():
    rclpy.init()
    node = HomingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
