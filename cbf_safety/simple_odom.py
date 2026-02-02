#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import math

class SimpleOdom(Node):
    def __init__(self):
        super().__init__('simple_odom')
        
        self.R = 0.033
        self.L = 0.160

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.prev_l = 0.0
        self.prev_r = 0.0
        self.first_run = True

        self.sub = self.create_subscription(JointState, '/joint_states', self.update_odom, 10)
        self.pub = self.create_publisher(Odometry, '/my_odom', 10)

        self.get_logger().info("SimpleOdom initialized! Waiting for wheel data...")

    def update_odom(self, msg):
        try:
            curr_l = msg.position[0] 
            curr_r = msg.position[1]
        except IndexError:
            return

        if self.first_run:
            self.prev_l = curr_l
            self.prev_r = curr_r
            self.first_run = False
            self.get_logger().info(f"First data received. Initial Wheels: L={curr_l:.2f}, R={curr_r:.2f}")
            return

        d_left_wheel  = (curr_l - self.prev_l) * self.R
        d_right_wheel = (curr_r - self.prev_r) * self.R

        d_center = (d_right_wheel + d_left_wheel) / 2.0
        d_theta  = (d_right_wheel - d_left_wheel) / self.L

        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        self.theta += d_theta

        self.prev_l = curr_l
        self.prev_r = curr_r

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = self.get_clock().now().to_msg()
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        self.pub.publish(odom)

        self.get_logger().info(
            f"Pos: x={self.x:.3f}, y={self.y:.3f} | Theta: {self.theta:.3f} rad",
            throttle_duration_sec=1.0
        )

def main():
    rclpy.init()
    node = SimpleOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
