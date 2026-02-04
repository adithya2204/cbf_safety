#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import threading

class HomingNode(Node):
    def __init__(self):
        super().__init__('homing_node')
        
        # --- Control Parameters ---
        self.kp_linear = 0.4
        self.kp_angular = 1.8
        self.distance_tolerance = 0.05
        self.angle_tolerance = math.radians(5.0)
        self.max_linear_speed = 0.19
        self.max_angular_speed = 1.0

        # --- State Variables ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.homing_active = False

        # --- ROS Setup ---
        self.odom_sub = self.create_subscription(Odometry, '/my_odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Thread for non-blocking UI
        self.input_thread = threading.Thread(target=self.wait_for_user, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info("Homing Node Initialized.")
        self.get_logger().info("Drive the robot. When ready to test dead reckoning, press ENTER here.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Convert Quaternion to Yaw
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_theta = 2.0 * math.atan2(qz, qw)

        if self.homing_active:
            self.execute_homing()

    def wait_for_user(self):
        while rclpy.ok():
            input("\n[READY] Press ENTER to return to (0,0)...\n")
            if not self.homing_active:
                self.homing_active = True
                self.get_logger().info("Homing Started")
            else:
                self.get_logger().warn("Already homing")

    def execute_homing(self):
        cmd = Twist()
        
        # 1. Calculate vector to origin
        dist = math.sqrt(self.current_x**2 + self.current_y**2)
        target_angle = math.atan2(-self.current_y, -self.current_x)
        
        angle_error = target_angle - self.current_theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if dist < self.distance_tolerance:
            # ARRIVED
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f"Home Reached. Error: {dist:.4f}m")
            self.homing_active = False
            return

        if abs(angle_error) > self.angle_tolerance:
            cmd.linear.x = 0.0
            ang_vel = self.kp_angular * angle_error
            cmd.angular.z = max(min(ang_vel, self.max_angular_speed), -self.max_angular_speed)
        else:
            lin_vel = self.kp_linear * dist
            cmd.linear.x = max(min(lin_vel, self.max_linear_speed), 0.03)
            cmd.angular.z = self.kp_angular * angle_error
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = HomingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()