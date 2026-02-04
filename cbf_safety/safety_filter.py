#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data 
from rclpy.executors import MultiThreadedExecutor
import csv
import time

class CBFSafetyNode(Node):
    def __init__(self):
        super().__init__('cbf_safety_filter')

        # --- Parameters ---
        self.declare_parameter('wall_pos', 2.0)
        self.declare_parameter('gamma', 0.50)
        
        self.wall_pos = self.get_parameter('wall_pos').value
        self.gamma = self.get_parameter('gamma').value
        
        # --- Variables (Shared Resources) ---
        self.robot_x = 0.0
        self.latest_u = 0.0
        self.odom_received = False
        self.latest_angular_z = 0.0
        self.actual_vel_odom = 0.0  # To compare Command vs Reality

        # --- CSV Logging Setup ---
        self.csv_filename = 'cbf_safety_log.csv'
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Header: Time | Position | Input (Unsafe) | Output (Safe) | Actual (Odom)
        self.csv_writer.writerow(['timestamp', 'pos_x', 'dist_to_wall', 'input_u', 'safe_u', 'actual_vel_odom'])
        self.start_time = time.time()
        
        self.get_logger().info(f"Logging data to {self.csv_filename}...")

        # --- Communication ---
        self.sub_odom = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            qos_profile=qos_profile_sensor_data 
        )

        # Command Subscriber (The "Input")
        self.sub_cmd = self.create_subscription(
            Twist, 
            '/cmd_vel_raw', 
            self.cmd_callback, 
            10
        )
        
        # Safe Command Publisher (The "Output")
        self.pub_safe = self.create_publisher(Twist, '/cmd_vel', 10)

        # 3. THE CONTROL LOOP
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(f"CBF Decoupled Loop Active. Wall : {self.wall_pos}m")

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        # We capture the REAL velocity from Odom to compare against our Command
        self.actual_vel_odom = msg.twist.twist.linear.x 
        self.odom_received = True

    def cmd_callback(self, msg: Twist):
        self.latest_u = msg.linear.x
        self.latest_angular_z = msg.angular.z

    def control_loop(self):
        if not self.odom_received:
            return

        # --- CBF Math ---
        dist = self.wall_pos - self.robot_x
        u_limit = self.gamma * dist
        
        # Check Safety
        if self.latest_u > u_limit:
            u_safe = u_limit
            status = "BRAKING"
        else:
            u_safe = self.latest_u
            status = "SAFE"

        # Publish Safe Command
        safe_msg = Twist()
        safe_msg.linear.x = float(u_safe)
        safe_msg.angular.z = self.latest_angular_z
        self.pub_safe.publish(safe_msg)

        # --- CSV LOGGING ---
        # Only log if we are actually moving or receiving commands (to keep file clean)
        if abs(self.latest_u) > 0.01 or abs(self.actual_vel_odom) > 0.01:
            t = time.time() - self.start_time
            self.csv_writer.writerow([
                f"{t:.3f}", 
                f"{self.robot_x:.3f}", 
                f"{dist:.3f}", 
                f"{self.latest_u:.3f}", 
                f"{u_safe:.3f}", 
                f"{self.actual_vel_odom:.3f}"
            ])

        # Console Logging 
        if status != "SAFE":
            self.get_logger().warn(f"[{status}] Dist: {dist:.2f} | Cmd: {self.latest_u:.2f} -> Safe: {u_safe:.2f} | Real: {self.actual_vel_odom:.2f}")

    def close_file(self):
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info("CSV Log Closed.")

def main(args=None):
    rclpy.init(args=args)
    node = CBFSafetyNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close_file()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
