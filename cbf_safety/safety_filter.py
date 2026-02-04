#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data 
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class CBFSafetyNode(Node):
    def __init__(self):
        super().__init__('cbf_safety_filter')

        # --- Parameters ---
        self.declare_parameter('wall_pos', 2.0)
        self.declare_parameter('gamma', 0.1)
        
        self.wall_pos = self.get_parameter('wall_pos').value
        self.gamma = self.get_parameter('gamma').value
        
        # --- Variables (Shared Resources) ---
        self.robot_x = 0.0
        self.latest_u = 0.0
        self.odom_received = False
        self.latest_angular_z = 0.0

        # --- Communication ---
        self.sub_odom = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            qos_profile=qos_profile_sensor_data 
        )

        # Command Subscriber
        self.sub_cmd = self.create_subscription(
            Twist, 
            '/cmd_vel_raw', 
            self.cmd_callback, 
            5  # Fixed: Changed 05 to 5
        )
        
        self.pub_safe = self.create_publisher(Twist, '/cmd_vel', 5)

        # 3. THE CONTROL  LOOP
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(f"CBF Decoupled Loop Active. Wall : {self.wall_pos}m")

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
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

        # Publish
        safe_msg = Twist()
        safe_msg.linear.x = float(u_safe)
        safe_msg.angular.z = self.latest_angular_z
        self.pub_safe.publish(safe_msg)

        # Logging 
        if status != "SAFE":
            self.get_logger().warn(f"[{status}] Pos: {self.robot_x:.2f} | Dist: {dist:.2f} | In: {self.latest_u:.2f} -> Out: {u_safe:.2f}")
        elif self.latest_u > 0.01:
            self.get_logger().info(f"[MOVING] Pos: {self.robot_x:.2f} | Dist: {dist:.2f}", throttle_duration_sec=0.5)

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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()