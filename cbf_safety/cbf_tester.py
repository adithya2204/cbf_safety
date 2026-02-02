#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import csv, time

class CBFTester(Node):
    def __init__(self):
        super().__init__('cbf_tester')
        print("WARNING: Max Speed Command. Ensure Safety Node is ON.")
        input("Press ENTER...")

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        
        self.f = open('cbf_data.csv', 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['timestamp', 'odom_x', 'act_vel', 'safe_cmd'])
        
        self.start = time.time()
        self.vx = 0.0; self.ox = 0.0; self.safe_v = 0.0
        self.create_timer(0.1, self.loop)

    def odom_cb(self, msg):
        self.ox = msg.pose.pose.position.x
        self.vx = msg.twist.twist.linear.x

    def cmd_cb(self, msg): self.safe_v = msg.linear.x

    def loop(self):
        msg = Twist(); msg.linear.x = 0.22
        self.pub.publish(msg)
        
        t = time.time() - self.start
        self.w.writerow([f"{t:.3f}", f"{self.ox:.3f}", f"{self.vx:.3f}", f"{self.safe_v:.3f}"])
        
        if t > 8.0:
            self.pub.publish(Twist()) # Stop
            self.f.close(); self.destroy_node(); rclpy.shutdown()

def main(): rclpy.init(); rclpy.spin(CBFTester())
if __name__ == '__main__': main()