#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import time, csv, math, threading

class Choreographer(Node):
    def __init__(self):
        super().__init__('choreographer_node')
        
        try:
            val = input("Enter Bias Factor (Default 1.0): ")
            self.bias = float(val) if val.strip() else 1.0
        except: self.bias = 1.0
            
        print(f"Bias: {self.bias}. Press ENTER to start.")
        input()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_cb, 
            qos_profile=qos_profile_sensor_data 
        )
        
        self.f = open('choreography_data.csv', 'w', newline='')
        self.writer = csv.writer(self.f)
        self.writer.writerow(['timestamp', 'pos_x', 'pos_y', 'cmd_lin', 'cmd_ang'])
        
        self.cx = 0.0; self.cy = 0.0; self.running = True
        self.thread = threading.Thread(target=self.run_seq)
        self.thread.start()

    def odom_cb(self, msg):
        self.cx = msg.pose.pose.position.x
        self.cy = msg.pose.pose.position.y

    def pub(self, lin, ang):
        msg = Twist()
        msg.linear.x = float(lin); msg.angular.z = float(ang)
        self.cmd_pub.publish(msg)
        self.writer.writerow([time.time(), f"{self.cx:.3f}", f"{self.cy:.3f}", lin, ang])

    def run_seq(self):
        v = 0.1; w = 0.5
        
        # Distances: 1.5m, 0.75m, 0.5m
        t_1_5 = 1.5 / v
        t_0_75 = 0.75 / v
        t_0_5 = 0.5 / v
        
        # Bias applied to Turns
        t_90 = (1.57 / w) * self.bias
        t_360 = (6.28 / w) * self.bias
        
        # Circle R=0.5
        t_circ = (math.pi) / v
        t_half_circ = t_circ / 2
        w_circ = 0.2 * self.bias

        seq = [
            (v, 0.0, t_1_5, "Fwd 1.5m"),
            (0.0, w, t_90, "Left 90"),
            (v, 0.0, t_0_75, "Fwd 0.75m"),
            (0.0, w, t_90, "Left 90"),
            (v, 0.0, t_0_5, "Fwd 0.5m"),
            (0.0, w, t_360, "Spin 360"),
            (v, -w_circ, t_half_circ, "Circle R0.5")
        ]

        for lin, ang, dur, desc in seq:
            print(f"Running: {desc}")
            s = time.time()
            while time.time() - s < dur and self.running:
                self.pub(lin, ang)
                time.sleep(0.1)
            
            print(f"DONE: {desc}. MEASURE NOW.")
            print("Press ENTER to continue...")
            while True:
                self.pub(0.0, 0.0)
                break 
            input() 

        self.f.close(); self.running = False; rclpy.shutdown()

def main(): rclpy.init(); rclpy.spin(Choreographer())
if __name__ == '__main__': main()
