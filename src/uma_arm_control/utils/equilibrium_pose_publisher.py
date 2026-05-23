#!/usr/bin/python3

# Author: Juan M. Gandarias (http://jmgandarias.com)
# email: jmgandarias@uma.es
# 
# This script creates trackbars for equilibrium pose (X and Y). 
# Run it:
# python3 equilibrium_pose_trackbar_publisher.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tkinter as tk

class EquilibriumPosePublisher(Node):
    def __init__(self):
        super().__init__('equilibrium_pose_publisher')
        self.publisher_pose = self.create_publisher(PoseStamped, 'equilibrium_pose', 10)
        self.pose_x = 0.0
        self.pose_y = 0.0

        self.root = tk.Tk()
        self.root.title("Equilibrium Pose Publisher")

        self.create_trackbar('Pose X', -1.6, 1.6, self.update_pose_x)
        self.create_trackbar('Pose Y', -1.6, 1.6, self.update_pose_y)

    def create_trackbar(self, label, min_val, max_val, callback):
        frame = tk.Frame(self.root)
        frame.pack()
        tk.Label(frame, text=f"{min_val}").pack(side=tk.LEFT)
        trackbar = tk.Scale(frame, label=label, from_=min_val, to=max_val, orient=tk.HORIZONTAL, command=callback, resolution=0.01, length=400, sliderlength=30)
        trackbar.pack(side=tk.LEFT)
        tk.Label(frame, text=f"{max_val}").pack(side=tk.LEFT)

    def update_pose_x(self, value):
        self.pose_x = float(value)
        self.publish_equilibrium_pose()

    def update_pose_y(self, value):
        self.pose_y = float(value)
        self.publish_equilibrium_pose()

    def publish_equilibrium_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"  # Set the appropriate frame_id
        msg.pose.position.x = 0.0
        msg.pose.position.y = self.pose_x
        msg.pose.position.z = self.pose_y
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0  # Assuming no rotation
        self.publisher_pose.publish(msg)
        self.get_logger().info(f'Publishing: X Pos = {self.pose_x}, Y Pos = {self.pose_y}')

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = EquilibriumPosePublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()