#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import random

class SimpleExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(10.0, self.send_random_goal)  # every 10 seconds

    def send_random_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        # Set random goal coordinates (adjust range to fit your map)
        goal.pose.position.x = random.uniform(-2.0, 2.0)
        goal.pose.position.y = random.uniform(-2.0, 2.0)
        goal.pose.orientation.w = 1.0  # facing forward
        self.get_logger().info(f'Sending goal: x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}')
        self.goal_pub.publish(goal)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
