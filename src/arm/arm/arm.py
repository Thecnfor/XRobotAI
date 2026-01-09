# /opt/projects/ros/src/arm/arm/arm_node.py
import rclpy
from rclpy.node import Node

class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')
        self.get_logger().info('你好')

def main():
    rclpy.init()
    node = ArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()