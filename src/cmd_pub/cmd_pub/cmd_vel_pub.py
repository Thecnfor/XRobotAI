import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelPublisher(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_publisher')
        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._msg = Twist()
        self._msg.linear.x = 0.2
        self._msg.angular.z = 0.5
        self._timer = self.create_timer(0.1, self._publish)

    def _publish(self) -> None:
        self._publisher.publish(self._msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

