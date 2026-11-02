#!/usr/bin/env python3
"""任务 1 订阅者：订阅 ball_pos 并打印。"""
import rclpy
from rclpy.node import Node
from ball_pos.msg import BallPos


class BallPosSubscriber(Node):
    def __init__(self):
        super().__init__('ball_pos_subscriber')
        self.subscription = self.create_subscription(
            BallPos, 'ball_pos', self.on_ball, 10)

    def on_ball(self, msg):
        self.get_logger().info(f'收到 球{msg.id} 坐标 ({msg.x}, {msg.y})')


def main(args=None):
    rclpy.init(args=args)
    node = BallPosSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
