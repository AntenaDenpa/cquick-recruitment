#!/usr/bin/env python3
"""任务 1 发布者：把 3 个球的坐标循环发布到 ball_pos 话题。"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from ball_pos.msg import BallPos

# 1.2 节规定的三个固定坐标，不许改
BALLS = [
    (1, 2.0, 3.0),
    (2, 5.5, 8.0),
    (3, 9.0, 4.0),
]


class BallPosPublisher(Node):
    def __init__(self):
        super().__init__('ball_pos_publisher')
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)

        # 默认 2 Hz；可用 ros2 param set /ball_pos_publisher rate 5.0 在线改
        self.declare_parameter('rate', 2.0)
        self.add_on_set_parameters_callback(self.on_param_change)
        self._create_timer_from_param()

    def _create_timer_from_param(self):
        rate = float(self.get_parameter('rate').value)
        period = 1.0 / rate
        if hasattr(self, 'timer'):
            self.timer.cancel()
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(f'发布频率 {rate} Hz')

    def on_param_change(self, params):
        for p in params:
            if p.name == 'rate':
                if p.value <= 0.0:
                    return SetParametersResult(successful=False,
                                               reason='rate 必须大于 0')
                self._create_timer_from_param()
        return SetParametersResult(successful=True)

    def timer_callback(self):
        for ball_id, x, y in BALLS:
            msg = BallPos()
            msg.id = ball_id
            msg.x = float(x)
            msg.y = float(y)
            self.publisher_.publish(msg)
            self.get_logger().info(f'发布 球{msg.id} 坐标 ({msg.x}, {msg.y})')


def main(args=None):
    rclpy.init(args=args)
    node = BallPosPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
