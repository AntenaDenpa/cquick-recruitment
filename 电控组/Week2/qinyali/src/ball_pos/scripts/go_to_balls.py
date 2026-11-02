#!/usr/bin/env python3
"""任务 2（路线 A · 闭环）：订阅 ball_pos，收齐 3 个球后按 1->2->3 依次走过去。

思路：
  1. 订阅 /turtle1/pose 拿到当前 (x, y, theta)；
  2. 算到目标点的距离 dist 和方位差 angle_err；
  3. 方位差大 -> 原地转（linear.x = 0），先对准再走；
  4. 对准后 -> 边走边微调，速度与距离成正比（越近越慢），到点切下一个。
"""
import math
import rclpy
from rclpy.node import Node
from ball_pos.msg import BallPos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# 任务 1.2 的三个固定坐标，不许改
TARGETS = {
    1: (2.0, 3.0),
    2: (5.5, 8.0),
    3: (9.0, 4.0),
}


def wrap_angle(a):
    """把角度归一到 [-pi, pi]。"""
    return math.atan2(math.sin(a), math.cos(a))


class GoToBalls(Node):
    def __init__(self):
        super().__init__('go_to_balls')
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose',
                                                 self.on_pose, 10)
        self.ball_sub = self.create_subscription(BallPos, 'ball_pos',
                                                 self.on_ball, 10)

        self.pose = None          # 乌龟当前位姿 (x, y, theta)
        self.balls = {}           # id -> (x, y)
        self.order = [1, 2, 3]
        self.step = -1            # -1 表示还没开始
        self.current = None       # 当前目标 (x, y)

        # 控制参数
        self.k_linear = 1.0       # 线速度与距离成正比
        self.k_angular = 4.0      # 角速度与方位差成正比
        self.max_linear = 1.5
        self.max_angular = 3.5
        self.arrive_thresh = 0.2  # 到点判定阈值（米）
        self.angle_thresh = 0.05  # 方位对准阈值（弧度）

        # 10 Hz 控制循环
        self.timer = self.create_timer(0.1, self.control)

    def on_ball(self, msg):
        if msg.id not in self.balls:
            self.balls[msg.id] = (msg.x, msg.y)
            self.get_logger().info(f'收到 球{msg.id} 坐标 ({msg.x}, {msg.y})')
        if len(self.balls) >= 3 and self.step == -1:
            self.step = 0
            self.current = TARGETS[self.order[0]]
            self.get_logger().info('收齐 3 个球，开始 1 -> 2 -> 3 导航')

    def on_pose(self, msg):
        self.pose = msg

    def control(self):
        if self.pose is None or self.current is None:
            return

        dx = self.current[0] - self.pose.x
        dy = self.current[1] - self.pose.y
        dist = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_err = wrap_angle(target_angle - self.pose.theta)

        twist = Twist()

        # 到点：报日志，切下一个
        if dist < self.arrive_thresh:
            self.get_logger().info(
                f'到达球 {self.order[self.step]}，'
                f'实际坐标 ({self.pose.x:.3f}, {self.pose.y:.3f})，'
                f'误差 {dist:.3f}'
            )
            self.step += 1
            if self.step >= len(self.order):
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.get_logger().info('三个球全部到达，任务完成')
                self.current = None
                return
            self.current = TARGETS[self.order[self.step]]
            return

        # 先对准再走
        if abs(angle_err) > self.angle_thresh:
            twist.angular.z = max(-self.max_angular,
                                  min(self.max_angular,
                                      self.k_angular * angle_err))
            twist.linear.x = 0.0
        else:
            # 越近越慢
            twist.linear.x = min(self.k_linear * dist, self.max_linear)
            # 边走边微调
            twist.angular.z = max(-self.max_angular,
                                  min(self.max_angular,
                                      self.k_angular * angle_err))

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GoToBalls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
