#!/usr/bin/env python3
"""任务 2（路线 B · 开环兜底）：按时间估算每段转多久、走多久，不做实时修正。

注意：Twist 是速度不是位移，误差会累积，越到后面越偏。推荐优先用 go_to_balls.py
（闭环）。本脚本只作兜底演示。
"""
import math
import rclpy
from rclpy.node import Node
from ball_pos.msg import BallPos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

TARGETS = {
    1: (2.0, 3.0),
    2: (5.5, 8.0),
    3: (9.0, 4.0),
}


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class GoToBallsOpenLoop(Node):
    def __init__(self):
        super().__init__('go_to_balls_openloop')
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose',
                                                 self.on_pose, 10)
        self.ball_sub = self.create_subscription(BallPos, 'ball_pos',
                                                 self.on_ball, 10)

        self.pose = None
        self.balls = {}
        self.order = [1, 2, 3]
        self.step = -1

        # 固定速度
        self.linear = 0.5      # m/s
        self.angular = 1.0     # rad/s

        # 状态机
        self.phase = 'idle'    # idle / rotate / drive / done
        self.phase_elapsed = 0.0
        self.rotate_time = 0.0
        self.rotate_dir = 1.0
        self.drive_time = 0.0

        self.timer = self.create_timer(0.1, self.control)

    def on_ball(self, msg):
        if msg.id not in self.balls:
            self.balls[msg.id] = (msg.x, msg.y)
            self.get_logger().info(f'收到 球{msg.id} 坐标 ({msg.x}, {msg.y})')
        if len(self.balls) >= 3 and self.step == -1 and self.pose is not None:
            self.step = 0
            self.get_logger().info('收齐 3 个球，开始 1 -> 2 -> 3 导航（开环）')
            self.start_segment()

    def on_pose(self, msg):
        self.pose = msg

    def start_segment(self):
        if self.step >= len(self.order):
            self.phase = 'done'
            return
        tx, ty = TARGETS[self.order[self.step]]
        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_err = wrap_angle(target_angle - self.pose.theta)

        # 转多少角度 = 角速度 × 时间  ->  时间 = 角度 / 角速度
        self.rotate_time = abs(angle_err) / self.angular
        self.rotate_dir = 1.0 if angle_err >= 0 else -1.0
        # 走多远 = 线速度 × 时间  ->  时间 = 距离 / 线速度
        self.drive_time = dist / self.linear

        self.phase = 'rotate'
        self.phase_elapsed = 0.0
        self.get_logger().info(
            f'目标球 {self.order[self.step]}：需转 {self.rotate_time:.2f}s，'
            f'需走 {self.drive_time:.2f}s'
        )

    def control(self):
        if self.pose is None or self.phase in ('idle', 'done'):
            return

        dt = 0.1
        self.phase_elapsed += dt
        twist = Twist()

        if self.phase == 'rotate':
            if self.phase_elapsed < self.rotate_time:
                twist.angular.z = self.rotate_dir * self.angular
            else:
                twist.angular.z = 0.0
                self.phase = 'drive'
                self.phase_elapsed = 0.0
        elif self.phase == 'drive':
            if self.phase_elapsed < self.drive_time:
                twist.linear.x = self.linear
            else:
                twist.linear.x = 0.0
                tx, ty = TARGETS[self.order[self.step]]
                err = math.hypot(tx - self.pose.x, ty - self.pose.y)
                self.get_logger().info(
                    f'到达球 {self.order[self.step]}，'
                    f'实际坐标 ({self.pose.x:.3f}, {self.pose.y:.3f})，'
                    f'误差 {err:.3f}'
                )
                self.step += 1
                if self.step >= len(self.order):
                    self.phase = 'done'
                    self.get_logger().info('三个球全部到达（开环）')
                else:
                    self.start_segment()
                    return

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GoToBallsOpenLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
