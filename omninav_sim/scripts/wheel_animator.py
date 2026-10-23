#!/usr/bin/env python3
"""Spin the wheels in RViz to match how the simulated robot is actually moving.

In simulation the body is moved by Gazebo's VelocityControl and the wheels are frictionless,
so the real wheel joints never turn. This node reads the body velocity from /odom, runs the
mecanum inverse kinematics (the same equations the real robot's bridge uses), integrates each
wheel's angle and publishes /joint_states, so robot_state_publisher draws turning wheels.

Wheel speed (rad/s), with k = lx + ly and r = wheel radius:
    FL = (vx - vy - k*wz) / r        FR = (vx + vy + k*wz) / r
    RL = (vx + vy - k*wz) / r        RR = (vx - vy + k*wz) / r
"""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ['front_left_wheel_joint', 'front_right_wheel_joint',
          'back_left_wheel_joint', 'back_right_wheel_joint']


class WheelAnimator(Node):
    def __init__(self):
        super().__init__('wheel_animator')
        self.r = self.declare_parameter('wheel_radius', 0.05005).value
        self.k = self.declare_parameter('k', 0.282).value          # wheel_x_offset + wheel_y_offset
        rate = self.declare_parameter('rate', 30.0).value

        self.vx = self.vy = self.wz = 0.0
        self.angles = [0.0, 0.0, 0.0, 0.0]
        self.last = None

        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(1.0 / rate, self.tick)

    def on_odom(self, msg):
        t = msg.twist.twist                     # body-frame velocity
        self.vx, self.vy, self.wz = t.linear.x, t.linear.y, t.angular.z

    def tick(self):
        now = self.get_clock().now()
        if self.last is not None:
            dt = (now - self.last).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:                  # skip sim-time jumps and pauses
                vx, vy, kw = self.vx, self.vy, self.k * self.wz
                speeds = [vx - vy - kw, vx + vy + kw, vx + vy - kw, vx - vy + kw]
                for i, v in enumerate(speeds):
                    self.angles[i] += v / self.r * dt
        self.last = now

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = JOINTS
        js.position = list(self.angles)
        self.pub.publish(js)


def main():
    rclpy.init()
    rclpy.spin(WheelAnimator())


if __name__ == '__main__':
    main()
