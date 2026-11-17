"""Translate planar Twist commands into the two Webots wheel velocities."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter

from webots_simulation.diff_drive_odometry import DiffDriveOdometry


class DiffDriveDriver:
    def init(self, webots_node, properties):
        self._radius = float(properties.get('wheelRadius', '0.10'))
        self._track = float(properties.get('wheelSeparation', '0.57'))
        self._timeout = float(properties.get('cmdVelTimeout', '0.3'))
        if not all(math.isfinite(v) and v > 0 for v in
                   (self._radius, self._track, self._timeout)):
            raise ValueError('Wheel geometry and command timeout must be positive and finite')
        self._motors = [webots_node.robot.getDevice(name) for name in
                        ('left_wheel_motor', 'right_wheel_motor')]
        self._limits = [motor.getMaxVelocity() for motor in self._motors]
        for motor in self._motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
        self._target = (0.0, 0.0)
        self._last_command = float('-inf')
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node(
            'diff_drive_driver',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self._odometry = DiffDriveOdometry(self._node, webots_node.robot, self._radius)
        self._subscription = self._node.create_subscription(
            Twist, 'cmd_vel', self._on_command, 1)
        # This host's teleop uses cmd_vel_nav; upstream teleop uses cmd_vel.
        self._teleop_subscription = self._node.create_subscription(
            Twist, 'cmd_vel_nav', self._on_command, 1)
        self._node.get_logger().info(
            f'DiffDrive ready: cmd_vel / cmd_vel_nav (Twist), timeout={self._timeout}s')

    def _on_command(self, command):
        # Differential drive only supports forward velocity and yaw rate.
        linear, angular = command.linear.x, command.angular.z
        left = (linear - angular * self._track / 2.0) / self._radius
        right = (linear + angular * self._track / 2.0) / self._radius
        if not all(math.isfinite(v) for v in (left, right)):
            self._target = (0.0, 0.0)
        else:
            # Scale both wheels together to preserve the requested curvature.
            scale = max(1.0, abs(left) / self._limits[0], abs(right) / self._limits[1])
            self._target = (left / scale, right / scale)
        self._last_command = time.monotonic()

    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0)
        self._odometry.step()
        target = self._target
        if time.monotonic() - self._last_command >= self._timeout:
            target = (0.0, 0.0)
        for motor, velocity in zip(self._motors, target):
            motor.setVelocity(velocity)

    def stop(self):
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
