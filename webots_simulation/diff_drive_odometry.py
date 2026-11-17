"""Publish encoder translation and relative IMU heading in simulation time."""

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time
from tf2_ros import TransformBroadcaster


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class DiffDriveOdometry:
    def __init__(self, node, robot, radius):
        self._robot = robot
        self._radius = radius
        timestep = int(robot.getBasicTimeStep())
        self._wheels = [robot.getDevice(name) for name in
                        ('left_wheel_sensor', 'right_wheel_sensor')]
        self._imu = robot.getDevice('imu_sensor')
        for sensor in [*self._wheels, self._imu]:
            sensor.enable(timestep)
        self._publisher = node.create_publisher(Odometry, 'odom', 10)
        self._tf = TransformBroadcaster(node)
        self._previous = None
        self._heading_origin = None
        self._x = self._y = self._yaw = 0.0

    def step(self):
        now = self._robot.getTime()
        left, right = (sensor.getValue() for sensor in self._wheels)
        heading = self._imu.getRollPitchYaw()[2]
        if not all(math.isfinite(v) for v in (now, left, right, heading)):
            # Rebase wheel readings after a gap instead of integrating stale data.
            self._previous = None
            return
        linear = angular = 0.0
        if self._previous is not None and now < self._previous[0]:
            self._previous = None
            self._heading_origin = None
            self._x = self._y = self._yaw = 0.0
        if self._heading_origin is None:
            self._heading_origin = heading
        yaw = wrap(heading - self._heading_origin)
        if self._previous is not None:
            old_time, old_left, old_right = self._previous
            dt = now - old_time
            if dt <= 0.0:
                return
            distance = self._radius * ((left - old_left) + (right - old_right)) / 2.0
            dyaw = wrap(yaw - self._yaw)
            # Integrate the measured arc, including the straight-line limit.
            half = dyaw / 2.0
            chord = distance * (math.sin(half) / half if abs(half) > 1e-9 else 1.0)
            self._x += chord * math.cos(self._yaw + half)
            self._y += chord * math.sin(self._yaw + half)
            linear, angular = distance / dt, dyaw / dt
        self._previous = (now, left, right)
        self._yaw = yaw

        odom = Odometry()
        odom.header.stamp = Time(nanoseconds=round(now * 1e9)).to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        # Planar estimates; unobserved z/roll/pitch have large uncertainty.
        for index, variance in zip((0, 7, 14, 21, 28, 35),
                                   (0.01, 0.01, 1e6, 1e6, 1e6, 0.01)):
            odom.pose.covariance[index] = variance
            odom.twist.covariance[index] = variance
        self._publisher.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.rotation = odom.pose.pose.orientation
        self._tf.sendTransform(transform)
