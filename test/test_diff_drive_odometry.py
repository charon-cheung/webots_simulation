"""Observable odometry and TF behavior with measured wheel/IMU samples."""

import math
from types import SimpleNamespace

import pytest
from webots_simulation import diff_drive_odometry as module


@pytest.fixture
def odometry(monkeypatch):
    state = [1.0, 0.0, 0.0, 0.7]
    messages, transforms = [], []
    devices = {
        'left_wheel_sensor': SimpleNamespace(enable=lambda _: None, getValue=lambda: state[1]),
        'right_wheel_sensor': SimpleNamespace(enable=lambda _: None, getValue=lambda: state[2]),
        'imu_sensor': SimpleNamespace(enable=lambda _: None,
                                     getRollPitchYaw=lambda: (0.0, 0.0, state[3])),
    }
    robot = SimpleNamespace(getBasicTimeStep=lambda: 10, getTime=lambda: state[0],
                            getDevice=devices.get)
    node = SimpleNamespace(create_publisher=lambda *args: SimpleNamespace(publish=messages.append))
    monkeypatch.setattr(module, 'TransformBroadcaster',
                        lambda _: SimpleNamespace(sendTransform=transforms.append))
    estimator = module.DiffDriveOdometry(node, robot, 0.1)

    def sample(time, left, right, yaw):
        state[:] = [time, left, right, yaw]
        estimator.step()
        return messages[-1] if messages else None

    return sample, messages, transforms


def test_measured_forward_reverse_and_matching_tf(odometry):
    sample, messages, transforms = odometry
    sample(1.0, 0.0, 0.0, 0.7)
    message = sample(2.0, 3.0, 3.0, 0.7)
    assert message.pose.pose.position.x == pytest.approx(0.3)
    assert message.pose.pose.position.y == pytest.approx(0.0)
    assert message.twist.twist.linear.x == pytest.approx(0.3)
    assert message.pose.pose.orientation.w == pytest.approx(1.0)
    assert message.header.frame_id == 'odom'
    assert message.child_frame_id == 'base_footprint'
    assert message.header.stamp.sec == 2
    transform = transforms[-1]
    assert transform.header == message.header
    assert transform.child_frame_id == message.child_frame_id
    assert transform.transform.translation.x == message.pose.pose.position.x
    assert transform.transform.rotation == message.pose.pose.orientation
    message = sample(3.0, 1.0, 1.0, 0.7)
    assert message.pose.pose.position.x == pytest.approx(0.1)
    assert message.twist.twist.linear.x == pytest.approx(-0.2)


def test_spin_uses_imu_and_wraps_heading(odometry):
    sample, _, _ = odometry
    sample(1.0, 0.0, 0.0, math.pi - 0.1)
    message = sample(1.1, -1.0, 1.0, -math.pi + 0.1)
    assert message.pose.pose.position.x == pytest.approx(0.0)
    assert message.twist.twist.angular.z == pytest.approx(2.0)
    assert message.pose.pose.orientation.z == pytest.approx(math.sin(0.1))


def test_measured_quarter_circle(odometry):
    sample, _, _ = odometry
    sample(1.0, 0.0, 0.0, 0.0)
    # Centre travels pi/2 m while yaw turns pi/2: radius 1 m.
    message = sample(2.0, 5 * math.pi - 2.0, 5 * math.pi + 2.0, math.pi / 2)
    assert message.pose.pose.position.x == pytest.approx(1.0)
    assert message.pose.pose.position.y == pytest.approx(1.0)


def test_invalid_feedback_rebases_without_position_jump(odometry):
    sample, messages, _ = odometry
    sample(1.0, 0.0, 0.0, 0.0)
    sample(2.0, float('nan'), 1.0, 0.0)
    assert len(messages) == 1
    message = sample(3.0, 20.0, 20.0, 0.0)
    assert message.pose.pose.position.x == 0.0
    message = sample(4.0, 21.0, 21.0, 0.0)
    assert message.pose.pose.position.x == pytest.approx(0.1)


def test_duplicate_and_reset_simulation_time(odometry):
    sample, messages, _ = odometry
    sample(1.0, 0.0, 0.0, 0.0)
    sample(2.0, 3.0, 3.0, 0.5)
    sample(2.0, 3.0, 3.0, 0.5)
    assert len(messages) == 2
    message = sample(0.0, 0.0, 0.0, -0.4)
    assert message.pose.pose.position.x == 0.0
    assert message.pose.pose.orientation.w == 1.0
    assert message.twist.twist.linear.x == 0.0
