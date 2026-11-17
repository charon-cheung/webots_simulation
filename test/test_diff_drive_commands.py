"""Check commanded wheel behavior with fake Webots actuators."""

from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist
from webots_simulation import diff_drive_driver as module


class Motor:
    def getMaxVelocity(self):
        return 12.0

    def setPosition(self, value):
        self.position = value

    def setVelocity(self, value):
        self.velocity = value


@pytest.fixture
def base(monkeypatch):
    motors = {'left_wheel_motor': Motor(), 'right_wheel_motor': Motor()}
    received = {}
    node = SimpleNamespace(
        create_subscription=lambda kind, topic, callback, qos: received.update({topic: callback}),
        get_logger=lambda: SimpleNamespace(info=lambda text: None),
    )
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.rclpy, 'create_node', lambda name: node)
    monkeypatch.setattr(module.rclpy, 'spin_once', lambda *args, **kwargs: None)
    clock = [100.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    driver = module.DiffDriveDriver()
    driver.init(SimpleNamespace(robot=SimpleNamespace(getDevice=motors.get)), {})

    def send(x=0.0, yaw=0.0, y=0.0, topic='cmd_vel'):
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = x, y, yaw
        received[topic](command)
        driver.step()
        return tuple(motor.velocity for motor in motors.values())

    return driver, motors, clock, send


@pytest.mark.parametrize('x,yaw,expected', [
    (0.3, 0.0, (3.0, 3.0)),
    (-0.3, 0.0, (-3.0, -3.0)),
    (0.0, 0.6, (-1.71, 1.71)),
    (0.0, -0.6, (1.71, -1.71)),
    (0.3, 0.6, (1.29, 4.71)),
])
def test_planar_motion(base, x, yaw, expected):
    assert base[3](x, yaw) == pytest.approx(expected)


def test_limits_preserve_turn_curvature(base):
    left, right = base[3](3.0, 6.0)
    assert max(abs(left), abs(right)) == pytest.approx(12.0)
    assert left / right == pytest.approx(1.29 / 4.71)


def test_stop_lateral_input_and_missing_commands(base):
    driver, motors, clock, send = base
    driver.step()
    assert all(motor.velocity == 0.0 for motor in motors.values())
    send(0.3)
    assert send() == (0.0, 0.0)
    assert send(y=0.3) == (0.0, 0.0)
    send(0.3)
    clock[0] += 1.01
    driver.step()
    assert all(motor.velocity == 0.0 for motor in motors.values())
    assert send(-0.3) == pytest.approx((-3.0, -3.0))


@pytest.mark.parametrize('x,yaw', [
    (float('nan'), 0.0), (0.0, float('inf')), (float('-inf'), 0.0), (1e308, 1e308),
])
def test_nonfinite_command_stops_both_wheels(base, x, yaw):
    send = base[3]
    send(0.3)
    assert send(x, yaw) == (0.0, 0.0)


def test_local_teleop_topic_controls_and_stops(base):
    send = base[3]
    assert send(0.5, topic='cmd_vel_nav') == pytest.approx((5.0, 5.0))
    assert send(yaw=1.0, topic='cmd_vel_nav') == pytest.approx((-2.85, 2.85))
    assert send(topic='cmd_vel_nav') == (0.0, 0.0)
