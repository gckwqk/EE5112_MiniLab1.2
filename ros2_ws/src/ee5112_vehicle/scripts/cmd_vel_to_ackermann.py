#!/usr/bin/env python3
# Task 3 owner: Mohammad Asif Bin Abdul Sahid (A0313732M), Group 13.
"""Task 3 Twist-to-joint converter for the supplied Gazebo Ackermann vehicle.

Developed with AI coding assistance.
Math/state logic is independent of ROS so tests can run with Python alone.
"""
from dataclasses import dataclass
import math
import signal
import time


def clamp(value, low, high):
    return max(low, min(high, value))


@dataclass(frozen=True)
class Limits:
    wheelbase: float = 0.20
    track_width: float = 0.16
    wheel_radius: float = 0.04
    max_steering_deg: float = 35.0
    max_speed: float = 0.50
    max_wheel_speed: float = 12.50
    max_acceleration: float = 0.40
    max_steering_rate: float = 1.50
    command_timeout: float = 0.50
    min_linear_speed: float = 0.0001

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError('All geometry and limit parameters must be finite and positive.')
        if self.max_steering_deg >= 89:
            raise ValueError('Steering limit must be below 89 degrees.')

    @property
    def max_curvature(self):
        t = math.tan(math.radians(self.max_steering_deg))
        # Limit the INNER front wheel, not just a virtual bicycle angle.
        return t / (self.wheelbase + self.track_width * t / 2)


def steering_angles(curvature, limits):
    k = clamp(curvature, -limits.max_curvature, limits.max_curvature)
    half_track = limits.track_width / 2
    return (math.atan(limits.wheelbase * k / (1 - half_track * k)),
            math.atan(limits.wheelbase * k / (1 + half_track * k)))


def limit_speed(speed, curvature, limits):
    # base_link is at chassis centre, L/2 ahead of the rear axle.
    # Its planar speed is |vx| * sqrt(1 + (k*L/2)^2).
    centre_limit = limits.max_speed / math.hypot(1, curvature * limits.wheelbase / 2)
    wheel_limit = limits.max_wheel_speed * limits.wheel_radius / (
        1 + abs(curvature) * limits.track_width / 2)
    bound = min(centre_limit, wheel_limit)
    return clamp(speed, -bound, bound)


@dataclass(frozen=True)
class Output:
    left_steering: float
    right_steering: float
    left_wheel: float
    right_wheel: float
    speed: float
    yaw_rate: float
    state: str


class Converter:
    def __init__(self, limits):
        self.limits = limits
        self.speed = 0.0
        self.curvature = 0.0
        self.request = None
        self.state = 'waiting'

    def stop(self, state='stopped'):
        self.request = None
        self.speed = 0.0
        self.state = state
        # Hold steering on stop; never slew steering abruptly to centre.
        return self.output()

    def accept(self, vx, wz, now, other_components=(0.0, 0.0, 0.0, 0.0)):
        if not all(math.isfinite(x) for x in (vx, wz, now, *other_components)):
            self.stop('invalid_command')
            return self.state
        if any(abs(x) > 1e-9 for x in other_components):
            self.stop('unsupported_axis')
            return self.state
        if abs(vx) < self.limits.min_linear_speed:
            self.stop('spin_not_supported' if abs(wz) > 1e-9 else 'stopped')
            return self.state
        k = clamp(wz / vx, -self.limits.max_curvature, self.limits.max_curvature)
        self.request = (limit_speed(vx, k, self.limits), k, now)
        self.state = 'driving'
        return self.state

    def step(self, now, dt):
        if self.request is None:
            return self.output()
        v_target, k_target, received = self.request
        if not math.isfinite(now) or now < received or now - received >= self.limits.command_timeout:
            return self.stop('timeout')
        if not math.isfinite(dt) or dt < 0:
            return self.stop('clock_reset')
        dt = min(dt, 0.1)  # No large jumps after a stalled executor.
        old_angles = steering_angles(self.curvature, self.limits)
        allowed = self.limits.max_steering_rate * dt
        # Find the greatest common curvature step respecting BOTH wheel slew
        # limits; independently clipping angles would break Ackermann geometry.
        low, high = 0.0, 1.0
        for _ in range(35):
            fraction = (low + high) / 2
            candidate = self.curvature + fraction * (k_target - self.curvature)
            angles = steering_angles(candidate, self.limits)
            if max(abs(a-b) for a, b in zip(angles, old_angles)) <= allowed:
                low = fraction
            else:
                high = fraction
        self.curvature += low * (k_target - self.curvature)
        # Reversals pass through zero before accelerating in the other direction.
        target = 0.0 if self.speed * v_target < 0 else v_target
        change = self.limits.max_acceleration * dt
        self.speed += clamp(target - self.speed, -change, change)
        # Geometric speed limits have priority over acceleration smoothing.
        self.speed = limit_speed(self.speed, self.curvature, self.limits)
        return self.output()

    def output(self):
        left, right = steering_angles(self.curvature, self.limits)
        omega = self.speed * self.curvature
        half_track = self.limits.track_width / 2
        return Output(left, right,
                      (self.speed - omega * half_track) / self.limits.wheel_radius,
                      (self.speed + omega * half_track) / self.limits.wheel_radius,
                      self.speed, omega, self.state)


def main(args=None):
    import rclpy
    from geometry_msgs.msg import Twist
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Float64MultiArray

    class CmdVelToAckermann(Node):
        def __init__(self):
            super().__init__('cmd_vel_to_ackermann')
            readonly = ParameterDescriptor(read_only=True)
            defaults = vars(Limits())
            for name, default in defaults.items():
                self.declare_parameter(name, default, readonly)
            self.declare_parameter('publish_rate', 50.0, readonly)
            self.declare_parameter('cmd_vel_topic', '/cmd_vel', readonly)
            self.declare_parameter('steering_topic', '/steering_controller/commands', readonly)
            self.declare_parameter('wheel_topic', '/rear_wheel_controller/commands', readonly)
            self.core = Converter(Limits(**{
                key: float(self.get_parameter(key).value) for key in defaults}))
            rate = float(self.get_parameter('publish_rate').value)
            if not math.isfinite(rate) or not 1 <= rate <= 500:
                raise ValueError('publish_rate must be between 1 and 500 Hz.')
            self.steer_pub = self.create_publisher(
                Float64MultiArray, self.get_parameter('steering_topic').value, 1)
            self.wheel_pub = self.create_publisher(
                Float64MultiArray, self.get_parameter('wheel_topic').value, 1)
            self.sub = self.create_subscription(
                Twist, self.get_parameter('cmd_vel_topic').value, self.on_command, 1)
            self.last_ros_time = self.get_clock().now().nanoseconds / 1e9
            self.last_warning = {}
            # Wall-clock timer keeps timeout active even while Gazebo is paused.
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(1 / rate, self.on_timer, clock=self.steady_clock)
            self.get_logger().info(
                f'Ready: {self.get_parameter("cmd_vel_topic").value} (Twist); '
                f'timeout={self.core.limits.command_timeout:.2f}s; '
                f'min rear-axle turning radius={1/self.core.limits.max_curvature:.3f}m.')

        def warn(self, state):
            messages = {
                'invalid_command': 'Invalid non-finite cmd_vel: stopping.',
                'unsupported_axis': 'Only linear.x and angular.z are supported: stopping.',
                'spin_not_supported': 'Ackermann cannot rotate in place. Use nonzero linear.x.',
                'timeout': 'cmd_vel timeout: stopping rear wheels.',
                'clock_reset': 'ROS clock moved backwards: send a fresh command.',
            }
            now = time.monotonic()
            if state in messages and now - self.last_warning.get(state, -math.inf) >= 2:
                self.get_logger().warning(messages[state])
                self.last_warning[state] = now

        def on_command(self, msg):
            state = self.core.accept(
                msg.linear.x, msg.angular.z, time.monotonic(),
                (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y))
            if state != 'driving':
                self.publish(self.core.output())  # Explicit stop is immediate.
                self.warn(state)

        def on_timer(self):
            now_ros = self.get_clock().now().nanoseconds / 1e9
            dt = now_ros - self.last_ros_time
            self.last_ros_time = now_ros
            previous_state = self.core.state
            result = self.core.step(time.monotonic(), dt)
            self.publish(result)
            if result.state != previous_state:
                self.warn(result.state)

        def publish(self, result):
            steering = Float64MultiArray()
            steering.data = [result.left_steering, result.right_steering]
            wheels = Float64MultiArray()
            wheels.data = [result.left_wheel, result.right_wheel]
            self.steer_pub.publish(steering)
            self.wheel_pub.publish(wheels)

    # Keep ROS alive briefly on Ctrl+C / SIGTERM so a final zero can be sent.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    stopping = False

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True

    old_handlers = {sig: signal.signal(sig, request_stop)
                    for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        node = CmdVelToAckermann()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.timer.cancel()
            if rclpy.ok():
                # Disallow any queued command from replacing the shutdown stop.
                node.destroy_subscription(node.sub)
                for _ in range(3):
                    node.publish(node.core.stop())
                    rclpy.spin_once(node, timeout_sec=0.02)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    main()
