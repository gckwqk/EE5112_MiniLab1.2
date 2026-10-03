#!/usr/bin/env python3

"""
EE5112 MiniLab 1.2
Task 2 - Keyboard Ackermann Teleoperation

Controls
--------
W / S   increase / decrease longitudinal speed
A / D   steer further left / right
SPACE   set speed to zero
C       centre steering
X       stop and centre steering
R       reset the live-plot/model origin
Q       stop safely and quit

The node commands the existing steering and rear-wheel controllers and
publishes the same live Task 2 interfaces used by live_trajectory_plot.py:

    /task2/command_model_pose
    /task2/measured_model_pose
    /task2/telemetry

Telemetry layout:
    [elapsed,
     commanded_v,
     command_model_v,
     measured_v,
     odom_v,
     commanded_delta,
     measured_delta]

All steering values in telemetry are radians.
"""

import math
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class AckermannTeleop(Node):

    WHEELBASE = 0.20
    TRACK_WIDTH = 0.16
    WHEEL_RADIUS = 0.04
    REF_OFFSET = WHEELBASE / 2.0

    MAX_SPEED = 0.50
    MAX_STEERING = math.radians(35.0)

    SPEED_STEP = 0.02
    STEERING_STEP = math.radians(5.0)

    def __init__(self):
        super().__init__('ackermann_teleop')

        # Commands.
        self.command_speed = 0.0
        self.command_delta = 0.0

        # Gazebo state.
        self.have_odom = False
        self.actual_x = 0.0
        self.actual_y = 0.0
        self.actual_theta = 0.0
        self.odom_speed = 0.0

        # Joint measurements.
        self.rear_left_velocity = 0.0
        self.rear_right_velocity = 0.0
        self.front_left_steering = 0.0
        self.front_right_steering = 0.0
        self.wheel_derived_speed = 0.0
        self.measured_delta = 0.0

        # Model states.
        self.command_model_x = 0.0
        self.command_model_y = 0.0
        self.command_model_theta = 0.0
        self.command_rear_x = 0.0
        self.command_rear_y = 0.0

        self.measured_model_x = 0.0
        self.measured_model_y = 0.0
        self.measured_model_theta = 0.0
        self.measured_rear_x = 0.0
        self.measured_rear_y = 0.0

        self.start_time = None
        self.last_model_time = None
        self.reset_requested = False
        self.quit_requested = False

        # Controller publishers.
        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_controller/commands',
            10
        )
        self.rear_wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/rear_wheel_controller/commands',
            10
        )

        # Live dashboard publishers.
        self.command_model_pub = self.create_publisher(
            Pose2D,
            '/task2/command_model_pose',
            10
        )
        self.measured_model_pub = self.create_publisher(
            Pose2D,
            '/task2/measured_model_pose',
            10
        )
        self.telemetry_pub = self.create_publisher(
            Float64MultiArray,
            '/task2/telemetry',
            10
        )

        # Feedback.
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            50
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            50
        )

        self.create_timer(0.02, self.control_loop)

        self.keyboard_thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True
        )
        self.keyboard_thread.start()

        self.print_help()

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def quaternion_to_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def rear_to_reference(self, rear_x, rear_y, theta):
        return (
            rear_x + self.REF_OFFSET * math.cos(theta),
            rear_y + self.REF_OFFSET * math.sin(theta)
        )

    def reference_to_rear(self, ref_x, ref_y, theta):
        return (
            ref_x - self.REF_OFFSET * math.cos(theta),
            ref_y - self.REF_OFFSET * math.sin(theta)
        )

    def calculate_ackermann_angles(self, delta):
        if abs(delta) < 1.0e-9:
            return 0.0, 0.0

        radius = self.WHEELBASE / math.tan(abs(delta))
        half_track = self.TRACK_WIDTH / 2.0

        inner = math.atan(
            self.WHEELBASE / max(radius - half_track, 1.0e-6)
        )
        outer = math.atan(
            self.WHEELBASE / (radius + half_track)
        )

        if delta > 0.0:
            left, right = inner, outer
        else:
            left, right = -outer, -inner

        left = max(-self.MAX_STEERING, min(self.MAX_STEERING, left))
        right = max(-self.MAX_STEERING, min(self.MAX_STEERING, right))
        return left, right

    def calculate_rear_wheel_velocities(self, speed, delta):
        curvature = math.tan(delta) / self.WHEELBASE
        half_track = self.TRACK_WIDTH / 2.0

        left_speed = speed * (1.0 - half_track * curvature)
        right_speed = speed * (1.0 + half_track * curvature)

        return (
            left_speed / self.WHEEL_RADIUS,
            right_speed / self.WHEEL_RADIUS
        )

    def equivalent_delta_from_front_joints(self, left, right):
        # Reconstruct curvature from each Ackermann front wheel.
        # For near-zero steering, avoid numerical noise.
        if abs(left) < 1.0e-5 and abs(right) < 1.0e-5:
            return 0.0

        estimates = []

        # tan(delta_left) = L / (R - W/2) for a left turn and
        # the sign reverses for a right turn. Curvature form is
        # robust for both signs:
        # kappa_left  = tan(dl) / (L + W/2 * tan(dl))
        # kappa_right = tan(dr) / (L - W/2 * tan(dr))
        tl = math.tan(left)
        tr = math.tan(right)
        half_track = self.TRACK_WIDTH / 2.0

        denom_l = self.WHEELBASE + half_track * tl
        denom_r = self.WHEELBASE - half_track * tr

        if abs(denom_l) > 1.0e-6:
            estimates.append(tl / denom_l)
        if abs(denom_r) > 1.0e-6:
            estimates.append(tr / denom_r)

        if not estimates:
            return 0.0

        curvature = sum(estimates) / len(estimates)
        delta = math.atan(self.WHEELBASE * curvature)
        return max(-self.MAX_STEERING, min(self.MAX_STEERING, delta))

    def integrate_bicycle_model(self, x, y, theta, speed, delta, dt):
        x_new = x + speed * math.cos(theta) * dt
        y_new = y + speed * math.sin(theta) * dt
        yaw_rate = speed / self.WHEELBASE * math.tan(delta)
        theta_new = self.normalize_angle(theta + yaw_rate * dt)
        return x_new, y_new, theta_new

    def odom_callback(self, msg):
        self.actual_x = msg.pose.pose.position.x
        self.actual_y = msg.pose.pose.position.y
        self.actual_theta = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.odom_speed = math.copysign(
            math.hypot(vx, vy),
            self.command_speed if abs(self.command_speed) > 1.0e-6 else 1.0
        )

        if not self.have_odom:
            self.have_odom = True
            self.reset_models_to_odom()
            now = self.get_clock().now()
            self.start_time = now
            self.last_model_time = now
            self.get_logger().info(
                'Odometry received. Teleoperation is active.'
            )

    def joint_state_callback(self, msg):
        positions = {
            name: msg.position[i]
            for i, name in enumerate(msg.name)
            if i < len(msg.position)
        }
        velocities = {
            name: msg.velocity[i]
            for i, name in enumerate(msg.name)
            if i < len(msg.velocity)
        }

        self.rear_left_velocity = velocities.get(
            'rear_left_wheel_joint',
            self.rear_left_velocity
        )
        self.rear_right_velocity = velocities.get(
            'rear_right_wheel_joint',
            self.rear_right_velocity
        )
        self.front_left_steering = positions.get(
            'front_left_steering_joint',
            self.front_left_steering
        )
        self.front_right_steering = positions.get(
            'front_right_steering_joint',
            self.front_right_steering
        )

        self.wheel_derived_speed = (
            self.WHEEL_RADIUS
            * 0.5
            * (self.rear_left_velocity + self.rear_right_velocity)
        )

        self.measured_delta = self.equivalent_delta_from_front_joints(
            self.front_left_steering,
            self.front_right_steering
        )

    def reset_models_to_odom(self):
        self.command_model_x = self.actual_x
        self.command_model_y = self.actual_y
        self.command_model_theta = self.actual_theta

        self.measured_model_x = self.actual_x
        self.measured_model_y = self.actual_y
        self.measured_model_theta = self.actual_theta

        rear_x, rear_y = self.reference_to_rear(
            self.actual_x,
            self.actual_y,
            self.actual_theta
        )
        self.command_rear_x = rear_x
        self.command_rear_y = rear_y
        self.measured_rear_x = rear_x
        self.measured_rear_y = rear_y

        now = self.get_clock().now()
        self.start_time = now
        self.last_model_time = now
        self.publish_model_states()

    def publish_commands(self):
        left_steer, right_steer = self.calculate_ackermann_angles(
            self.command_delta
        )
        left_wheel, right_wheel = self.calculate_rear_wheel_velocities(
            self.command_speed,
            self.command_delta
        )

        steering_msg = Float64MultiArray()
        steering_msg.data = [left_steer, right_steer]
        self.steering_pub.publish(steering_msg)

        rear_msg = Float64MultiArray()
        rear_msg.data = [left_wheel, right_wheel]
        self.rear_wheel_pub.publish(rear_msg)

    def publish_model_states(self):
        command_msg = Pose2D()
        command_msg.x = float(self.command_model_x)
        command_msg.y = float(self.command_model_y)
        command_msg.theta = float(self.command_model_theta)
        self.command_model_pub.publish(command_msg)

        measured_msg = Pose2D()
        measured_msg.x = float(self.measured_model_x)
        measured_msg.y = float(self.measured_model_y)
        measured_msg.theta = float(self.measured_model_theta)
        self.measured_model_pub.publish(measured_msg)

    def publish_telemetry(self, elapsed):
        msg = Float64MultiArray()
        msg.data = [
            float(elapsed),
            float(self.command_speed),
            float(self.command_speed),
            float(self.wheel_derived_speed),
            float(self.odom_speed),
            float(self.command_delta),
            float(self.measured_delta),
        ]
        self.telemetry_pub.publish(msg)

    def control_loop(self):
        if self.quit_requested:
            return

        if not self.have_odom:
            return

        if self.reset_requested:
            self.reset_requested = False
            self.reset_models_to_odom()
            self.get_logger().info('Recording/model origin reset.')

        now = self.get_clock().now()

        if self.last_model_time is None:
            self.last_model_time = now
            return

        dt = (now - self.last_model_time).nanoseconds * 1.0e-9
        self.last_model_time = now

        if dt <= 0.0 or dt > 0.2:
            return

        # Command-input bicycle model.
        (
            self.command_rear_x,
            self.command_rear_y,
            self.command_model_theta
        ) = self.integrate_bicycle_model(
            self.command_rear_x,
            self.command_rear_y,
            self.command_model_theta,
            self.command_speed,
            self.command_delta,
            dt
        )
        (
            self.command_model_x,
            self.command_model_y
        ) = self.rear_to_reference(
            self.command_rear_x,
            self.command_rear_y,
            self.command_model_theta
        )

        # Measured-input bicycle model.
        (
            self.measured_rear_x,
            self.measured_rear_y,
            self.measured_model_theta
        ) = self.integrate_bicycle_model(
            self.measured_rear_x,
            self.measured_rear_y,
            self.measured_model_theta,
            self.wheel_derived_speed,
            self.measured_delta,
            dt
        )
        (
            self.measured_model_x,
            self.measured_model_y
        ) = self.rear_to_reference(
            self.measured_rear_x,
            self.measured_rear_y,
            self.measured_model_theta
        )

        self.publish_commands()
        self.publish_model_states()

        elapsed = (now - self.start_time).nanoseconds * 1.0e-9
        self.publish_telemetry(elapsed)

    def stop_vehicle(self):
        self.command_speed = 0.0
        self.command_delta = 0.0
        self.publish_commands()

    def print_status(self):
        self.get_logger().info(
            f'command: v={self.command_speed:+.2f} m/s, '
            f'delta={math.degrees(self.command_delta):+.1f} deg'
        )

    def handle_key(self, key):
        key = key.lower()

        if key == 'w':
            self.command_speed = min(
                self.MAX_SPEED,
                self.command_speed + self.SPEED_STEP
            )
            self.print_status()

        elif key == 's':
            self.command_speed = max(
                -self.MAX_SPEED,
                self.command_speed - self.SPEED_STEP
            )
            self.print_status()

        elif key == 'a':
            self.command_delta = min(
                self.MAX_STEERING,
                self.command_delta + self.STEERING_STEP
            )
            self.print_status()

        elif key == 'd':
            self.command_delta = max(
                -self.MAX_STEERING,
                self.command_delta - self.STEERING_STEP
            )
            self.print_status()

        elif key == ' ':
            self.command_speed = 0.0
            self.print_status()

        elif key == 'c':
            self.command_delta = 0.0
            self.print_status()

        elif key == 'x':
            self.command_speed = 0.0
            self.command_delta = 0.0
            self.print_status()

        elif key == 'r':
            self.reset_requested = True

        elif key == 'q':
            self.stop_vehicle()
            self.quit_requested = True

    def keyboard_loop(self):
        if not sys.stdin.isatty():
            self.get_logger().error(
                'ackermann_teleop requires an interactive terminal.'
            )
            self.quit_requested = True
            return

        old_settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setcbreak(sys.stdin.fileno())

            while rclpy.ok() and not self.quit_requested:
                ready, _, _ = select.select(
                    [sys.stdin],
                    [],
                    [],
                    0.1
                )
                if ready:
                    key = sys.stdin.read(1)
                    self.handle_key(key)

        finally:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                old_settings
            )

    def print_help(self):
        print()
        print('==============================================')
        print('EE5112 Ackermann Keyboard Teleoperation')
        print('==============================================')
        print('W / S : increase / decrease speed')
        print('A / D : steer left / right')
        print('SPACE : stop longitudinal motion')
        print('C     : centre steering')
        print('X     : stop + centre steering')
        print('R     : reset live plot/model origin')
        print('Q     : stop and quit')
        print()
        print('Limits:')
        print(f'  speed    +/-{self.MAX_SPEED:.2f} m/s')
        print(f'  steering +/-{math.degrees(self.MAX_STEERING):.1f} deg')
        print('==============================================')
        print()


def main(args=None):
    rclpy.init(args=args)
    node = AckermannTeleop()

    try:
        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=0.05)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            node.stop_vehicle()
            # Send the stop more than once before destruction.
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.02)
                node.publish_commands()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
