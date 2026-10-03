#!/usr/bin/env python3

"""EE5112 MiniLab 1.2 - Task 2 combined Ackermann trajectory experiment.

Motion sequence (34 s total):
  0-10 s   straight
 10-12 s   right turn
 12-22 s   straight
 22-24 s   left turn
 24-34 s   straight

The node records and publishes x, y, theta, v and delta for live plotting.
The bicycle models are integrated at the rear axle centre and shifted to the
base_footprint (/odom) reference point before comparison.
"""

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class TrajectoryExperiment(Node):
    WHEELBASE = 0.20
    TRACK_WIDTH = 0.16
    WHEEL_RADIUS = 0.04
    REF_OFFSET = WHEELBASE / 2.0
    MAX_STEERING = math.radians(35.0)
    MAX_SPEED = 0.50

    COMMAND_SPEED = 0.12
    TURN_ANGLE = math.radians(20.0)
    TOTAL_DURATION = 34.0

    # (start time, end time, speed [m/s], equivalent steering [rad], label)
    MOTION_SEQUENCE = (
        (0.0, 10.0, COMMAND_SPEED, 0.0, 'Straight 1'),
        (10.0, 12.0, COMMAND_SPEED, -TURN_ANGLE, 'Right turn'),
        (12.0, 22.0, COMMAND_SPEED, 0.0, 'Straight 2'),
        (22.0, 24.0, COMMAND_SPEED, TURN_ANGLE, 'Left turn'),
        (24.0, 34.0, COMMAND_SPEED, 0.0, 'Straight 3'),
    )

    def __init__(self):
        super().__init__('trajectory_experiment')

        self.declare_parameter('speed_lag_tau', 0.05)
        self.speed_lag_tau = float(
            self.get_parameter('speed_lag_tau').get_parameter_value().double_value
        )
        if self.speed_lag_tau < 0.0:
            raise ValueError('speed_lag_tau must be >= 0.')

        self.experiment_name = 'combined_motion'
        self.duration = self.TOTAL_DURATION

        # Current command, changed automatically by the phase scheduler.
        self.command_speed = self.COMMAND_SPEED
        self.command_delta = 0.0
        self.left_steering = 0.0
        self.right_steering = 0.0
        self.left_wheel_velocity_cmd = 0.0
        self.right_wheel_velocity_cmd = 0.0
        self.command_wheel_velocity = 0.0
        self.current_phase = ''
        self.current_phase_index = -1
        self.set_motion_command(self.COMMAND_SPEED, 0.0)

        # Controller publishers.
        self.steering_pub = self.create_publisher(
            Float64MultiArray, '/steering_controller/commands', 10
        )
        self.rear_wheel_pub = self.create_publisher(
            Float64MultiArray, '/rear_wheel_controller/commands', 10
        )

        # Live model poses.
        self.command_model_pub = self.create_publisher(
            Pose2D, '/task2/command_model_pose', 10
        )
        self.measured_model_pub = self.create_publisher(
            Pose2D, '/task2/measured_model_pose', 10
        )

        # Live scalar telemetry. Layout:
        # [time, command_v, command_model_v, measured_v, odom_v,
        #  command_delta, measured_delta]
        self.telemetry_pub = self.create_publisher(
            Float64MultiArray, '/task2/telemetry', 10
        )

        self.create_subscription(Odometry, '/odom', self.odom_callback, 50)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 50)

        self.have_odom = False
        self.experiment_started = False
        self.experiment_finished = False
        self.shutdown_requested = False
        self.start_time = None
        self.last_model_time = None

        # Gazebo ground truth.
        self.actual_x = 0.0
        self.actual_y = 0.0
        self.actual_theta = 0.0
        self.odom_speed = 0.0

        # Command-input model.
        self.command_model_x = 0.0
        self.command_model_y = 0.0
        self.command_model_theta = 0.0
        self.command_rear_x = 0.0
        self.command_rear_y = 0.0
        self.command_model_speed = 0.0

        # Measured-input model.
        self.measured_model_x = 0.0
        self.measured_model_y = 0.0
        self.measured_model_theta = 0.0
        self.measured_rear_x = 0.0
        self.measured_rear_y = 0.0

        # Joint measurements.
        self.rear_left_velocity = 0.0
        self.rear_right_velocity = 0.0
        self.front_left_steering = 0.0
        self.front_right_steering = 0.0
        self.wheel_derived_speed = 0.0
        self.measured_delta = 0.0

        self.data = []
        self.control_timer = self.create_timer(0.02, self.control_loop)
        self.print_configuration()

    def print_configuration(self):
        self.get_logger().info('==============================================')
        self.get_logger().info('EE5112 Task 2 Combined Trajectory Experiment')
        self.get_logger().info('==============================================')
        self.get_logger().info('Motion sequence:')
        for start, end, speed, delta, label in self.MOTION_SEQUENCE:
            self.get_logger().info(
                f'  {start:4.0f}-{end:4.0f} s  {label:<12} '
                f'v={speed:.3f} m/s  delta={math.degrees(delta):+.1f} deg'
            )
        self.get_logger().info(f'Total duration: {self.duration:.1f} s')
        self.get_logger().info('Live topics:')
        self.get_logger().info('  /task2/command_model_pose')
        self.get_logger().info('  /task2/measured_model_pose')
        self.get_logger().info('  /task2/telemetry')
        self.get_logger().info('Waiting for /odom...')

    def command_for_time(self, elapsed):
        for index, (start, end, speed, delta, label) in enumerate(self.MOTION_SEQUENCE):
            if start <= elapsed < end:
                return index, speed, delta, label
        return len(self.MOTION_SEQUENCE) - 1, 0.0, 0.0, 'Finished'

    def set_motion_command(self, speed, delta):
        self.command_speed = max(-self.MAX_SPEED, min(self.MAX_SPEED, float(speed)))
        self.command_delta = max(-self.MAX_STEERING, min(self.MAX_STEERING, float(delta)))
        self.left_steering, self.right_steering = self.calculate_ackermann_angles(
            self.command_delta
        )
        (
            self.left_wheel_velocity_cmd,
            self.right_wheel_velocity_cmd,
        ) = self.calculate_rear_wheel_velocities(
            self.command_speed, self.command_delta
        )
        self.command_wheel_velocity = self.command_speed / self.WHEEL_RADIUS

    def calculate_ackermann_angles(self, delta):
        if abs(delta) < 1.0e-9:
            return 0.0, 0.0
        radius = self.WHEELBASE / math.tan(abs(delta))
        half_track = self.TRACK_WIDTH / 2.0
        inner = math.atan(self.WHEELBASE / (radius - half_track))
        outer = math.atan(self.WHEELBASE / (radius + half_track))
        if delta > 0.0:
            left, right = inner, outer
        else:
            left, right = -outer, -inner
        return (
            max(-self.MAX_STEERING, min(self.MAX_STEERING, left)),
            max(-self.MAX_STEERING, min(self.MAX_STEERING, right)),
        )

    def calculate_rear_wheel_velocities(self, speed, delta):
        curvature = math.tan(delta) / self.WHEELBASE
        half_track = self.TRACK_WIDTH / 2.0
        left_speed = speed * (1.0 - half_track * curvature)
        right_speed = speed * (1.0 + half_track * curvature)
        return left_speed / self.WHEEL_RADIUS, right_speed / self.WHEEL_RADIUS

    def calculate_equivalent_steering(self, left_angle, right_angle):
        """Estimate bicycle steering delta from measured front-wheel angles."""
        estimates = []
        half_track = self.TRACK_WIDTH / 2.0
        if abs(left_angle) > 1.0e-5:
            r_left = self.WHEELBASE / math.tan(left_angle) + half_track
            estimates.append(r_left)
        if abs(right_angle) > 1.0e-5:
            r_right = self.WHEELBASE / math.tan(right_angle) - half_track
            estimates.append(r_right)
        if not estimates:
            return 0.0
        radius = sum(estimates) / len(estimates)
        if abs(radius) < 1.0e-6:
            return 0.0
        return math.atan(self.WHEELBASE / radius)

    def rear_to_reference(self, rear_x, rear_y, theta):
        return (
            rear_x + self.REF_OFFSET * math.cos(theta),
            rear_y + self.REF_OFFSET * math.sin(theta),
        )

    def reference_to_rear(self, ref_x, ref_y, theta):
        return (
            ref_x - self.REF_OFFSET * math.cos(theta),
            ref_y - self.REF_OFFSET * math.sin(theta),
        )

    def update_lagged_speed(self, current, target, dt):
        if self.speed_lag_tau <= 0.0:
            return target
        alpha = dt / (self.speed_lag_tau + dt)
        return current + alpha * (target - current)

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_callback(self, msg):
        self.actual_x = msg.pose.pose.position.x
        self.actual_y = msg.pose.pose.position.y
        self.actual_theta = self.quaternion_to_yaw(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.odom_speed = math.hypot(vx, vy)

        if not self.have_odom:
            self.have_odom = True
            self.command_model_x = self.actual_x
            self.command_model_y = self.actual_y
            self.command_model_theta = self.actual_theta
            self.measured_model_x = self.actual_x
            self.measured_model_y = self.actual_y
            self.measured_model_theta = self.actual_theta
            rear_x, rear_y = self.reference_to_rear(
                self.actual_x, self.actual_y, self.actual_theta
            )
            self.command_rear_x = rear_x
            self.command_rear_y = rear_y
            self.measured_rear_x = rear_x
            self.measured_rear_y = rear_y
            self.publish_model_states()
            self.get_logger().info(
                f'Initial pose: x={self.actual_x:.4f}, y={self.actual_y:.4f}, '
                f'theta={math.degrees(self.actual_theta):.3f} deg'
            )

    def joint_state_callback(self, msg):
        velocities = {}
        positions = {}
        for index, name in enumerate(msg.name):
            if index < len(msg.velocity):
                velocities[name] = msg.velocity[index]
            if index < len(msg.position):
                positions[name] = msg.position[index]

        self.rear_left_velocity = velocities.get(
            'rear_left_wheel_joint', self.rear_left_velocity
        )
        self.rear_right_velocity = velocities.get(
            'rear_right_wheel_joint', self.rear_right_velocity
        )
        self.front_left_steering = positions.get(
            'front_left_steering_joint', self.front_left_steering
        )
        self.front_right_steering = positions.get(
            'front_right_steering_joint', self.front_right_steering
        )
        self.wheel_derived_speed = self.WHEEL_RADIUS * 0.5 * (
            self.rear_left_velocity + self.rear_right_velocity
        )
        self.measured_delta = self.calculate_equivalent_steering(
            self.front_left_steering, self.front_right_steering
        )

    def publish_commands(self):
        steering_msg = Float64MultiArray()
        steering_msg.data = [self.left_steering, self.right_steering]
        self.steering_pub.publish(steering_msg)

        rear_msg = Float64MultiArray()
        rear_msg.data = [
            self.left_wheel_velocity_cmd,
            self.right_wheel_velocity_cmd,
        ]
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
            float(self.command_model_speed),
            float(self.wheel_derived_speed),
            float(self.odom_speed),
            float(self.command_delta),
            float(self.measured_delta),
        ]
        self.telemetry_pub.publish(msg)

    def stop_vehicle(self):
        rear_msg = Float64MultiArray()
        rear_msg.data = [0.0, 0.0]
        self.rear_wheel_pub.publish(rear_msg)
        steering_msg = Float64MultiArray()
        steering_msg.data = [0.0, 0.0]
        self.steering_pub.publish(steering_msg)

    def integrate_bicycle_model(self, x, y, theta, speed, delta, dt):
        x_new = x + speed * math.cos(theta) * dt
        y_new = y + speed * math.sin(theta) * dt
        theta_new = self.normalize_angle(
            theta + (speed / self.WHEELBASE) * math.tan(delta) * dt
        )
        return x_new, y_new, theta_new

    def control_loop(self):
        if self.experiment_finished or not self.have_odom:
            return

        now = self.get_clock().now()
        if not self.experiment_started:
            self.experiment_started = True
            self.start_time = now
            self.last_model_time = now
            self.current_phase_index = 0
            _, speed, delta, label = self.command_for_time(0.0)
            self.current_phase = label
            self.set_motion_command(speed, delta)
            self.get_logger().info(f'EXPERIMENT STARTED - {label}')
            self.publish_model_states()
            self.publish_telemetry(0.0)
            self.publish_commands()
            return

        elapsed = (now - self.start_time).nanoseconds * 1.0e-9
        if elapsed >= self.duration:
            self.finish_experiment()
            return

        dt = (now - self.last_model_time).nanoseconds * 1.0e-9
        self.last_model_time = now
        if dt <= 0.0 or dt > 0.2:
            return

        phase_index, speed, delta, label = self.command_for_time(elapsed)
        if phase_index != self.current_phase_index:
            self.current_phase_index = phase_index
            self.current_phase = label
            self.get_logger().info(
                f'Phase {phase_index + 1}/5: {label} '
                f'(v={speed:.3f} m/s, delta={math.degrees(delta):+.1f} deg)'
            )
        self.set_motion_command(speed, delta)

        # Command-input model: commanded speed with first-order drive lag.
        self.command_model_speed = self.update_lagged_speed(
            self.command_model_speed, self.command_speed, dt
        )
        (
            self.command_rear_x,
            self.command_rear_y,
            self.command_model_theta,
        ) = self.integrate_bicycle_model(
            self.command_rear_x,
            self.command_rear_y,
            self.command_model_theta,
            self.command_model_speed,
            self.command_delta,
            dt,
        )
        self.command_model_x, self.command_model_y = self.rear_to_reference(
            self.command_rear_x, self.command_rear_y, self.command_model_theta
        )

        # Measured-input model: wheel-derived speed + measured steering.
        measured_speed = self.wheel_derived_speed
        (
            self.measured_rear_x,
            self.measured_rear_y,
            self.measured_model_theta,
        ) = self.integrate_bicycle_model(
            self.measured_rear_x,
            self.measured_rear_y,
            self.measured_model_theta,
            measured_speed,
            self.measured_delta,
            dt,
        )
        self.measured_model_x, self.measured_model_y = self.rear_to_reference(
            self.measured_rear_x, self.measured_rear_y, self.measured_model_theta
        )

        self.publish_model_states()
        self.publish_telemetry(elapsed)
        self.publish_commands()

        command_position_error = math.hypot(
            self.actual_x - self.command_model_x,
            self.actual_y - self.command_model_y,
        )
        measured_position_error = math.hypot(
            self.actual_x - self.measured_model_x,
            self.actual_y - self.measured_model_y,
        )
        command_heading_error = abs(self.normalize_angle(
            self.actual_theta - self.command_model_theta
        ))
        measured_heading_error = abs(self.normalize_angle(
            self.actual_theta - self.measured_model_theta
        ))

        self.data.append({
            'time': elapsed,
            'phase': self.current_phase,
            'command_model_x': self.command_model_x,
            'command_model_y': self.command_model_y,
            'command_model_theta': self.command_model_theta,
            'measured_model_x': self.measured_model_x,
            'measured_model_y': self.measured_model_y,
            'measured_model_theta': self.measured_model_theta,
            'actual_x': self.actual_x,
            'actual_y': self.actual_y,
            'actual_theta': self.actual_theta,
            'command_v': self.command_speed,
            'command_model_v': self.command_model_speed,
            'measured_v': measured_speed,
            'odom_v': self.odom_speed,
            'command_delta': self.command_delta,
            'measured_delta': self.measured_delta,
            'left_steering': self.left_steering,
            'right_steering': self.right_steering,
            'measured_left_steering': self.front_left_steering,
            'measured_right_steering': self.front_right_steering,
            'rear_left_omega': self.rear_left_velocity,
            'rear_right_omega': self.rear_right_velocity,
            'command_position_error': command_position_error,
            'measured_position_error': measured_position_error,
            'command_heading_error': command_heading_error,
            'measured_heading_error': measured_heading_error,
        })

    def get_results_directories(self):
        root = Path.home() / 'EE5112_MiniLab1.2' / 'task2_results'
        csv_dir = root / 'csv'
        figure_dir = root / 'figures'
        csv_dir.mkdir(parents=True, exist_ok=True)
        figure_dir.mkdir(parents=True, exist_ok=True)
        return csv_dir, figure_dir

    @staticmethod
    def rms(values):
        if not values:
            return 0.0
        return math.sqrt(sum(v * v for v in values) / len(values))

    def save_csv(self, path):
        with open(path, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=list(self.data[0].keys()))
            writer.writeheader()
            writer.writerows(self.data)

    def save_summary(self, path):
        cmd_err = [r['command_position_error'] for r in self.data]
        meas_err = [r['measured_position_error'] for r in self.data]
        with open(path, 'w', encoding='utf-8') as f:
            f.write('EE5112 Task 2 Combined Ackermann Trajectory Validation\n')
            f.write('=====================================================\n\n')
            f.write('Motion: straight 10 s -> right 2 s -> straight 10 s -> '
                    'left 2 s -> straight 10 s\n')
            f.write(f'Command speed: {self.COMMAND_SPEED:.3f} m/s\n')
            f.write(f'Turn steering magnitude: {math.degrees(self.TURN_ANGLE):.1f} deg\n')
            f.write(f'Duration: {self.duration:.1f} s\n\n')
            f.write(f'Command-model RMS position error: {self.rms(cmd_err):.6f} m\n')
            f.write(f'Measured-model RMS position error: {self.rms(meas_err):.6f} m\n')
            f.write(f'Final command-model position error: {cmd_err[-1]:.6f} m\n')
            f.write(f'Final measured-model position error: {meas_err[-1]:.6f} m\n')

    def add_phase_lines(self):
        for t in (10.0, 12.0, 22.0, 24.0):
            plt.axvline(t, linestyle=':', linewidth=1.0)

    def plot_trajectory(self, path):
        plt.figure(figsize=(8, 6))
        plt.plot([r['command_model_x'] for r in self.data],
                 [r['command_model_y'] for r in self.data], label='Command-input model')
        plt.plot([r['measured_model_x'] for r in self.data],
                 [r['measured_model_y'] for r in self.data], label='Measured-input model')
        plt.plot([r['actual_x'] for r in self.data],
                 [r['actual_y'] for r in self.data], '--', label='Gazebo ground truth')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.title('Combined Motion - Trajectory')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(path, dpi=200)
        plt.close()

    def plot_heading(self, path):
        t = [r['time'] for r in self.data]
        plt.figure(figsize=(8, 5))
        plt.plot(t, [math.degrees(r['command_model_theta']) for r in self.data], label='Command model')
        plt.plot(t, [math.degrees(r['measured_model_theta']) for r in self.data], label='Measured model')
        plt.plot(t, [math.degrees(r['actual_theta']) for r in self.data], '--', label='Gazebo')
        self.add_phase_lines()
        plt.xlabel('Time [s]')
        plt.ylabel('theta [deg]')
        plt.title('Heading theta')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(path, dpi=200)
        plt.close()

    def plot_velocity(self, path):
        t = [r['time'] for r in self.data]
        plt.figure(figsize=(8, 5))
        plt.plot(t, [r['command_v'] for r in self.data], label='Commanded v')
        plt.plot(t, [r['measured_v'] for r in self.data], label='Wheel-derived v')
        plt.plot(t, [r['odom_v'] for r in self.data], '--', label='Gazebo /odom v')
        self.add_phase_lines()
        plt.xlabel('Time [s]')
        plt.ylabel('v [m/s]')
        plt.title('Velocity v')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(path, dpi=200)
        plt.close()

    def plot_steering(self, path):
        t = [r['time'] for r in self.data]
        plt.figure(figsize=(8, 5))
        plt.plot(t, [math.degrees(r['command_delta']) for r in self.data], label='Commanded delta')
        plt.plot(t, [math.degrees(r['measured_delta']) for r in self.data], '--', label='Measured delta')
        self.add_phase_lines()
        plt.xlabel('Time [s]')
        plt.ylabel('delta [deg]')
        plt.title('Equivalent steering angle delta')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(path, dpi=200)
        plt.close()

    def plot_error(self, path):
        t = [r['time'] for r in self.data]
        plt.figure(figsize=(8, 5))
        plt.plot(t, [r['command_position_error'] for r in self.data], label='Command model error')
        plt.plot(t, [r['measured_position_error'] for r in self.data], label='Measured model error')
        self.add_phase_lines()
        plt.xlabel('Time [s]')
        plt.ylabel('Position error [m]')
        plt.title('Trajectory position error')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(path, dpi=200)
        plt.close()

    def save_results(self):
        if not self.data:
            self.get_logger().error('No trajectory data recorded.')
            return
        csv_dir, fig_dir = self.get_results_directories()
        name = self.experiment_name
        self.save_csv(csv_dir / f'{name}.csv')
        self.save_summary(csv_dir / f'{name}_summary.txt')
        self.plot_trajectory(fig_dir / f'{name}_trajectory.png')
        self.plot_heading(fig_dir / f'{name}_heading.png')
        self.plot_velocity(fig_dir / f'{name}_velocity.png')
        self.plot_steering(fig_dir / f'{name}_steering.png')
        self.plot_error(fig_dir / f'{name}_error.png')
        self.get_logger().info(f'Results saved under {csv_dir.parent}')

    def finish_experiment(self):
        if self.experiment_finished:
            return
        self.experiment_finished = True
        self.set_motion_command(0.0, 0.0)
        self.command_model_speed = 0.0
        self.publish_model_states()
        self.publish_telemetry(self.duration)
        for _ in range(10):
            self.stop_vehicle()
        self.get_logger().info('EXPERIMENT FINISHED')
        try:
            self.save_results()
        except Exception as exc:
            self.get_logger().error(f'Failed to save results: {exc}')
        if not self.shutdown_requested:
            self.shutdown_requested = True
            self.create_timer(1.0, self.shutdown_node)

    def shutdown_node(self):
        self.stop_vehicle()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryExperiment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_vehicle()
    finally:
        try:
            node.stop_vehicle()
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
