#!/usr/bin/env python3

"""
EE5112 MiniLab 1.2
Task 2 - Ackermann Trajectory Validation

This node compares:

1. Command-input bicycle model
   - Uses commanded vehicle speed v
   - Uses commanded equivalent steering angle delta

2. Measured-input bicycle model
   - Uses actual translational speed measured from Gazebo /odom
   - Uses commanded equivalent steering angle delta

3. Gazebo ground truth
   - Uses x, y and yaw directly from /odom

The comparison separates actuator / physics effects from
kinematic-model effects.

Available experiments:
    straight
    gentle_left
    sharp_left
    right_turn
"""

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class TrajectoryExperiment(Node):

    # ==========================================================
    # VEHICLE PARAMETERS
    # ==========================================================

    WHEELBASE = 0.20
    TRACK_WIDTH = 0.16
    WHEEL_RADIUS = 0.04

    MAX_STEERING = math.radians(35.0)
    MAX_SPEED = 0.50


    # ==========================================================
    # EXPERIMENT DEFINITIONS
    # ==========================================================

    EXPERIMENTS = {

        'straight': {
            'speed': 0.12,
            'steering_deg': 0.0,
            'duration': 4.0,
        },

        'gentle_left': {
            'speed': 0.12,
            'steering_deg': 15.0,
            'duration': 5.0,
        },

        'sharp_left': {
            'speed': 0.12,
            'steering_deg': 30.0,
            'duration': 4.0,
        },

        'right_turn': {
            'speed': 0.12,
            'steering_deg': -20.0,
            'duration': 5.0,
        },
    }


    # ==========================================================
    # INITIALIZATION
    # ==========================================================

    def __init__(self):

        super().__init__('trajectory_experiment')

        # ------------------------------------------------------
        # Experiment parameter
        # ------------------------------------------------------

        self.declare_parameter(
            'experiment',
            'straight'
        )

        self.experiment_name = (
            self.get_parameter('experiment')
            .get_parameter_value()
            .string_value
        )

        if self.experiment_name not in self.EXPERIMENTS:

            valid = ', '.join(
                self.EXPERIMENTS.keys()
            )

            raise ValueError(
                f"Unknown experiment "
                f"'{self.experiment_name}'. "
                f"Valid experiments: {valid}"
            )

        config = self.EXPERIMENTS[
            self.experiment_name
        ]

        self.command_speed = float(
            config['speed']
        )

        self.command_delta = math.radians(
            float(
                config['steering_deg']
            )
        )

        self.duration = float(
            config['duration']
        )


        # ------------------------------------------------------
        # Safety checks
        # ------------------------------------------------------

        if abs(self.command_speed) > self.MAX_SPEED:

            raise ValueError(
                'Experiment speed exceeds '
                'the vehicle speed limit.'
            )

        if abs(self.command_delta) > self.MAX_STEERING:

            raise ValueError(
                'Experiment steering exceeds '
                'the vehicle steering limit.'
            )


        # ------------------------------------------------------
        # Ackermann wheel steering angles
        # ------------------------------------------------------

        (
            self.left_steering,
            self.right_steering
        ) = self.calculate_ackermann_angles(
            self.command_delta
        )


        # ------------------------------------------------------
        # Rear-wheel angular velocity command
        # ------------------------------------------------------

        self.command_wheel_velocity = (
            self.command_speed
            / self.WHEEL_RADIUS
        )


        # ------------------------------------------------------
        # Publishers
        # ------------------------------------------------------

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


        # ------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            50
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            50
        )


        # ------------------------------------------------------
        # Experiment state
        # ------------------------------------------------------

        self.have_odom = False
        self.experiment_started = False
        self.experiment_finished = False
        self.shutdown_requested = False

        self.start_time = None
        self.last_model_time = None


        # ------------------------------------------------------
        # Actual Gazebo state
        # ------------------------------------------------------

        self.actual_x = 0.0
        self.actual_y = 0.0
        self.actual_theta = 0.0

        self.odom_speed = 0.0


        # ------------------------------------------------------
        # Command-input bicycle model
        # ------------------------------------------------------

        self.command_model_x = 0.0
        self.command_model_y = 0.0
        self.command_model_theta = 0.0


        # ------------------------------------------------------
        # Measured-input bicycle model
        # ------------------------------------------------------

        self.measured_model_x = 0.0
        self.measured_model_y = 0.0
        self.measured_model_theta = 0.0


        # ------------------------------------------------------
        # Measured wheel quantities
        # ------------------------------------------------------

        self.rear_left_velocity = 0.0
        self.rear_right_velocity = 0.0

        self.wheel_derived_speed = 0.0


        # ------------------------------------------------------
        # Data storage
        # ------------------------------------------------------

        self.data = []


        # ------------------------------------------------------
        # Main timer
        # ------------------------------------------------------

        self.control_timer = self.create_timer(
            0.02,
            self.control_loop
        )


        # ------------------------------------------------------
        # Print configuration
        # ------------------------------------------------------

        self.print_configuration()


    # ==========================================================
    # CONFIGURATION INFORMATION
    # ==========================================================

    def print_configuration(self):

        self.get_logger().info(
            '=============================================='
        )

        self.get_logger().info(
            'EE5112 Task 2 Ackermann Trajectory Validation'
        )

        self.get_logger().info(
            '=============================================='
        )

        self.get_logger().info(
            f'Experiment       : {self.experiment_name}'
        )

        self.get_logger().info(
            f'Command speed    : '
            f'{self.command_speed:.3f} m/s'
        )

        self.get_logger().info(
            f'Bicycle steering : '
            f'{math.degrees(self.command_delta):.2f} deg'
        )

        self.get_logger().info(
            f'Left steering    : '
            f'{math.degrees(self.left_steering):.2f} deg'
        )

        self.get_logger().info(
            f'Right steering   : '
            f'{math.degrees(self.right_steering):.2f} deg'
        )

        self.get_logger().info(
            f'Rear wheel cmd   : '
            f'{self.command_wheel_velocity:.3f} rad/s'
        )

        self.get_logger().info(
            f'Duration         : '
            f'{self.duration:.2f} s'
        )

        if abs(self.command_delta) > 1.0e-9:

            radius = (
                self.WHEELBASE
                / math.tan(
                    abs(self.command_delta)
                )
            )

            self.get_logger().info(
                f'Theoretical R    : '
                f'{radius:.3f} m'
            )

        else:

            self.get_logger().info(
                'Theoretical R    : infinity'
            )

        self.get_logger().info(
            'Waiting for /odom...'
        )


    # ==========================================================
    # ACKERMANN STEERING GEOMETRY
    # ==========================================================

    def calculate_ackermann_angles(
        self,
        delta
    ):

        """
        Convert equivalent bicycle steering angle into
        individual left/right front-wheel steering angles.

        Positive delta:
            left turn

        Negative delta:
            right turn
        """

        if abs(delta) < 1.0e-9:

            return 0.0, 0.0


        radius = (
            self.WHEELBASE
            / math.tan(
                abs(delta)
            )
        )

        half_track = (
            self.TRACK_WIDTH / 2.0
        )


        # ------------------------------------------------------
        # Ideal Ackermann angles
        # ------------------------------------------------------

        inner_angle = math.atan(
            self.WHEELBASE
            / (
                radius
                - half_track
            )
        )

        outer_angle = math.atan(
            self.WHEELBASE
            / (
                radius
                + half_track
            )
        )


        if delta > 0.0:

            # Left turn
            #
            # Left wheel is inner wheel.

            left = inner_angle
            right = outer_angle

        else:

            # Right turn
            #
            # Right wheel is inner wheel.

            left = -outer_angle
            right = -inner_angle


        # ------------------------------------------------------
        # Apply physical steering limit
        # ------------------------------------------------------

        left = max(
            -self.MAX_STEERING,
            min(
                self.MAX_STEERING,
                left
            )
        )

        right = max(
            -self.MAX_STEERING,
            min(
                self.MAX_STEERING,
                right
            )
        )

        return left, right


    # ==========================================================
    # QUATERNION -> YAW
    # ==========================================================

    @staticmethod
    def quaternion_to_yaw(q):

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )


    # ==========================================================
    # NORMALIZE ANGLE
    # ==========================================================

    @staticmethod
    def normalize_angle(angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )


    # ==========================================================
    # ODOMETRY CALLBACK
    # ==========================================================

    def odom_callback(
        self,
        msg
    ):

        # ------------------------------------------------------
        # Actual position
        # ------------------------------------------------------

        self.actual_x = (
            msg.pose.pose.position.x
        )

        self.actual_y = (
            msg.pose.pose.position.y
        )

        self.actual_theta = (
            self.quaternion_to_yaw(
                msg.pose.pose.orientation
            )
        )


        # ------------------------------------------------------
        # Actual translational speed
        # ------------------------------------------------------

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        self.odom_speed = math.sqrt(
            vx * vx
            + vy * vy
        )


        # ------------------------------------------------------
        # Initialize both models from exact Gazebo pose
        # ------------------------------------------------------

        if not self.have_odom:

            self.have_odom = True


            # Command-input model

            self.command_model_x = (
                self.actual_x
            )

            self.command_model_y = (
                self.actual_y
            )

            self.command_model_theta = (
                self.actual_theta
            )


            # Measured-input model

            self.measured_model_x = (
                self.actual_x
            )

            self.measured_model_y = (
                self.actual_y
            )

            self.measured_model_theta = (
                self.actual_theta
            )


            self.get_logger().info(
                'Ground-truth odometry received.'
            )

            self.get_logger().info(
                'Initial pose: '
                f'x={self.actual_x:.4f}, '
                f'y={self.actual_y:.4f}, '
                f'theta='
                f'{math.degrees(self.actual_theta):.4f} deg'
            )


    # ==========================================================
    # JOINT STATE CALLBACK
    # ==========================================================

    def joint_state_callback(
        self,
        msg
    ):

        joint_velocities = {}

        for index, name in enumerate(
            msg.name
        ):

            if index < len(msg.velocity):

                joint_velocities[name] = (
                    msg.velocity[index]
                )


        self.rear_left_velocity = (
            joint_velocities.get(
                'rear_left_wheel_joint',
                self.rear_left_velocity
            )
        )

        self.rear_right_velocity = (
            joint_velocities.get(
                'rear_right_wheel_joint',
                self.rear_right_velocity
            )
        )


        average_wheel_velocity = (
            0.5
            * (
                self.rear_left_velocity
                + self.rear_right_velocity
            )
        )


        self.wheel_derived_speed = (
            self.WHEEL_RADIUS
            * average_wheel_velocity
        )


    # ==========================================================
    # PUBLISH VEHICLE COMMANDS
    # ==========================================================

    def publish_commands(self):

        # ------------------------------------------------------
        # Steering
        # ------------------------------------------------------

        steering_msg = Float64MultiArray()

        steering_msg.data = [
            self.left_steering,
            self.right_steering
        ]

        self.steering_pub.publish(
            steering_msg
        )


        # ------------------------------------------------------
        # Rear wheels
        # ------------------------------------------------------

        rear_msg = Float64MultiArray()

        rear_msg.data = [
            self.command_wheel_velocity,
            self.command_wheel_velocity
        ]

        self.rear_wheel_pub.publish(
            rear_msg
        )


    # ==========================================================
    # STOP VEHICLE
    # ==========================================================

    def stop_vehicle(self):

        rear_msg = Float64MultiArray()

        rear_msg.data = [
            0.0,
            0.0
        ]

        self.rear_wheel_pub.publish(
            rear_msg
        )


        steering_msg = Float64MultiArray()

        steering_msg.data = [
            0.0,
            0.0
        ]

        self.steering_pub.publish(
            steering_msg
        )


    # ==========================================================
    # BICYCLE MODEL INTEGRATOR
    # ==========================================================

    def integrate_bicycle_model(
        self,
        x,
        y,
        theta,
        speed,
        delta,
        dt
    ):

        x_new = (
            x
            + speed
            * math.cos(theta)
            * dt
        )

        y_new = (
            y
            + speed
            * math.sin(theta)
            * dt
        )

        yaw_rate = (
            speed
            / self.WHEELBASE
            * math.tan(delta)
        )

        theta_new = (
            theta
            + yaw_rate * dt
        )

        theta_new = (
            self.normalize_angle(
                theta_new
            )
        )

        return (
            x_new,
            y_new,
            theta_new
        )


    # ==========================================================
    # MAIN CONTROL LOOP
    # ==========================================================

    def control_loop(self):

        if self.experiment_finished:

            return


        # ------------------------------------------------------
        # Wait for odometry
        # ------------------------------------------------------

        if not self.have_odom:

            return


        now = self.get_clock().now()


        # ------------------------------------------------------
        # Start experiment
        # ------------------------------------------------------

        if not self.experiment_started:

            self.experiment_started = True

            self.start_time = now
            self.last_model_time = now

            self.get_logger().info(
                '=============================================='
            )

            self.get_logger().info(
                'EXPERIMENT STARTED'
            )

            self.get_logger().info(
                '=============================================='
            )

            # Start commanding immediately.

            self.publish_commands()

            return


        # ------------------------------------------------------
        # Elapsed time
        # ------------------------------------------------------

        elapsed = (
            now - self.start_time
        ).nanoseconds * 1.0e-9


        # ------------------------------------------------------
        # AUTOMATIC END OF EXPERIMENT
        # ------------------------------------------------------

        if elapsed >= self.duration:

            self.finish_experiment()

            return


        # ------------------------------------------------------
        # Time step
        # ------------------------------------------------------

        dt = (
            now - self.last_model_time
        ).nanoseconds * 1.0e-9

        self.last_model_time = now


        if dt <= 0.0 or dt > 0.2:

            return


        # ------------------------------------------------------
        # 1. COMMAND-INPUT BICYCLE MODEL
        #
        # Assumes the commanded vehicle speed is achieved.
        # ------------------------------------------------------

        (
            self.command_model_x,
            self.command_model_y,
            self.command_model_theta
        ) = self.integrate_bicycle_model(

            self.command_model_x,
            self.command_model_y,
            self.command_model_theta,

            self.command_speed,
            self.command_delta,

            dt
        )


        # ------------------------------------------------------
        # 2. MEASURED-INPUT BICYCLE MODEL
        #
        # Uses actual Gazebo translational speed from /odom.
        #
        # This isolates kinematic-model error from the
        # drivetrain / actuator speed-response error.
        # ------------------------------------------------------

        (
            self.measured_model_x,
            self.measured_model_y,
            self.measured_model_theta
        ) = self.integrate_bicycle_model(

            self.measured_model_x,
            self.measured_model_y,
            self.measured_model_theta,

            self.odom_speed,
            self.command_delta,

            dt
        )


        # ------------------------------------------------------
        # Continue sending controller commands
        # ------------------------------------------------------

        self.publish_commands()


        # ------------------------------------------------------
        # Position errors
        # ------------------------------------------------------

        command_position_error = math.hypot(

            self.actual_x
            - self.command_model_x,

            self.actual_y
            - self.command_model_y
        )


        measured_position_error = math.hypot(

            self.actual_x
            - self.measured_model_x,

            self.actual_y
            - self.measured_model_y
        )


        # ------------------------------------------------------
        # Heading errors
        # ------------------------------------------------------

        command_heading_error = abs(

            self.normalize_angle(

                self.actual_theta
                - self.command_model_theta
            )
        )


        measured_heading_error = abs(

            self.normalize_angle(

                self.actual_theta
                - self.measured_model_theta
            )
        )


        # ------------------------------------------------------
        # Record data
        # ------------------------------------------------------

        self.data.append({

            'time':
                elapsed,

            'command_model_x':
                self.command_model_x,

            'command_model_y':
                self.command_model_y,

            'command_model_theta':
                self.command_model_theta,

            'measured_model_x':
                self.measured_model_x,

            'measured_model_y':
                self.measured_model_y,

            'measured_model_theta':
                self.measured_model_theta,

            'actual_x':
                self.actual_x,

            'actual_y':
                self.actual_y,

            'actual_theta':
                self.actual_theta,

            'command_v':
                self.command_speed,

            'odom_v':
                self.odom_speed,

            'wheel_derived_v':
                self.wheel_derived_speed,

            'command_delta':
                self.command_delta,

            'left_steering':
                self.left_steering,

            'right_steering':
                self.right_steering,

            'rear_left_omega':
                self.rear_left_velocity,

            'rear_right_omega':
                self.rear_right_velocity,

            'command_position_error':
                command_position_error,

            'measured_position_error':
                measured_position_error,

            'command_heading_error':
                command_heading_error,

            'measured_heading_error':
                measured_heading_error,
        })


    # ==========================================================
    # RESULTS DIRECTORY
    # ==========================================================

    def get_results_directories(self):

        root = (
            Path.home()
            / 'EE5112_MiniLab'
            / 'task2_results'
        )

        csv_dir = root / 'csv'
        figure_dir = root / 'figures'

        csv_dir.mkdir(
            parents=True,
            exist_ok=True
        )

        figure_dir.mkdir(
            parents=True,
            exist_ok=True
        )

        return (
            csv_dir,
            figure_dir
        )


    # ==========================================================
    # RMS
    # ==========================================================

    @staticmethod
    def rms(values):

        if not values:

            return 0.0

        return math.sqrt(

            sum(
                value * value
                for value in values
            )

            / len(values)
        )


    # ==========================================================
    # SAVE CSV
    # ==========================================================

    def save_csv(
        self,
        path
    ):

        fieldnames = list(
            self.data[0].keys()
        )

        with open(
            path,
            'w',
            newline='',
            encoding='utf-8'
        ) as csv_file:

            writer = csv.DictWriter(
                csv_file,
                fieldnames=fieldnames
            )

            writer.writeheader()

            writer.writerows(
                self.data
            )


    # ==========================================================
    # SAVE SUMMARY
    # ==========================================================

    def save_summary(
        self,
        path
    ):

        command_position_errors = [

            row['command_position_error']

            for row in self.data
        ]


        measured_position_errors = [

            row['measured_position_error']

            for row in self.data
        ]


        command_heading_errors = [

            row['command_heading_error']

            for row in self.data
        ]


        measured_heading_errors = [

            row['measured_heading_error']

            for row in self.data
        ]


        odom_speeds = [

            row['odom_v']

            for row in self.data
        ]


        wheel_speeds = [

            row['wheel_derived_v']

            for row in self.data
        ]


        mean_odom_speed = (
            sum(odom_speeds)
            / len(odom_speeds)
        )


        mean_wheel_speed = (
            sum(wheel_speeds)
            / len(wheel_speeds)
        )


        if abs(self.command_delta) > 1.0e-9:

            theoretical_radius = (
                self.WHEELBASE
                / math.tan(
                    abs(self.command_delta)
                )
            )

        else:

            theoretical_radius = math.inf


        with open(
            path,
            'w',
            encoding='utf-8'
        ) as file:

            file.write(
                'EE5112 Task 2 Ackermann '
                'Trajectory Validation\n'
            )

            file.write(
                '========================================\n\n'
            )

            file.write(
                f'Experiment: '
                f'{self.experiment_name}\n'
            )

            file.write(
                f'Duration: '
                f'{self.duration:.3f} s\n'
            )

            file.write(
                f'Commanded v: '
                f'{self.command_speed:.6f} m/s\n'
            )

            file.write(
                f'Mean /odom speed: '
                f'{mean_odom_speed:.6f} m/s\n'
            )

            file.write(
                f'Mean wheel-derived speed: '
                f'{mean_wheel_speed:.6f} m/s\n'
            )

            file.write(
                f'Equivalent steering: '
                f'{math.degrees(self.command_delta):.6f} deg\n'
            )

            file.write(
                f'Left steering: '
                f'{math.degrees(self.left_steering):.6f} deg\n'
            )

            file.write(
                f'Right steering: '
                f'{math.degrees(self.right_steering):.6f} deg\n'
            )


            if math.isfinite(
                theoretical_radius
            ):

                file.write(
                    f'Theoretical bicycle radius: '
                    f'{theoretical_radius:.6f} m\n'
                )

            else:

                file.write(
                    'Theoretical bicycle radius: '
                    'infinity\n'
                )


            file.write('\n')


            # --------------------------------------------------
            # Command-input model
            # --------------------------------------------------

            file.write(
                'COMMAND-INPUT BICYCLE MODEL\n'
            )

            file.write(
                '---------------------------\n'
            )

            file.write(
                f'RMS position error: '
                f'{self.rms(command_position_errors):.6f} m\n'
            )

            file.write(
                f'Maximum position error: '
                f'{max(command_position_errors):.6f} m\n'
            )

            file.write(
                f'Final position error: '
                f'{command_position_errors[-1]:.6f} m\n'
            )

            file.write(
                f'Final heading error: '
                f'{math.degrees(command_heading_errors[-1]):.6f} deg\n'
            )


            file.write('\n')


            # --------------------------------------------------
            # Measured-input model
            # --------------------------------------------------

            file.write(
                'MEASURED-INPUT BICYCLE MODEL\n'
            )

            file.write(
                '----------------------------\n'
            )

            file.write(
                f'RMS position error: '
                f'{self.rms(measured_position_errors):.6f} m\n'
            )

            file.write(
                f'Maximum position error: '
                f'{max(measured_position_errors):.6f} m\n'
            )

            file.write(
                f'Final position error: '
                f'{measured_position_errors[-1]:.6f} m\n'
            )

            file.write(
                f'Final heading error: '
                f'{math.degrees(measured_heading_errors[-1]):.6f} deg\n'
            )


    # ==========================================================
    # TRAJECTORY PLOT
    # ==========================================================

    def plot_trajectory(
        self,
        path
    ):

        command_x = [
            row['command_model_x']
            for row in self.data
        ]

        command_y = [
            row['command_model_y']
            for row in self.data
        ]

        measured_x = [
            row['measured_model_x']
            for row in self.data
        ]

        measured_y = [
            row['measured_model_y']
            for row in self.data
        ]

        actual_x = [
            row['actual_x']
            for row in self.data
        ]

        actual_y = [
            row['actual_y']
            for row in self.data
        ]


        plt.figure(
            figsize=(8, 6)
        )


        plt.plot(
            command_x,
            command_y,
            linewidth=2.0,
            label='Command-input bicycle model'
        )


        plt.plot(
            measured_x,
            measured_y,
            linewidth=2.0,
            label='Measured-input bicycle model'
        )


        plt.plot(
            actual_x,
            actual_y,
            '--',
            linewidth=2.0,
            label='Gazebo ground truth'
        )


        plt.scatter(
            [actual_x[0]],
            [actual_y[0]],
            s=60,
            label='Start'
        )


        plt.xlabel(
            'x [m]'
        )

        plt.ylabel(
            'y [m]'
        )


        plt.title(
            'Task 2 Ackermann Trajectory Comparison\n'
            + self.experiment_name
        )


        plt.axis(
            'equal'
        )

        plt.grid(
            True
        )

        plt.legend()

        plt.tight_layout()


        plt.savefig(
            path,
            dpi=200
        )

        plt.close()


    # ==========================================================
    # VELOCITY PLOT
    # ==========================================================

    def plot_velocity(
        self,
        path
    ):

        time = [
            row['time']
            for row in self.data
        ]

        command_v = [
            row['command_v']
            for row in self.data
        ]

        odom_v = [
            row['odom_v']
            for row in self.data
        ]

        wheel_v = [
            row['wheel_derived_v']
            for row in self.data
        ]


        plt.figure(
            figsize=(8, 5)
        )


        plt.plot(
            time,
            command_v,
            linewidth=2.0,
            label='Commanded v'
        )


        plt.plot(
            time,
            odom_v,
            linewidth=2.0,
            label='Gazebo /odom speed'
        )


        plt.plot(
            time,
            wheel_v,
            '--',
            linewidth=2.0,
            label='Wheel-derived speed'
        )


        plt.xlabel(
            'Time [s]'
        )

        plt.ylabel(
            'Speed [m/s]'
        )


        plt.title(
            'Commanded and Measured Vehicle Speed\n'
            + self.experiment_name
        )


        plt.grid(
            True
        )

        plt.legend()

        plt.tight_layout()


        plt.savefig(
            path,
            dpi=200
        )

        plt.close()


    # ==========================================================
    # HEADING PLOT
    # ==========================================================

    def plot_heading(
        self,
        path
    ):

        time = [
            row['time']
            for row in self.data
        ]


        command_theta = [

            math.degrees(
                row['command_model_theta']
            )

            for row in self.data
        ]


        measured_theta = [

            math.degrees(
                row['measured_model_theta']
            )

            for row in self.data
        ]


        actual_theta = [

            math.degrees(
                row['actual_theta']
            )

            for row in self.data
        ]


        plt.figure(
            figsize=(8, 5)
        )


        plt.plot(
            time,
            command_theta,
            linewidth=2.0,
            label='Command-input model'
        )


        plt.plot(
            time,
            measured_theta,
            linewidth=2.0,
            label='Measured-input model'
        )


        plt.plot(
            time,
            actual_theta,
            '--',
            linewidth=2.0,
            label='Gazebo ground truth'
        )


        plt.xlabel(
            'Time [s]'
        )

        plt.ylabel(
            'Heading [deg]'
        )


        plt.title(
            'Heading Comparison\n'
            + self.experiment_name
        )


        plt.grid(
            True
        )

        plt.legend()

        plt.tight_layout()


        plt.savefig(
            path,
            dpi=200
        )

        plt.close()


    # ==========================================================
    # POSITION ERROR PLOT
    # ==========================================================

    def plot_error(
        self,
        path
    ):

        time = [
            row['time']
            for row in self.data
        ]


        command_error = [

            row['command_position_error']

            for row in self.data
        ]


        measured_error = [

            row['measured_position_error']

            for row in self.data
        ]


        plt.figure(
            figsize=(8, 5)
        )


        plt.plot(
            time,
            command_error,
            linewidth=2.0,
            label='Command-input model error'
        )


        plt.plot(
            time,
            measured_error,
            linewidth=2.0,
            label='Measured-input model error'
        )


        plt.xlabel(
            'Time [s]'
        )

        plt.ylabel(
            'Position error [m]'
        )


        plt.title(
            'Trajectory Position Error\n'
            + self.experiment_name
        )


        plt.grid(
            True
        )

        plt.legend()

        plt.tight_layout()


        plt.savefig(
            path,
            dpi=200
        )

        plt.close()


    # ==========================================================
    # STEERING PLOT
    # ==========================================================

    def plot_steering(
        self,
        path
    ):

        time = [
            row['time']
            for row in self.data
        ]


        equivalent = [

            math.degrees(
                row['command_delta']
            )

            for row in self.data
        ]


        left = [

            math.degrees(
                row['left_steering']
            )

            for row in self.data
        ]


        right = [

            math.degrees(
                row['right_steering']
            )

            for row in self.data
        ]


        plt.figure(
            figsize=(8, 5)
        )


        plt.plot(
            time,
            equivalent,
            linewidth=2.0,
            label='Equivalent bicycle steering'
        )


        plt.plot(
            time,
            left,
            '--',
            linewidth=2.0,
            label='Left wheel steering'
        )


        plt.plot(
            time,
            right,
            '--',
            linewidth=2.0,
            label='Right wheel steering'
        )


        plt.xlabel(
            'Time [s]'
        )

        plt.ylabel(
            'Steering angle [deg]'
        )


        # Display the physical steering range.

        plt.ylim(
            -35.0,
            35.0
        )


        plt.title(
            'Ackermann Steering Commands\n'
            + self.experiment_name
        )


        plt.grid(
            True
        )

        plt.legend()

        plt.tight_layout()


        plt.savefig(
            path,
            dpi=200
        )

        plt.close()


    # ==========================================================
    # LATERAL DISPLACEMENT PLOT
    # ==========================================================

    def plot_lateral_displacement(
        self,
        path
    ):

        time = [
            row['time']
            for row in self.data
        ]


        initial_y = (
            self.data[0]['actual_y']
        )


        command_dy = [

            row['command_model_y']
            - initial_y

            for row in self.data
        ]


        measured_dy = [

            row['measured_model_y']
            - initial_y

            for row in self.data
        ]


        actual_dy = [

            row['actual_y']
            - initial_y

            for row in self.data
        ]


        plt.figure(
            figsize=(8, 5)
        )


        plt.plot(
            time,
            command_dy,
            linewidth=2.0,
            label='Command-input model'
        )


        plt.plot(
            time,
            measured_dy,
            linewidth=2.0,
            label='Measured-input model'
        )


        plt.plot(
            time,
            actual_dy,
            '--',
            linewidth=2.0,
            label='Gazebo ground truth'
        )


        plt.xlabel(
            'Time [s]'
        )

        plt.ylabel(
            'Lateral displacement Δy [m]'
        )


        plt.title(
            'Lateral Displacement\n'
            + self.experiment_name
        )


        plt.grid(
            True
        )

        plt.legend()

        plt.tight_layout()


        plt.savefig(
            path,
            dpi=200
        )

        plt.close()


    # ==========================================================
    # SAVE ALL RESULTS
    # ==========================================================

    def save_results(self):

        if not self.data:

            self.get_logger().error(
                'No trajectory data recorded.'
            )

            return


        (
            csv_dir,
            figure_dir
        ) = self.get_results_directories()


        name = self.experiment_name


        # ------------------------------------------------------
        # CSV
        # ------------------------------------------------------

        csv_path = (
            csv_dir
            / f'{name}.csv'
        )


        summary_path = (
            csv_dir
            / f'{name}_summary.txt'
        )


        # ------------------------------------------------------
        # Figures
        # ------------------------------------------------------

        trajectory_path = (
            figure_dir
            / f'{name}_trajectory.png'
        )

        velocity_path = (
            figure_dir
            / f'{name}_velocity.png'
        )

        heading_path = (
            figure_dir
            / f'{name}_heading.png'
        )

        error_path = (
            figure_dir
            / f'{name}_error.png'
        )

        steering_path = (
            figure_dir
            / f'{name}_steering.png'
        )

        lateral_path = (
            figure_dir
            / f'{name}_lateral.png'
        )


        # ------------------------------------------------------
        # Save
        # ------------------------------------------------------

        self.save_csv(
            csv_path
        )

        self.save_summary(
            summary_path
        )

        self.plot_trajectory(
            trajectory_path
        )

        self.plot_velocity(
            velocity_path
        )

        self.plot_heading(
            heading_path
        )

        self.plot_error(
            error_path
        )

        self.plot_steering(
            steering_path
        )

        self.plot_lateral_displacement(
            lateral_path
        )


        # ------------------------------------------------------
        # Terminal summary
        # ------------------------------------------------------

        command_errors = [

            row['command_position_error']

            for row in self.data
        ]


        measured_errors = [

            row['measured_position_error']

            for row in self.data
        ]


        self.get_logger().info(
            '=============================================='
        )

        self.get_logger().info(
            'RESULTS'
        )

        self.get_logger().info(
            '=============================================='
        )


        self.get_logger().info(
            f'Command-model RMS error: '
            f'{self.rms(command_errors):.4f} m'
        )


        self.get_logger().info(
            f'Measured-model RMS error: '
            f'{self.rms(measured_errors):.4f} m'
        )


        self.get_logger().info(
            f'CSV results: {csv_path}'
        )


        self.get_logger().info(
            f'Figures: {figure_dir}'
        )


    # ==========================================================
    # FINISH EXPERIMENT
    # ==========================================================

    def finish_experiment(self):

        if self.experiment_finished:

            return


        self.experiment_finished = True


        # ------------------------------------------------------
        # Stop vehicle
        # ------------------------------------------------------

        for _ in range(10):

            self.stop_vehicle()


        self.get_logger().info(
            '=============================================='
        )

        self.get_logger().info(
            'EXPERIMENT FINISHED'
        )

        self.get_logger().info(
            '=============================================='
        )


        # ------------------------------------------------------
        # Save results
        # ------------------------------------------------------

        try:

            self.save_results()

        except Exception as exc:

            self.get_logger().error(
                f'Failed to save results: {exc}'
            )


        # ------------------------------------------------------
        # Shut down after results are written
        # ------------------------------------------------------

        if not self.shutdown_requested:

            self.shutdown_requested = True

            self.create_timer(
                1.0,
                self.shutdown_node
            )


    # ==========================================================
    # SHUTDOWN
    # ==========================================================

    def shutdown_node(self):

        self.stop_vehicle()

        self.get_logger().info(
            'Trajectory experiment node shutting down.'
        )

        if rclpy.ok():

            rclpy.shutdown()


# ==============================================================
# MAIN
# ==============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = TrajectoryExperiment()


    try:

        rclpy.spin(
            node
        )


    except KeyboardInterrupt:

        node.get_logger().warning(
            'Experiment interrupted by user.'
        )

        node.stop_vehicle()


    finally:

        try:

            node.stop_vehicle()

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
