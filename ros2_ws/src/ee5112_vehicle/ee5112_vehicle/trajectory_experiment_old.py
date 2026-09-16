#!/usr/bin/env python3

"""
EE5112 MiniLab 1.2 - Task 2 Trajectory Experiment

Automatically:
1. Receives ground-truth Gazebo pose from /odom.
2. Commands the Ackermann steering controller.
3. Commands the rear-wheel velocity controller.
4. Runs a selected open-loop manoeuvre.
5. Integrates the kinematic bicycle model in parallel.
6. Records planned and Gazebo trajectories.
7. Saves x, y, theta, v and delta to CSV.
8. Generates planned-vs-recorded trajectory plots.
9. Calculates trajectory errors.

Experiments:
    straight
    gentle_left
    sharp_left
    right_turn

Vehicle:
    wheelbase = 0.20 m
    track      = 0.16 m
    radius     = 0.04 m
    max steer  = 35 deg
    max speed  = 0.50 m/s
"""

import csv
import math
import os
from pathlib import Path

import matplotlib.pyplot as plt
import rclpy

from nav_msgs.msg import Odometry
from rclpy.node import Node
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
        "straight": {
            "speed": 0.12,
            "steering_deg": 0.0,
            "duration": 4.0,
        },

        "gentle_left": {
            "speed": 0.12,
            "steering_deg": 15.0,
            "duration": 5.0,
        },

        "sharp_left": {
            "speed": 0.12,
            "steering_deg": 30.0,
            "duration": 4.0,
        },

        "right_turn": {
            "speed": 0.12,
            "steering_deg": -20.0,
            "duration": 5.0,
        },
    }

    def __init__(self):

        super().__init__("trajectory_experiment")

        # ------------------------------------------------------
        # ROS parameter
        # ------------------------------------------------------

        self.declare_parameter("experiment", "straight")

        self.experiment_name = (
            self.get_parameter("experiment")
            .get_parameter_value()
            .string_value
        )

        if self.experiment_name not in self.EXPERIMENTS:

            self.get_logger().error(
                f"Unknown experiment: {self.experiment_name}"
            )

            self.get_logger().error(
                "Valid experiments: "
                + ", ".join(self.EXPERIMENTS.keys())
            )

            raise ValueError(
                f"Invalid experiment '{self.experiment_name}'"
            )

        config = self.EXPERIMENTS[self.experiment_name]

        self.command_speed = float(config["speed"])

        self.command_delta = math.radians(
            float(config["steering_deg"])
        )

        self.duration = float(config["duration"])

        # ------------------------------------------------------
        # Safety checks
        # ------------------------------------------------------

        if abs(self.command_speed) > self.MAX_SPEED:

            raise ValueError(
                "Experiment speed exceeds vehicle limit"
            )

        if abs(self.command_delta) > self.MAX_STEERING:

            raise ValueError(
                "Experiment steering exceeds vehicle limit"
            )

        # ------------------------------------------------------
        # Publishers
        # ------------------------------------------------------

        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            "/steering_controller/commands",
            10,
        )

        self.rear_wheel_pub = self.create_publisher(
            Float64MultiArray,
            "/rear_wheel_controller/commands",
            10,
        )

        # ------------------------------------------------------
        # Ground-truth odometry
        # ------------------------------------------------------

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            50,
        )

        # ------------------------------------------------------
        # Experiment state
        # ------------------------------------------------------

        self.have_odom = False

        self.experiment_started = False
        self.experiment_finished = False

        self.start_time = None
        self.last_model_time = None

        # Actual initial pose
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_theta = 0.0

        # Planned bicycle-model pose
        self.planned_x = 0.0
        self.planned_y = 0.0
        self.planned_theta = 0.0

        # ------------------------------------------------------
        # Data storage
        # ------------------------------------------------------

        self.data = []

        # ------------------------------------------------------
        # Calculate wheel commands
        # ------------------------------------------------------

        (
            self.left_steering,
            self.right_steering,
        ) = self.calculate_ackermann_angles(
            self.command_delta
        )

        self.rear_wheel_velocity = (
            self.command_speed / self.WHEEL_RADIUS
        )

        # ------------------------------------------------------
        # Control timer
        # ------------------------------------------------------

        self.timer = self.create_timer(
            0.02,
            self.control_loop,
        )

        # ------------------------------------------------------
        # Startup information
        # ------------------------------------------------------

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            "EE5112 Task 2 Trajectory Experiment"
        )

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            f"Experiment      : {self.experiment_name}"
        )

        self.get_logger().info(
            f"Speed           : {self.command_speed:.3f} m/s"
        )

        self.get_logger().info(
            f"Bicycle steering: "
            f"{math.degrees(self.command_delta):.2f} deg"
        )

        self.get_logger().info(
            f"Left steering   : "
            f"{math.degrees(self.left_steering):.2f} deg"
        )

        self.get_logger().info(
            f"Right steering  : "
            f"{math.degrees(self.right_steering):.2f} deg"
        )

        self.get_logger().info(
            f"Rear wheel speed: "
            f"{self.rear_wheel_velocity:.3f} rad/s"
        )

        self.get_logger().info(
            f"Duration        : {self.duration:.2f} s"
        )

        if abs(self.command_delta) > 1.0e-9:

            radius = (
                self.WHEELBASE
                / math.tan(abs(self.command_delta))
            )

            self.get_logger().info(
                f"Theoretical R   : {radius:.3f} m"
            )

        else:

            self.get_logger().info(
                "Theoretical R   : infinity"
            )

        self.get_logger().info(
            "Waiting for /odom..."
        )

    # ==========================================================
    # ACKERMANN GEOMETRY
    # ==========================================================

    def calculate_ackermann_angles(self, delta):

        """
        Convert an equivalent bicycle steering angle into
        physical left and right front-wheel steering angles.

        Positive delta = left turn.
        Negative delta = right turn.
        """

        if abs(delta) < 1.0e-9:

            return 0.0, 0.0

        radius = (
            self.WHEELBASE
            / math.tan(abs(delta))
        )

        half_track = self.TRACK_WIDTH / 2.0

        # Protect against impossible geometry
        if radius <= half_track:

            raise ValueError(
                "Requested turning radius is smaller "
                "than half the track width."
            )

        inner = math.atan(
            self.WHEELBASE
            / (radius - half_track)
        )

        outer = math.atan(
            self.WHEELBASE
            / (radius + half_track)
        )

        if delta > 0.0:

            # Left turn:
            # left wheel is inner wheel

            left = inner
            right = outer

        else:

            # Right turn:
            # right wheel is inner wheel

            left = -outer
            right = -inner

        # Safety clamp
        left = max(
            -self.MAX_STEERING,
            min(self.MAX_STEERING, left),
        )

        right = max(
            -self.MAX_STEERING,
            min(self.MAX_STEERING, right),
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
            cosy_cosp,
        )

    # ==========================================================
    # ANGLE NORMALIZATION
    # ==========================================================

    @staticmethod
    def normalize_angle(angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # ==========================================================
    # ODOMETRY CALLBACK
    # ==========================================================

    def odom_callback(self, msg):

        actual_x = msg.pose.pose.position.x
        actual_y = msg.pose.pose.position.y

        actual_theta = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        # First odometry sample
        if not self.have_odom:

            self.have_odom = True

            self.initial_x = actual_x
            self.initial_y = actual_y
            self.initial_theta = actual_theta

            self.planned_x = actual_x
            self.planned_y = actual_y
            self.planned_theta = actual_theta

            self.get_logger().info(
                "Ground-truth odometry received."
            )

            self.get_logger().info(
                "Initial pose: "
                f"x={actual_x:.4f}, "
                f"y={actual_y:.4f}, "
                f"theta="
                f"{math.degrees(actual_theta):.3f} deg"
            )

        # Do not record until experiment starts
        if not self.experiment_started:

            return

        if self.experiment_finished:

            return

        now = self.get_clock().now()

        elapsed = (
            now - self.start_time
        ).nanoseconds * 1.0e-9

        # ------------------------------------------------------
        # Actual velocity from P3D
        # ------------------------------------------------------

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        actual_speed = math.sqrt(
            vx * vx + vy * vy
        )

        # ------------------------------------------------------
        # Position error
        # ------------------------------------------------------

        position_error = math.hypot(
            actual_x - self.planned_x,
            actual_y - self.planned_y,
        )

        heading_error = self.normalize_angle(
            actual_theta - self.planned_theta
        )

        # ------------------------------------------------------
        # Save sample
        # ------------------------------------------------------

        self.data.append(
            {
                "time": elapsed,

                "planned_x": self.planned_x,
                "planned_y": self.planned_y,
                "planned_theta": self.planned_theta,

                "actual_x": actual_x,
                "actual_y": actual_y,
                "actual_theta": actual_theta,

                "command_v": self.command_speed,
                "actual_v": actual_speed,

                "command_delta": self.command_delta,

                "left_steering": self.left_steering,
                "right_steering": self.right_steering,

                "rear_wheel_velocity":
                    self.rear_wheel_velocity,

                "position_error": position_error,
                "heading_error": heading_error,
            }
        )

    # ==========================================================
    # PUBLISH VEHICLE COMMAND
    # ==========================================================

    def publish_command(self):

        steering_msg = Float64MultiArray()

        steering_msg.data = [
            self.left_steering,
            self.right_steering,
        ]

        self.steering_pub.publish(
            steering_msg
        )

        rear_msg = Float64MultiArray()

        rear_msg.data = [
            self.rear_wheel_velocity,
            self.rear_wheel_velocity,
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
            0.0,
        ]

        self.rear_wheel_pub.publish(
            rear_msg
        )

        steering_msg = Float64MultiArray()

        steering_msg.data = [
            0.0,
            0.0,
        ]

        self.steering_pub.publish(
            steering_msg
        )

    # ==========================================================
    # BICYCLE MODEL INTEGRATION
    # ==========================================================

    def update_bicycle_model(self, dt):

        theta = self.planned_theta

        self.planned_x += (
            self.command_speed
            * math.cos(theta)
            * dt
        )

        self.planned_y += (
            self.command_speed
            * math.sin(theta)
            * dt
        )

        yaw_rate = (
            self.command_speed
            / self.WHEELBASE
            * math.tan(self.command_delta)
        )

        self.planned_theta += (
            yaw_rate * dt
        )

        self.planned_theta = (
            self.normalize_angle(
                self.planned_theta
            )
        )

    # ==========================================================
    # MAIN CONTROL LOOP
    # ==========================================================

    def control_loop(self):

        if self.experiment_finished:
            return

        # Wait for ground-truth pose
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
                "============================================"
            )

            self.get_logger().info(
                "EXPERIMENT STARTED"
            )

            self.get_logger().info(
                "============================================"
            )

            return

        # ------------------------------------------------------
        # Elapsed time
        # ------------------------------------------------------

        elapsed = (
            now - self.start_time
        ).nanoseconds * 1.0e-9

        # ------------------------------------------------------
        # End experiment
        # ------------------------------------------------------

        if elapsed >= self.duration:

            self.finish_experiment()

            return

        # ------------------------------------------------------
        # Integrate bicycle model
        # ------------------------------------------------------

        dt = (
            now - self.last_model_time
        ).nanoseconds * 1.0e-9

        self.last_model_time = now

        # Ignore unreasonable timer jumps
        if 0.0 < dt < 0.2:

            self.update_bicycle_model(dt)

        # ------------------------------------------------------
        # Command Gazebo vehicle
        # ------------------------------------------------------

        self.publish_command()

    # ==========================================================
    # FINISH EXPERIMENT
    # ==========================================================

    def finish_experiment(self):

        if self.experiment_finished:
            return

        self.experiment_finished = True

        # Send stop command several times
        for _ in range(10):
            self.stop_vehicle()

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            "EXPERIMENT FINISHED"
        )

        self.get_logger().info(
            "============================================"
        )

        if len(self.data) < 2:

            self.get_logger().error(
                "Not enough data was recorded."
            )

        else:

            try:

                self.save_results()

            except Exception as exc:

                self.get_logger().error(
                    f"Failed to save results: {exc}"
                )

        # Shutdown shortly after writing files
        self.create_timer(
            1.0,
            self.shutdown_node,
        )

    # ==========================================================
    # RESULTS DIRECTORY
    # ==========================================================

    def get_results_directory(self):

        """
        Default:
        ~/EE5112_MiniLab/task2_results/
        """

        home = Path.home()

        results = (
            home
            / "EE5112_MiniLab"
            / "task2_results"
        )

        csv_dir = results / "csv"
        figure_dir = results / "figures"

        csv_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        figure_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        return csv_dir, figure_dir

    # ==========================================================
    # SAVE CSV
    # ==========================================================

    def save_csv(self, csv_path):

        fieldnames = [
            "time",

            "planned_x",
            "planned_y",
            "planned_theta",

            "actual_x",
            "actual_y",
            "actual_theta",

            "command_v",
            "actual_v",

            "command_delta",

            "left_steering",
            "right_steering",

            "rear_wheel_velocity",

            "position_error",
            "heading_error",
        ]

        with open(
            csv_path,
            "w",
            newline="",
            encoding="utf-8",
        ) as csv_file:

            writer = csv.DictWriter(
                csv_file,
                fieldnames=fieldnames,
            )

            writer.writeheader()

            writer.writerows(
                self.data
            )

    # ==========================================================
    # CALCULATE ERROR STATISTICS
    # ==========================================================

    def calculate_statistics(self):

        errors = [
            row["position_error"]
            for row in self.data
        ]

        heading_errors = [
            abs(row["heading_error"])
            for row in self.data
        ]

        mean_error = (
            sum(errors)
            / len(errors)
        )

        rms_error = math.sqrt(
            sum(e * e for e in errors)
            / len(errors)
        )

        max_error = max(errors)

        final_error = errors[-1]

        final_heading_error = (
            heading_errors[-1]
        )

        return {
            "mean_position_error": mean_error,
            "rms_position_error": rms_error,
            "max_position_error": max_error,
            "final_position_error": final_error,
            "final_heading_error":
                final_heading_error,
        }

    # ==========================================================
    # SAVE SUMMARY
    # ==========================================================

    def save_summary(
        self,
        summary_path,
        statistics,
    ):

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
            summary_path,
            "w",
            encoding="utf-8",
        ) as f:

            f.write(
                "EE5112 Task 2 Trajectory Experiment\n"
            )

            f.write(
                "===================================\n\n"
            )

            f.write(
                f"Experiment: "
                f"{self.experiment_name}\n"
            )

            f.write(
                f"Command speed: "
                f"{self.command_speed:.6f} m/s\n"
            )

            f.write(
                f"Bicycle steering: "
                f"{math.degrees(self.command_delta):.6f} deg\n"
            )

            f.write(
                f"Left steering: "
                f"{math.degrees(self.left_steering):.6f} deg\n"
            )

            f.write(
                f"Right steering: "
                f"{math.degrees(self.right_steering):.6f} deg\n"
            )

            if math.isfinite(
                theoretical_radius
            ):

                f.write(
                    f"Theoretical radius: "
                    f"{theoretical_radius:.6f} m\n"
                )

            else:

                f.write(
                    "Theoretical radius: infinity\n"
                )

            f.write("\n")

            f.write(
                f"Mean position error: "
                f"{statistics['mean_position_error']:.6f} m\n"
            )

            f.write(
                f"RMS position error: "
                f"{statistics['rms_position_error']:.6f} m\n"
            )

            f.write(
                f"Maximum position error: "
                f"{statistics['max_position_error']:.6f} m\n"
            )

            f.write(
                f"Final position error: "
                f"{statistics['final_position_error']:.6f} m\n"
            )

            f.write(
                f"Final heading error: "
                f"{math.degrees(statistics['final_heading_error']):.6f} deg\n"
            )

    # ==========================================================
    # TRAJECTORY PLOT
    # ==========================================================

    def plot_trajectory(
        self,
        output_path,
    ):

        planned_x = [
            row["planned_x"]
            for row in self.data
        ]

        planned_y = [
            row["planned_y"]
            for row in self.data
        ]

        actual_x = [
            row["actual_x"]
            for row in self.data
        ]

        actual_y = [
            row["actual_y"]
            for row in self.data
        ]

        plt.figure(figsize=(8, 6))

        plt.plot(
            planned_x,
            planned_y,
            linewidth=2.0,
            label="Planned bicycle model",
        )

        plt.plot(
            actual_x,
            actual_y,
            "--",
            linewidth=2.0,
            label="Recorded Gazebo",
        )

        plt.scatter(
            [planned_x[0]],
            [planned_y[0]],
            marker="o",
            s=60,
            label="Start",
        )

        plt.xlabel("x [m]")
        plt.ylabel("y [m]")

        plt.title(
            "Task 2 Planned vs Recorded Trajectory\n"
            f"{self.experiment_name}"
        )

        plt.axis("equal")
        plt.grid(True)
        plt.legend()

        plt.tight_layout()

        plt.savefig(
            output_path,
            dpi=200,
        )

        plt.close()

    # ==========================================================
    # STATE PLOT
    # ==========================================================

    def plot_states(
        self,
        output_path,
    ):

        time = [
            row["time"]
            for row in self.data
        ]

        planned_x = [
            row["planned_x"]
            for row in self.data
        ]

        actual_x = [
            row["actual_x"]
            for row in self.data
        ]

        planned_y = [
            row["planned_y"]
            for row in self.data
        ]

        actual_y = [
            row["actual_y"]
            for row in self.data
        ]

        planned_theta = [
            math.degrees(
                row["planned_theta"]
            )
            for row in self.data
        ]

        actual_theta = [
            math.degrees(
                row["actual_theta"]
            )
            for row in self.data
        ]

        # x
        plt.figure(figsize=(8, 5))

        plt.plot(
            time,
            planned_x,
            label="Planned x",
        )

        plt.plot(
            time,
            actual_x,
            "--",
            label="Actual x",
        )

        plt.xlabel("Time [s]")
        plt.ylabel("x [m]")

        plt.title(
            f"x(t) - {self.experiment_name}"
        )

        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.savefig(
            str(output_path).replace(
                ".png",
                "_x.png",
            ),
            dpi=200,
        )

        plt.close()

        # y
        plt.figure(figsize=(8, 5))

        plt.plot(
            time,
            planned_y,
            label="Planned y",
        )

        plt.plot(
            time,
            actual_y,
            "--",
            label="Actual y",
        )

        plt.xlabel("Time [s]")
        plt.ylabel("y [m]")

        plt.title(
            f"y(t) - {self.experiment_name}"
        )

        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.savefig(
            str(output_path).replace(
                ".png",
                "_y.png",
            ),
            dpi=200,
        )

        plt.close()

        # theta
        plt.figure(figsize=(8, 5))

        plt.plot(
            time,
            planned_theta,
            label="Planned theta",
        )

        plt.plot(
            time,
            actual_theta,
            "--",
            label="Actual theta",
        )

        plt.xlabel("Time [s]")
        plt.ylabel("Heading [deg]")

        plt.title(
            f"Heading - {self.experiment_name}"
        )

        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.savefig(
            str(output_path).replace(
                ".png",
                "_theta.png",
            ),
            dpi=200,
        )

        plt.close()

    # ==========================================================
    # CONTROL PLOT
    # ==========================================================

    def plot_controls(
        self,
        output_path,
    ):

        time = [
            row["time"]
            for row in self.data
        ]

        command_v = [
            row["command_v"]
            for row in self.data
        ]

        actual_v = [
            row["actual_v"]
            for row in self.data
        ]

        delta = [
            math.degrees(
                row["command_delta"]
            )
            for row in self.data
        ]

        # Velocity
        plt.figure(figsize=(8, 5))

        plt.plot(
            time,
            command_v,
            label="Commanded v",
        )

        plt.plot(
            time,
            actual_v,
            "--",
            label="Recorded speed",
        )

        plt.xlabel("Time [s]")
        plt.ylabel("Speed [m/s]")

        plt.title(
            f"Vehicle Speed - {self.experiment_name}"
        )

        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.savefig(
            str(output_path).replace(
                ".png",
                "_velocity.png",
            ),
            dpi=200,
        )

        plt.close()

        # Steering
        plt.figure(figsize=(8, 5))

        plt.plot(
            time,
            delta,
            label="Equivalent bicycle steering",
        )

        plt.xlabel("Time [s]")
        plt.ylabel("Steering angle [deg]")

        plt.title(
            f"Steering Input - {self.experiment_name}"
        )

        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.savefig(
            str(output_path).replace(
                ".png",
                "_steering.png",
            ),
            dpi=200,
        )

        plt.close()

    # ==========================================================
    # ERROR PLOT
    # ==========================================================

    def plot_error(
        self,
        output_path,
    ):

        time = [
            row["time"]
            for row in self.data
        ]

        error = [
            row["position_error"]
            for row in self.data
        ]

        plt.figure(figsize=(8, 5))

        plt.plot(
            time,
            error,
            linewidth=2.0,
        )

        plt.xlabel("Time [s]")
        plt.ylabel("Position error [m]")

        plt.title(
            f"Trajectory Error - {self.experiment_name}"
        )

        plt.grid(True)
        plt.tight_layout()

        plt.savefig(
            output_path,
            dpi=200,
        )

        plt.close()

    # ==========================================================
    # SAVE ALL RESULTS
    # ==========================================================

    def save_results(self):

        csv_dir, figure_dir = (
            self.get_results_directory()
        )

        csv_path = (
            csv_dir
            / f"{self.experiment_name}.csv"
        )

        summary_path = (
            csv_dir
            / f"{self.experiment_name}_summary.txt"
        )

        trajectory_path = (
            figure_dir
            / f"{self.experiment_name}_trajectory.png"
        )

        state_path = (
            figure_dir
            / f"{self.experiment_name}_states.png"
        )

        control_path = (
            figure_dir
            / f"{self.experiment_name}_controls.png"
        )

        error_path = (
            figure_dir
            / f"{self.experiment_name}_error.png"
        )

        # ------------------------------------------------------
        # Save
        # ------------------------------------------------------

        self.save_csv(
            csv_path
        )

        statistics = (
            self.calculate_statistics()
        )

        self.save_summary(
            summary_path,
            statistics,
        )

        self.plot_trajectory(
            trajectory_path
        )

        self.plot_states(
            state_path
        )

        self.plot_controls(
            control_path
        )

        self.plot_error(
            error_path
        )

        # ------------------------------------------------------
        # Terminal summary
        # ------------------------------------------------------

        self.get_logger().info(
            "Results saved successfully."
        )

        self.get_logger().info(
            f"CSV: {csv_path}"
        )

        self.get_logger().info(
            f"Figures: {figure_dir}"
        )

        self.get_logger().info(
            "--------------------------------------------"
        )

        self.get_logger().info(
            f"Mean position error: "
            f"{statistics['mean_position_error']:.4f} m"
        )

        self.get_logger().info(
            f"RMS position error: "
            f"{statistics['rms_position_error']:.4f} m"
        )

        self.get_logger().info(
            f"Max position error: "
            f"{statistics['max_position_error']:.4f} m"
        )

        self.get_logger().info(
            f"Final heading error: "
            f"{math.degrees(statistics['final_heading_error']):.3f} deg"
        )

        self.get_logger().info(
            "--------------------------------------------"
        )

    # ==========================================================
    # SHUTDOWN
    # ==========================================================

    def shutdown_node(self):

        self.stop_vehicle()

        self.get_logger().info(
            "Trajectory experiment node shutting down."
        )

        rclpy.shutdown()


# ==============================================================
# MAIN
# ==============================================================

def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryExperiment()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().warning(
            "Experiment interrupted by user."
        )

        node.stop_vehicle()

    finally:

        if rclpy.ok():

            node.stop_vehicle()

            node.destroy_node()

            rclpy.shutdown()


if __name__ == "__main__":

    main()
