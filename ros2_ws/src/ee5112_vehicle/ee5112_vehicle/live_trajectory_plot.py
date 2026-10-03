#!/usr/bin/env python3

"""EE5112 Task 2 live display for x, y, theta, v and delta."""

import math
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


class LiveTrajectoryPlot(Node):
    def __init__(self):
        super().__init__('live_trajectory_plot')

        self.command_x, self.command_y, self.command_theta = [], [], []
        self.measured_x, self.measured_y, self.measured_theta = [], [], []
        self.gazebo_x, self.gazebo_y, self.gazebo_theta = [], [], []
        self.time = []
        self.command_theta_time = []
        self.measured_theta_time = []
        self.gazebo_theta_time = []
        self.local_start_time = None
        self.last_telemetry_elapsed = None
        self.command_v, self.measured_v, self.odom_v = [], [], []
        self.command_delta, self.measured_delta = [], []

        self.initial_odom_received = False
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta0 = 0.0

        self.create_subscription(Pose2D, '/task2/command_model_pose', self.command_callback, 50)
        self.create_subscription(Pose2D, '/task2/measured_model_pose', self.measured_callback, 50)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 50)
        self.create_subscription(Float64MultiArray, '/task2/telemetry', self.telemetry_callback, 50)

        plt.ion()
        self.fig, axes = plt.subplots(
            2,
            2,
            figsize=(14, 9),
            constrained_layout=True
        )
        self.ax_xy = axes[0, 0]
        self.ax_theta = axes[0, 1]
        self.ax_v = axes[1, 0]
        self.ax_delta = axes[1, 1]

        try:
            self.fig.canvas.manager.set_window_title('EE5112 Task 2 - Live x, y, theta, v, delta')
        except Exception:
            pass

        # x-y trajectory.
        self.command_xy_line, = self.ax_xy.plot([], [], linewidth=2.0, label='Command model')
        self.measured_xy_line, = self.ax_xy.plot([], [], linewidth=2.0, label='Measured model')
        self.gazebo_xy_line, = self.ax_xy.plot([], [], '--', linewidth=2.0, label='Gazebo ground truth')
        self.vehicle_marker, = self.ax_xy.plot([], [], marker='o', markersize=7, linestyle='None', label='Current position')
        self.start_marker, = self.ax_xy.plot([], [], marker='x', markersize=8, linestyle='None', label='Start')
        self.ax_xy.set_title('x-y trajectory')
        self.ax_xy.set_xlabel('x displacement [m]')
        self.ax_xy.set_ylabel('y displacement [m]')
        self.ax_xy.grid(True)
        self.ax_xy.legend(loc='best', fontsize=8)
        self.ax_xy.set_aspect('auto')
        self.ax_xy.set_xlim(-0.1, 1.5)
        self.ax_xy.set_ylim(-1.0, 1.0)

        # theta.
        self.command_theta_line, = self.ax_theta.plot([], [], linewidth=2.0, label='Command model')
        self.measured_theta_line, = self.ax_theta.plot([], [], linewidth=2.0, label='Measured model')
        self.gazebo_theta_line, = self.ax_theta.plot([], [], '--', linewidth=2.0, label='Gazebo ground truth')
        self.ax_theta.set_title('Heading theta')
        self.ax_theta.set_xlabel('Time [s]')
        self.ax_theta.set_ylabel('theta [deg]')
        self.ax_theta.grid(True)
        self.ax_theta.legend(loc='best', fontsize=8)

        # v.
        self.command_v_line, = self.ax_v.plot([], [], linewidth=2.0, label='Commanded v')
        self.measured_v_line, = self.ax_v.plot([], [], linewidth=2.0, label='Wheel-derived v')
        self.odom_v_line, = self.ax_v.plot([], [], '--', linewidth=2.0, label='Gazebo /odom v')
        self.ax_v.set_title('Velocity v')
        self.ax_v.set_xlabel('Time [s]')
        self.ax_v.set_ylabel('v [m/s]')
        self.ax_v.grid(True)
        self.ax_v.legend(loc='best', fontsize=8)

        # delta.
        self.command_delta_line, = self.ax_delta.plot([], [], linewidth=2.0, label='Commanded delta')
        self.measured_delta_line, = self.ax_delta.plot([], [], '--', linewidth=2.0, label='Measured delta')
        self.ax_delta.set_title('Steering angle delta')
        self.ax_delta.set_xlabel('Time [s]')
        self.ax_delta.set_ylabel('delta [deg]')
        self.ax_delta.grid(True)
        self.ax_delta.legend(loc='best', fontsize=8)

        # Teleoperation uses a rolling 30-second time window.
        self.window_seconds = 30.0

        self.fig.suptitle('EE5112 Task 2 - Live Ackermann Validation: x, y, theta, v, delta')
        self.create_timer(0.05, self.update_plot)

        self.get_logger().info('Live plot ready. Listening to model poses, /odom and /task2/telemetry.')

    @staticmethod
    def quaternion_to_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def relative_angle(angle, reference):
        return math.atan2(math.sin(angle - reference), math.cos(angle - reference))

    def local_elapsed(self):
        if self.local_start_time is None:
            return 0.0
        return (self.get_clock().now() - self.local_start_time).nanoseconds * 1.0e-9

    def command_callback(self, msg):
        if not self.initial_odom_received:
            return
        self.command_x.append(msg.x - self.x0)
        self.command_y.append(msg.y - self.y0)
        self.command_theta_time.append(self.local_elapsed())
        self.command_theta.append(math.degrees(self.relative_angle(msg.theta, self.theta0)))

    def measured_callback(self, msg):
        if not self.initial_odom_received:
            return
        self.measured_x.append(msg.x - self.x0)
        self.measured_y.append(msg.y - self.y0)
        self.measured_theta_time.append(self.local_elapsed())
        self.measured_theta.append(math.degrees(self.relative_angle(msg.theta, self.theta0)))

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = self.quaternion_to_yaw(msg.pose.pose.orientation)
        if not self.initial_odom_received:
            self.initial_odom_received = True
            self.x0, self.y0, self.theta0 = x, y, theta
            self.local_start_time = self.get_clock().now()
            self.start_marker.set_data([0.0], [0.0])
            self.get_logger().info(
                f'Plot origin: x={self.x0:.4f}, y={self.y0:.4f}, theta={math.degrees(self.theta0):.2f} deg'
            )
        self.gazebo_x.append(x - self.x0)
        self.gazebo_y.append(y - self.y0)
        self.gazebo_theta_time.append(self.local_elapsed())
        self.gazebo_theta.append(math.degrees(self.relative_angle(theta, self.theta0)))

    def telemetry_callback(self, msg):
        if len(msg.data) < 7:
            return
        elapsed, command_v, _command_model_v, measured_v, odom_v, command_delta, measured_delta = msg.data[:7]
        if self.last_telemetry_elapsed is not None and elapsed + 0.5 < self.last_telemetry_elapsed:
            self.clear_plot_data()
        self.last_telemetry_elapsed = elapsed
        self.time.append(self.local_elapsed())
        self.command_v.append(command_v)
        self.measured_v.append(measured_v)
        self.odom_v.append(odom_v)
        self.command_delta.append(math.degrees(command_delta))
        self.measured_delta.append(math.degrees(measured_delta))

    def update_xy_limits(self):
        all_x = self.command_x + self.measured_x + self.gazebo_x
        all_y = self.command_y + self.measured_y + self.gazebo_y
        if not all_x or not all_y:
            return
        margin = 0.15
        xmin, xmax = min(all_x) - margin, max(all_x) + margin
        ymin, ymax = min(all_y) - margin, max(all_y) + margin
        if xmax - xmin < 0.8:
            xmax = xmin + 0.8
        if ymax - ymin < 0.8:
            mid = 0.5 * (ymin + ymax)
            ymin, ymax = mid - 0.4, mid + 0.4
        self.ax_xy.set_xlim(xmin, xmax)
        self.ax_xy.set_ylim(ymin, ymax)

    def clear_plot_data(self):
        self.command_x.clear(); self.command_y.clear(); self.command_theta.clear(); self.command_theta_time.clear()
        self.measured_x.clear(); self.measured_y.clear(); self.measured_theta.clear(); self.measured_theta_time.clear()
        self.gazebo_x.clear(); self.gazebo_y.clear(); self.gazebo_theta.clear(); self.gazebo_theta_time.clear()
        self.time.clear(); self.command_v.clear(); self.measured_v.clear(); self.odom_v.clear()
        self.command_delta.clear(); self.measured_delta.clear()
        if self.initial_odom_received:
            self.x0 = self.gazebo_x[-1] if self.gazebo_x else self.x0
        self.local_start_time = self.get_clock().now()

    def update_time_limits(self):
        candidates = []
        for values in (self.command_theta_time, self.measured_theta_time, self.gazebo_theta_time, self.time):
            if values:
                candidates.append(values[-1])
        if not candidates:
            return
        latest = max(candidates)
        xmin = max(0.0, latest - self.window_seconds)
        xmax = max(self.window_seconds, latest)
        for ax in (self.ax_theta, self.ax_v, self.ax_delta):
            ax.set_xlim(xmin, xmax)

    def update_plot(self):
        if self.command_x:
            self.command_xy_line.set_data(self.command_x, self.command_y)
        if self.measured_x:
            self.measured_xy_line.set_data(self.measured_x, self.measured_y)
        if self.gazebo_x:
            self.gazebo_xy_line.set_data(self.gazebo_x, self.gazebo_y)
            self.vehicle_marker.set_data([self.gazebo_x[-1]], [self.gazebo_y[-1]])
        self.update_xy_limits()

        self.command_theta_line.set_data(self.command_theta_time, self.command_theta)
        self.measured_theta_line.set_data(self.measured_theta_time, self.measured_theta)
        self.gazebo_theta_line.set_data(self.gazebo_theta_time, self.gazebo_theta)

        if self.time:
            self.command_v_line.set_data(self.time, self.command_v)
            self.measured_v_line.set_data(self.time, self.measured_v)
            self.odom_v_line.set_data(self.time, self.odom_v)
            self.command_delta_line.set_data(self.time, self.command_delta)
            self.measured_delta_line.set_data(self.time, self.measured_delta)

        self.update_time_limits()
        for ax in (self.ax_theta, self.ax_v, self.ax_delta):
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def close_plot(self):
        try:
            plt.close(self.fig)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = LiveTrajectoryPlot()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            plt.pause(0.01)
            if not plt.fignum_exists(node.fig.number):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.close_plot()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
