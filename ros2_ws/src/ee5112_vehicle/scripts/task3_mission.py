#!/usr/bin/env python3
# Task 3 owner: Mohammad Asif Bin Abdul Sahid (A0313732M), Group 13.
"""Ordered autonomous colour search using the existing EE5112 vehicle/Nav2.

Requires the already-tested colour_confirmation.py in the same scripts folder.
Developed with AI coding assistance.
"""
import copy
import json
import logging
import math
import os
import signal
import time

from colour_confirmation import ConfirmationGate, parse_colours
from task3_core import (Arena, Mission, add_blocks_to_map, bounded_velocity,
                        parse_command, shortlist_observation_poses, score_route,
                        RouteProgress, direction_legs, linear_target_approach, wrap_angle,
                        validate_final_approach_settings, controller_minimum_radius,
                        dubins_execution_points)


def main(args=None):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.clock import Clock, ClockType
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from rcl_interfaces.msg import ParameterDescriptor
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Twist, PoseStamped
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import ComputePathToPose, FollowPath
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    class Task3MissionNode(Node):
        def __init__(self):
            super().__init__('task3_mission')
            defaults = {
                'arena_file': os.path.join(os.path.dirname(__file__), '..', 'config', 'MiniLab1.2_platform_specs_5112.json'),
                'command_topic': '/task3/command', 'speech_topic': '/speech_command',
                'detections_topic': '/detected_colours', 'odom_topic': '/odom',
                'nav_action': '/follow_path', 'planner_action': '/compute_path_to_pose',
                'planner_id': 'GridBased', 'controller_id': 'FollowPath',
                'reverse_controller_id': 'ReversePath',
                # Map-frame base_link poses [x, y, yaw radians], facing into each room.
                'room1_control_pose': [0.69, 0.80, 1.5707963267948966],
                'room2_control_pose': [2.10, 0.80, 1.5707963267948966],
                'room3_control_pose': [3.465, 0.81, 1.5707963267948966],
                'room_control_goal_checker_id': 'room_control_goal_checker',
                'room_control_xy_tolerance_m': 0.08,
                'room_control_heading_tolerance_rad': 0.35,
                'room_control_settle_margin_m': 0.02,
                'room_retreat_timeout_s': 75.0, 'room_retreat_max_attempts': 3,
                'direction_deadband_m_s': 0.005,
                'cusp_goal_checker_id': 'cusp_goal_checker',
                'alignment_goal_checker_id': 'alignment_goal_checker',
                'final_approach_enabled': False,
                'final_controller_id': 'FinalApproach',
                'final_approach_speed_m_s': 0.08,
                'final_tracking_extension_m': 0.20,
                'final_stop_tolerance_m': 0.02,
                'final_entry_xy_tolerance_m': 0.08,
                'final_entry_settle_margin_m': 0.02,
                'final_entry_heading_tolerance_rad': 0.52,
                'final_cross_track_limit_m': 0.08,
                'final_heading_limit_rad': 0.70,
                'cusp_xy_tolerance_m': 0.025, 'cusp_heading_tolerance_rad': 0.20,
                'minimum_direction_leg_m': 0.08,
                'goal_checker_id': 'goal_checker', 'nav_cmd_topic': '/task3/nav_cmd_vel',
                'output_cmd_topic': '/cmd_vel', 'source_map_topic': '/map',
                'navigation_map_topic': '/task3/navigation_map', 'map_frame': 'map',
                'mission_timeout_s': 180.0, 'goal_timeout_s': 75.0,
                'final_approach_timeout_s': 60.0,
                'observation_timeout_s': 4.0, 'goal_response_timeout_s': 3.0,
                'cancel_timeout_s': 4.0, 'sensor_failure_timeout_s': 3.0,
                'sensor_freshness_s': 0.50, 'tf_max_age_s': 0.40,
                'tf_future_tolerance_s': 0.20, 'cmd_timeout_s': 0.30,
                'stop_settle_s': 0.30, 'max_goal_attempts': 12,
                'observation_radius_m': 0.45, 'confirmation_radius_m': 0.50,
                'final_approach_straight_m': 0.10,
                'confirmation_hold_s': 0.30, 'max_speed_m_s': 0.15,
                'controller_max_steering_deg': 35.0,
                'planning_max_candidates': 12, 'planning_budget_s': 12.0,
                'planning_feasible_candidates': 2, 'planning_selection_budget_s': 3.0,
                'planning_result_timeout_s': 4.0,
                'route_progress_timeout_s': 15.0, 'route_progress_min_m': 0.03,
            }
            readonly = ParameterDescriptor(read_only=True)
            for key, value in defaults.items():
                self.declare_parameter(key, value, readonly)
            self.p = {key: self.get_parameter(key).value for key in defaults}
            for key, value in defaults.items():
                if isinstance(value, bool):
                    if not isinstance(self.p[key], bool):
                        raise ValueError(key + ' must be a boolean.')
                elif isinstance(value, (float, int)):
                    if key in ('final_entry_settle_margin_m', 'room_control_settle_margin_m'):
                        if not math.isfinite(self.p[key]) or not 0 <= self.p[key] <= .05:
                            raise ValueError(key + ' must be finite and between 0 and .05 m.')
                    elif not math.isfinite(self.p[key]) or self.p[key] <= 0:
                        raise ValueError(key + ' must be finite and positive.')
            for key in ('max_goal_attempts', 'planning_max_candidates', 'planning_feasible_candidates',
                        'room_retreat_max_attempts'):
                if not isinstance(self.p[key], int):
                    raise ValueError(key + ' must be an integer.')
            if self.p['max_speed_m_s'] > .50:
                raise ValueError('Mission speed exceeds the vehicle speed limit.')
            if not .40 <= self.p['observation_radius_m'] <= .49:
                raise ValueError('observation_radius_m must be between .40 and .49.')
            validate_final_approach_settings(
                self.p['final_approach_straight_m'], self.p['final_entry_heading_tolerance_rad'],
                self.p['final_heading_limit_rad'])
            if self.p['final_entry_xy_tolerance_m']+self.p['final_entry_settle_margin_m'] > .15:
                raise ValueError('final entry XY tolerance plus settling margin must be at most .15 m.')
            if self.p['final_approach_speed_m_s'] > self.p['max_speed_m_s']:
                raise ValueError('Final approach speed must not exceed mission speed.')
            if not .05 <= self.p['final_tracking_extension_m'] <= .25:
                raise ValueError('Final tracking extension must be between .05 and .25 m.')
            if self.p['final_stop_tolerance_m'] > .03:
                raise ValueError('Final stop tolerance must be at most .03 m.')
            if self.p['direction_deadband_m_s'] > .01:
                raise ValueError('direction_deadband_m_s must be at most .01.')
            if self.p['nav_cmd_topic'] == self.p['output_cmd_topic']:
                raise ValueError('Nav2 input and vehicle output topics must be different.')
            with open(self.p['arena_file'], encoding='utf-8') as stream:
                arena_data = json.load(stream)
            self.arena = Arena(arena_data)
            self.room_controls = self.arena.control_poses({
                'Room1': self.p['room1_control_pose'], 'Room2': self.p['room2_control_pose'],
                'Room3': self.p['room3_control_pose']})
            self.departure_room = self.departure_colour = None
            self.route_stage = 'BOX'
            self.retreat_attempts = 0
            self.retreat_control_room = None
            self.retreat_failed_controls = set()
            self.controller_minimum_radius_m = controller_minimum_radius(
                arena_data['vehicle'], self.p['controller_max_steering_deg'])
            self.m = Mission()
            self.gate = ConfirmationGate(self.arena.blocks, self.p['confirmation_radius_m'],
                                         self.p['confirmation_hold_s'], self.p['sensor_freshness_s'])
            self.buffer = Buffer(cache_time=Duration(seconds=5.0))
            self.listener = TransformListener(self.buffer, self)
            self.nav = ActionClient(self, FollowPath, self.p['nav_action'])
            self.planner = ActionClient(self, ComputePathToPose, self.p['planner_action'])
            self.plan_batch = self.plan_request = None
            self.route_progress = None
            self.progress_mark = self.progress_ros = 0.
            self.route_metrics = None
            self.route_legs, self.leg_index = [], 0
            self.leg_started_ros = 0.
            self.nav_goal_handle = None
            self.cancel_sent = False
            self.tried = set()
            self.target_key = None
            self.pose = None
            self.visible = set()
            self.last_detection_message = None
            self.last_detection_error = None
            self.detection_wall = self.odom_wall = None
            self.odom_speed = self.odom_yaw_rate = math.inf
            self.odom_vx = self.odom_wz = math.nan
            self.odom_child_frame = 'unknown'
            self.cmd = (0., 0.)
            self.raw_nav_cmd = (0., 0.)
            self.raw_nav_cmd_wall = None
            self.last_motion_output = (0., 0.)
            self.last_motion_gate = 'startup'
            self.selected_nav_goal = None
            self.cmd_wall = 0.
            self.cmd_token = None
            self.stopped_since = self.unhealthy_since = None
            self.last_ros = 0.
            self.last_status = None
            self.reported_terminal = None
            self.ready_logged = None
            self.map_ready = False
            self.map_signature = None
            self.map_error = 'waiting_for_map'
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.cmd_pub = self.create_publisher(Twist, self.p['output_cmd_topic'], qos)
            self.status_pub = self.create_publisher(String, '/mission_status', latched)
            self.found_pub = self.create_publisher(String, '/task3/found', qos)
            self.route_pub = self.create_publisher(Path, '/task3/selected_path', latched)
            self.active_route_pub = self.create_publisher(Path, '/task3/active_path', latched)
            self.final_route_pub = self.create_publisher(Path, '/task3/final_approach', latched)
            if not self.p['final_approach_enabled']:
                # Clear any final-line preview RViz retained from the last run.
                empty_final = Path()
                empty_final.header.frame_id = self.p['map_frame']
                self.final_route_pub.publish(empty_final)
            self.nav_map_pub = self.create_publisher(OccupancyGrid, self.p['navigation_map_topic'], latched)
            self.command_sub = self.create_subscription(String, self.p['command_topic'], self.on_command, 10)
            self.speech_sub = self.create_subscription(
                String, self.p['speech_topic'], lambda msg: self.on_command(msg, True), 10)
            self.detection_sub = self.create_subscription(
                String, self.p['detections_topic'], self.on_detection, qos)
            self.odom_sub = self.create_subscription(Odometry, self.p['odom_topic'], self.on_odom, qos)
            self.cmd_sub = self.create_subscription(Twist, self.p['nav_cmd_topic'], self.on_nav_cmd, qos)
            self.map_sub = self.create_subscription(OccupancyGrid, self.p['source_map_topic'], self.on_map, latched)
            # Verify that BOTH live navigation costmaps actually contain the blocks.
            self.costmap_ok = {'global': False, 'local': False}
            self.costmap_subs = [self.create_subscription(
                OccupancyGrid, '/' + name + '_costmap/costmap',
                lambda msg, key=name: self.on_costmap(key, msg), latched)
                for name in ('global', 'local')]
            self.lifecycle = {}
            self.lifecycle_pending = set()
            self.lifecycle_clients = {name: self.create_client(GetState, '/' + name + '/get_state')
                                      for name in ('planner_server', 'controller_server')}
            self.steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.tick_timer = self.create_timer(.05, self.on_tick, clock=self.steady)
            self.health_timer = self.create_timer(1., self.on_health, clock=self.steady)
            self.get_logger().info('[TASK3] Waiting for map, camera, localization, converter and Nav2.')
            self.get_logger().debug(
                f"[LIMITS] controller_max_steering={self.p['controller_max_steering_deg']:.2f}deg "
                f"controller_minimum_radius={self.controller_minimum_radius_m:.4f}m "
                f"cusp_xy={self.p['cusp_xy_tolerance_m']:.3f}m "
                f"cusp_heading={self.p['cusp_heading_tolerance_rad']:.3f}rad "
                f"final_entry_xy={self.p['final_entry_xy_tolerance_m']:.3f}m "
                f"final_entry_settle_margin={self.p['final_entry_settle_margin_m']:.3f}m "
                f"final_entry_heading={self.p['final_entry_heading_tolerance_rad']:.3f}rad "
                f"final_heading_limit={self.p['final_heading_limit_rad']:.3f}rad "
                f"final_cross_track_limit={self.p['final_cross_track_limit_m']:.3f}m "
                f"final_approach_enabled={self.p['final_approach_enabled']} "
                f"final_straight_min={self.p['final_approach_straight_m']:.3f}m")
            self.get_logger().debug(
                f"[CONTROLLERS] forward={self.p['controller_id']} "
                f"reverse={self.p['reverse_controller_id']} final={self.p['final_controller_id']}")
            for room, pose in self.room_controls.items():
                self.get_logger().debug(
                    f'[ROOM_CONTROL] room={room} rear=({pose[0]:.3f},{pose[1]:.3f},'
                    f'{math.degrees(pose[2]):.1f}deg) planner=DUBIN')

        def ros_now(self):
            return self.get_clock().now().nanoseconds / 1e9

        def get_pose(self, frame='base_link'):
            now = self.ros_now()
            try:
                t = self.buffer.lookup_transform(self.p['map_frame'], frame, Time())
            except TransformException:
                return None, 'missing_map_' + frame + '_tf'
            stamp = t.header.stamp.sec + t.header.stamp.nanosec/1e9
            if stamp <= 0 or now-stamp > self.p['tf_max_age_s']:
                return None, 'stale_map_' + frame + '_tf'
            if stamp-now > self.p['tf_future_tolerance_s']:
                return None, 'future_map_' + frame + '_tf'
            q = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            pose = (t.transform.translation.x, t.transform.translation.y, yaw)
            if not all(math.isfinite(v) for v in pose):
                return None, 'invalid_pose'
            return pose, None

        def on_map(self, msg):
            try:
                if msg.header.frame_id != self.p['map_frame']:
                    raise ValueError('Source map must use the configured map frame.')
                q = msg.info.origin.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                origin = (msg.info.origin.position.x, msg.info.origin.position.y, yaw)
                result = copy.deepcopy(msg)
                result.data = add_blocks_to_map(
                    msg.data, msg.info.width, msg.info.height, msg.info.resolution,
                    origin, self.arena.blocks, self.arena.block_size)
                result.header.stamp = self.get_clock().now().to_msg()
                self.nav_map_pub.publish(result)
                signature = (msg.info.width, msg.info.height, msg.info.resolution, origin, bytes((v+1) for v in msg.data))
                if self.map_signature is not None and signature != self.map_signature and self.m.active:
                    self.request_stop('fail', 'source_map_changed_during_mission')
                self.map_signature = signature
                self.map_ready, self.map_error = True, ''
            except (ValueError, TypeError, OverflowError) as exc:
                self.map_ready, self.map_error = False, str(exc)
                self.get_logger().error('[MAP_ERROR] ' + str(exc))
                if self.m.active:
                    self.request_stop('fail', 'invalid_source_map')

        def on_costmap(self, name, msg):
            # This is a configuration/obstacle-presence check. Nav2 owns collision checking.
            if msg.header.frame_id != ('map' if name == 'global' else 'odom'):
                return
            origin = msg.info.origin.position
            q = msg.info.origin.orientation
            if abs(q.x)+abs(q.y)+abs(q.z) > 1e-5 or msg.info.resolution <= 0:
                self.costmap_ok[name] = False
                return
            c, s, tx, ty = 1., 0., 0., 0.
            if name == 'local':
                try:
                    transform = self.buffer.lookup_transform('odom', self.p['map_frame'], Time())
                except TransformException:
                    self.costmap_ok[name] = False
                    return
                q = transform.transform.rotation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                c, s = math.cos(yaw), math.sin(yaw)
                tx, ty = transform.transform.translation.x, transform.transform.translation.y
            checks = []
            for x, y in self.arena.blocks.values():
                x, y = c*x-s*y+tx, s*x+c*y+ty
                col = math.floor((x-origin.x)/msg.info.resolution)
                row = math.floor((y-origin.y)/msg.info.resolution)
                i = row*msg.info.width+col
                if name == 'local' and not (0 <= col < msg.info.width and 0 <= row < msg.info.height):
                    continue  # Only blocks inside the rolling window are expected.
                checks.append(0 <= col < msg.info.width and 0 <= row < msg.info.height and
                              0 <= i < len(msg.data) and msg.data[i] >= 99)
            self.costmap_ok[name] = bool(msg.data) and all(checks)

        def on_odom(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            age = self.ros_now()-stamp
            v = msg.twist.twist
            if stamp <= 0 or age > self.p['sensor_freshness_s'] or age < -.2:
                return
            values = (v.linear.x, v.linear.y, v.angular.z)
            if not all(math.isfinite(x) for x in values):
                return
            self.odom_wall = time.monotonic()
            self.odom_speed = math.hypot(v.linear.x, v.linear.y)
            self.odom_yaw_rate = abs(v.angular.z)
            self.odom_vx, self.odom_wz = v.linear.x, v.angular.z
            self.odom_child_frame = msg.child_frame_id or 'unknown'

        def sensor_problem(self, wall):
            if self.ros_now() <= 0:
                return 'waiting_for_clock'
            if self.detection_wall is None or wall-self.detection_wall > self.p['sensor_freshness_s']:
                return 'stale_camera_detections'
            if self.odom_wall is None or wall-self.odom_wall > self.p['sensor_freshness_s']:
                return 'stale_odometry'
            self.pose, reason = self.get_pose()
            return reason

        def readiness(self):
            wall = time.monotonic()
            reason = self.sensor_problem(wall)
            if reason:
                return reason
            if not self.map_ready:
                return self.map_error
            if not all(self.costmap_ok.values()):
                return 'waiting_for_navigation_costmaps_with_blocks'
            if not self.nav.server_is_ready():
                return 'follow_path_unavailable'
            if not self.planner.server_is_ready():
                return 'compute_path_to_pose_unavailable'
            if self.plan_request is not None:
                return 'waiting_for_planner_cleanup'
            for name in self.lifecycle_clients:
                state, stamp = self.lifecycle.get(name, (0, 0.))
                if state != 3 or wall-stamp > 4.:
                    return name + '_not_active'
            _, reason = self.get_pose('nav_base')
            if reason:
                return reason
            if self.count_publishers(self.p['output_cmd_topic']) != 1:
                return 'multiple_cmd_vel_publishers_stop_other_navigation_or_teleop'
            if self.count_subscribers(self.p['output_cmd_topic']) == 0:
                return 'cmd_vel_converter_not_connected'
            if self.odom_speed >= .02 or self.odom_yaw_rate >= .10:
                return 'waiting_for_vehicle_to_stop'
            if self.count_publishers(self.p['nav_cmd_topic']) != 1:
                return 'nav2_must_publish_task3_nav_cmd_vel'
            return None

        def on_command(self, msg, speech=False):
            if msg.data.strip().lower() in ('cancel', 'stop'):
                if self.m.active:
                    self.request_stop('fail', 'cancelled_by_user')
                return
            try:
                names = parse_command(msg.data, allow_bare_list=speech)
                if not self.m.can_start:
                    raise ValueError('busy' if self.m.active else 'locked_restart_after_navigation_cleanup')
                problem = self.readiness()
                if problem:
                    raise ValueError(problem)
                self.m.start(names, self.ros_now(), time.monotonic())
            except ValueError as exc:
                self.get_logger().warning('[CMD_REJECTED] reason=' + str(exc))
                return
            self.tried.clear()
            self.target_key = None
            # Also recover a departure after a node restart / failed approach.
            # Home and the corridor need no initial retreat. A new command at
            # a box uses the robot's current room, never the target's room.
            self.departure_room = self.arena.room_at(self.pose)
            if (self.departure_colour is not None and
                    self.arena.block_rooms[self.departure_colour] != self.departure_room):
                self.departure_colour = None
            self.route_stage = 'BOX'
            self.retreat_attempts = 0
            self.retreat_control_room = None
            self.retreat_failed_controls.clear()
            self.unhealthy_since = None
            self.reported_terminal = None
            self.get_logger().info('[CMD] colours=' + ', '.join(names) + f' n={len(names)}')
            self.send_next_goal()

        def pose_message(self, pose):
            msg = PoseStamped()
            msg.header.frame_id = self.p['map_frame']
            msg.header.stamp = self.get_clock().now().to_msg()
            x, y, yaw = pose
            msg.pose.position.x, msg.pose.position.y = x, y
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            return msg

        def send_next_goal(self):
            if self.m.phase != 'NEED_GOAL' or self.plan_request is not None:
                return
            key = (self.m.mission_id, self.m.index)
            if self.target_key != key:
                self.target_key = key
                self.tried.clear()
                self.gate.arm(self.m.target)
            pose, problem = self.get_pose()
            start, start_problem = self.get_pose('nav_base')
            if problem or start_problem:
                self.request_stop('fail', problem or start_problem)
                return
            if self.departure_room is not None and self.retreat_attempts == 0:
                ready_room = self.retreat_control_room or self.departure_room
                control = self.room_controls[ready_room]
                # A stationary command at the control pose can already plan
                # towards a box. Do not ask Dubins for a zero-length loop.
                if (math.dist(start[:2], control[:2]) <= self.p['room_control_xy_tolerance_m'] and
                        abs(wrap_angle(start[2]-control[2])) <= self.p['room_control_heading_tolerance_rad']):
                    self.get_logger().info(
                        f'[ROOM_CONTROL_READY] room={self.departure_room} control_room={ready_room}')
                    self.departure_room = self.departure_colour = None
                    self.retreat_control_room = None
                    self.retreat_failed_controls.clear()
            self.route_stage = 'RETREAT' if self.departure_room is not None else 'BOX'
            if self.retreat_active():
                if self.retreat_attempts >= self.p['room_retreat_max_attempts']:
                    self.request_stop('fail', 'room_retreat_attempts_exhausted_' + self.departure_room)
                    return
                options = self.arena.retreat_control_candidates(
                    self.departure_room, self.room_controls, start, self.retreat_failed_controls)
                if not options:
                    self.request_stop('fail', 'room_retreat_controls_exhausted_' + self.departure_room)
                    return
                self.retreat_control_room = None
                self.get_logger().debug(
                    f'[RETREAT_OPTIONS] departure_room={self.departure_room} '
                    f'order={",".join(option["control_room"] for option in options)} '
                    f'heading={math.degrees(start[2]):.1f}deg')
                self.gate.invalidate('room_retreat')
            else:
                options = self.planning_candidates(pose)
            if not options or (not self.retreat_active() and len(self.tried) >= self.p['max_goal_attempts']):
                self.request_stop('fail', 'viewpoints_exhausted_' + self.m.target)
                return
            self.m.planning()
            self.selected_nav_goal = None
            self.route_metrics = None
            self.route_legs, self.leg_index = [], 0
            self.plan_batch = {'options': options, 'start': start,
                'started': time.monotonic(), 'tested': 0, 'feasible': 0, 'best': None,
                'stage': self.route_stage, 'gear': -1 if self.retreat_active() else 1}
            self.get_logger().info(
                f'[PLAN] target={self.m.target} stage={self.route_stage} candidates={len(options)}')
            self.plan_next_candidate()

        def planning_candidates(self, pose):
            enabled = self.p['final_approach_enabled']
            minimum_straight = self.p['final_approach_straight_m'] if enabled else 0.
            options = self.arena.observation_poses(
                self.m.target, pose, self.p['observation_radius_m'], minimum_straight,
                self.p['final_entry_xy_tolerance_m']+self.p['final_entry_settle_margin_m'],
                final_approach_enabled=enabled)
            options = [dict(p, minimum_straight_m=minimum_straight)
                       for p in options if p['id'] not in self.tried]
            return shortlist_observation_poses(options, self.p['planning_max_candidates'])

        def plan_next_candidate(self):
            batch = self.plan_batch
            if self.m.phase != 'PLANNING' or batch is None or self.plan_request is not None:
                return
            if self.ros_now()-self.m.started_ros >= self.p['mission_timeout_s']:
                self.request_stop('fail', 'mission_timeout')
                return
            # Only try a neighbouring control if the preferred one has no valid route.
            if batch['stage'] == 'RETREAT' and batch['best'] is not None:
                self.finish_planning()
                return
            # Compare a few valid box routes, then depart. If none has passed
            # all geometry checks, retain the full search budget/candidate list.
            # Check only between requests: an in-flight planner may run beyond
            # this soft selection budget and must finish before we can move.
            if (batch['stage'] == 'BOX' and batch['best'] is not None and
                    (batch['feasible'] >= self.p['planning_feasible_candidates'] or
                     time.monotonic()-batch['started'] >= self.p['planning_selection_budget_s'])):
                self.finish_planning()
                return
            if (not batch['options'] or
                    time.monotonic()-batch['started'] >= self.p['planning_budget_s']):
                self.finish_planning()
                return
            candidate = batch['options'].pop(0)
            batch['tested'] += 1
            if batch['stage'] == 'RETREAT':
                self.get_logger().debug(
                    f'[RETREAT_PLAN] departure_room={self.departure_room} '
                    f'control_room={candidate["control_room"]} '
                    f'fallback={candidate["control_room"] != self.departure_room}')
            goal = ComputePathToPose.Goal()
            destination = candidate.get('entry_nav', candidate['nav'])
            # Stock Dubins only searches forward. B->A with the REAL body
            # headings yields an A->B reverse-only route when ordered backwards.
            reverse = batch.get('gear', 1) < 0
            goal.goal = self.pose_message(batch['start'] if reverse else destination)
            goal.start = self.pose_message(destination if reverse else batch['start'])
            goal.use_start = True
            goal.planner_id = self.p['planner_id']
            request = {'batch': batch, 'candidate': candidate, 'handle': None,
                       'sent': time.monotonic(), 'cancel_sent': False}
            self.plan_request = request
            try:
                future = self.planner.send_goal_async(goal)
                future.add_done_callback(lambda f, r=request: self.on_plan_response(r, f))
            except Exception as exc:
                # Delivery is uncertain. Retain the request to prevent overlap.
                self.request_stop('fail', 'planner_send_error_' + str(exc))

        def on_plan_response(self, request, future):
            try:
                handle = future.result()
            except Exception as exc:
                if self.plan_request is request:
                    self.request_stop('fail', 'planner_response_error_' + str(exc))
                return
            if not handle.accepted:
                if self.plan_request is request:
                    self.plan_request = None
                    self.plan_next_candidate()
                return
            request['handle'] = handle
            try:
                handle.get_result_async().add_done_callback(lambda f, r=request: self.on_plan_result(r, f))
            except Exception as exc:
                self.request_stop('fail', 'planner_result_registration_error_' + str(exc))
                return
            if (self.plan_request is not request or self.plan_batch is not request['batch']
                    or self.m.phase != 'PLANNING'):
                request['cancel_sent'] = True
                try:
                    handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().error('[PLAN_CANCEL_ERROR] ' + str(exc))

        def on_plan_result(self, request, future):
            if self.plan_request is not request:
                return
            try:
                result = future.result()
            except Exception as exc:
                # A failed transport is not evidence the server finished.
                self.request_stop('fail', 'planner_result_error_' + str(exc))
                return
            self.plan_request = None
            batch = request['batch']
            if self.plan_batch is not batch or self.m.phase != 'PLANNING':
                return
            candidate = request['candidate']
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                try:
                    path = result.result.path
                    if path.header.frame_id != self.p['map_frame']:
                        raise ValueError('unexpected_path_frame')
                    points = []
                    for entry in path.poses:
                        if entry.header.frame_id not in ('', self.p['map_frame']):
                            raise ValueError('unexpected_pose_frame')
                        q, pos = entry.pose.orientation, entry.pose.position
                        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                        points.append((pos.x, pos.y, yaw))
                    if 'gear' in batch:
                        points = dubins_execution_points(points, batch['gear'])
                        if batch['gear'] < 0:
                            path = copy.deepcopy(path)
                            path.poses.reverse()  # Keep each quaternion unchanged.
                    planner_goal = candidate.get('entry_nav', candidate['nav'])
                    if not points or math.dist(points[-1][:2], planner_goal[:2]) > .03:
                        raise ValueError('path_does_not_reach_candidate')
                    end_yaw_error = points[-1][2]-planner_goal[2]
                    if abs(math.atan2(math.sin(end_yaw_error), math.cos(end_yaw_error))) > .20:
                        raise ValueError('path_does_not_reach_entry_heading')
                    if math.dist(points[0][:2], batch['start'][:2]) > .05:
                        raise ValueError('path_start_mismatch')
                    start_yaw_error = points[0][2]-batch['start'][2]
                    if abs(math.atan2(math.sin(start_yaw_error), math.cos(start_yaw_error))) > .15:
                        raise ValueError('path_start_heading_mismatch')
                    # The box route stops at entry. Score a straight preview,
                    # but never send that preview as part of the staging action.
                    legs = direction_legs(points)
                    # Apply the independently configured minimum to every leg,
                    # including a single staging leg before the final line.
                    if len(legs) > 1 or 'entry_nav' in candidate:
                        index, shortest_leg = min(enumerate(legs), key=lambda item: item[1]['length_m'])
                        shortest = shortest_leg['length_m']
                        if shortest < self.p['minimum_direction_leg_m']-1e-9:
                            location = ('single_staging' if len(legs) == 1 else 'first' if index == 0
                                        else 'last_staging' if index == len(legs)-1 else 'middle')
                            raise ValueError(
                                'direction_leg_too_short_for_control_tolerance '
                                f'length={shortest:.3f}m minimum={self.p["minimum_direction_leg_m"]:.3f}m '
                                f'leg={index+1}/{len(legs)} '
                                f'direction={"forward" if shortest_leg["gear"] > 0 else "reverse"} '
                                f'location={location} final_min={candidate.get("minimum_straight_m", self.p["final_approach_straight_m"]):.3f}m')
                    approach = None
                    complete_points = points
                    if 'entry_nav' in candidate:
                        approach = linear_target_approach(
                            points[-1], self.arena.blocks[self.m.target],
                            self.p['observation_radius_m'], candidate.get('minimum_straight_m', self.p['final_approach_straight_m']),
                            self.p['final_entry_heading_tolerance_rad'], self.p['final_tracking_extension_m'])
                        # Check the full tracking reference too, without counting
                        # its lookahead-only tail as distance the robot will drive.
                        score_route(approach['tracking_points'], self.arena)
                        complete_points = points+approach['points'][1:]
                    metrics = score_route(complete_points, self.arena)
                    if approach is not None:
                        metrics['final_straight_m'] = approach['straight_m']
                        metrics['final_straight_min_m'] = candidate.get('minimum_straight_m', self.p['final_approach_straight_m'])
                    batch['feasible'] += 1
                    key = (metrics['score'], candidate['id'])
                    if batch['best'] is None or key < batch['best']['key']:
                        batch['best'] = {'key': key, 'candidate': candidate, 'path': path,
                                         'points': points, 'metrics': metrics, 'legs': legs, 'linear': approach}
                    self.get_logger().debug(
                        f"[PLAN_CANDIDATE] id={candidate['id']} length={metrics['length_m']:.2f}m "
                        f"reverse={metrics['reverse_m']:.2f}m cusps={metrics['cusps']} "
                        f"score={metrics['score']:.2f} "
                        f"elapsed={time.monotonic()-request['sent']:.2f}s "
                        f"final_straight={metrics.get('final_straight_m', 0.):.3f}m")
                except (ValueError, TypeError, OverflowError) as exc:
                    self.get_logger().debug(f"[PLAN_REJECTED] id={candidate['id']} reason={exc}")
            else:
                self.get_logger().debug(f"[PLAN_REJECTED] id={candidate['id']} status={result.status}")
            if batch['stage'] == 'RETREAT' and batch['best'] is None:
                # Do not return to an already rejected control after a later
                # fallback's movement fails during this same departure.
                self.retreat_failed_controls.add(candidate['control_room'])
            self.plan_next_candidate()

        def finish_planning(self):
            batch = self.plan_batch
            if batch is None or self.m.phase != 'PLANNING' or self.plan_request is not None:
                return
            best = batch['best']
            self.plan_batch = None
            self.get_logger().info(
                f'[PLAN_DONE] stage={batch["stage"]} tested={batch["tested"]} '
                f'feasible={batch["feasible"]} elapsed={time.monotonic()-batch["started"]:.2f}s '
                f'selected={best["candidate"]["id"] if best is not None else "none"}')
            if best is None:
                reason = ('no_reverse_route_to_own_or_adjacent_control_' + self.departure_room if self.retreat_active()
                          else 'no_feasible_route_in_planning_budget_' + self.m.target)
                self.request_stop('fail', reason)
                return
            current, problem = self.get_pose('nav_base')
            if problem:
                self.request_stop('fail', problem)
                return
            dyaw = current[2]-batch['start'][2]
            if (math.dist(current[:2], batch['start'][:2]) > .05 or
                    abs(math.atan2(math.sin(dyaw), math.cos(dyaw))) > .15):
                self.request_stop('retry', 'localization_changed_during_planning')
                return
            candidate = best['candidate']
            self.selected_nav_goal = candidate['nav']
            if self.retreat_active():
                self.retreat_attempts += 1
                self.retreat_control_room = candidate['control_room']
                self.get_logger().info(
                    f'[RETREAT_SELECTED] departure_room={self.departure_room} '
                    f'control_room={self.retreat_control_room} '
                    f'fallback={self.retreat_control_room != self.departure_room}')
            else:
                self.tried.add(candidate['id'])
            self.route_metrics = best['metrics']
            # Map-fixed waypoints are timeless, not historical sensor poses.
            # Humble stores the final pose for its map->odom goal check. Use
            # latest TF even after this persistent path outlives the TF cache.
            for header in [best['path'].header, *[p.header for p in best['path'].poses]]:
                header.stamp.sec, header.stamp.nanosec = 0, 0
            preview = (self.path_with_points(best['path'], best['points']+best['linear']['points'][1:])
                       if best['linear'] is not None else best['path'])
            self.route_pub.publish(preview)
            self.route_legs, self.leg_index = [], 0
            for leg in best['legs']:
                first, last = leg['first'], leg['last']
                path = best['path'] if len(best['legs']) == 1 else copy.deepcopy(best['path'])
                if len(best['legs']) > 1:
                    path.poses = path.poses[first:last+1]
                self.route_legs.append(dict(leg, path=path, points=best['points'][first:last+1],
                                            mode=('retreat' if self.retreat_active() else
                                                  'staging' if self.p['final_approach_enabled'] else 'viewing')))
            if best['linear'] is not None:
                points = best['linear']['points']
                self.route_legs.append({'mode': 'linear', 'points': points,
                                        'path': self.path_with_points(best['path'], best['linear']['tracking_points']),
                                        'minimum_straight_m': candidate.get('minimum_straight_m', self.p['final_approach_straight_m']),
                                        'gear': 1, 'length_m': best['linear']['straight_m']})
            self.get_logger().debug(
                f"[NAV] target={self.m.target} stage={self.route_stage} "
                f"attempt={self.retreat_attempts if self.retreat_active() else len(self.tried)} "
                f"candidate={candidate['id']} length={best['metrics']['length_m']:.2f}m "
                f"reverse={best['metrics']['reverse_m']:.2f}m cusps={best['metrics']['cusps']} "
                f"final_straight={best['metrics'].get('final_straight_m', 0.):.3f}m "
                f"final_min={best['metrics'].get('final_straight_min_m', 0.):.3f}m")
            self.send_route_leg()

        def path_with_points(self, template, points):
            path = copy.deepcopy(template)
            path.header.stamp.sec = path.header.stamp.nanosec = 0
            path.poses = []
            for x, y, yaw in points:
                entry = copy.deepcopy(template.poses[0])
                entry.header.stamp.sec = entry.header.stamp.nanosec = 0
                entry.pose.position.x, entry.pose.position.y = x, y
                entry.pose.orientation.x = entry.pose.orientation.y = 0.
                entry.pose.orientation.z, entry.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                path.poses.append(entry)
            return path

        def final_approach_active(self):
            return (bool(self.route_legs) and
                    self.route_legs[self.leg_index].get('mode') == 'linear')

        def retreat_active(self):
            return self.route_stage == 'RETREAT'

        def room_control_errors(self, rear, settled=False):
            """Require the endpoint region AND progress along the reverse route."""
            goal = self.route_legs[self.leg_index]['points'][-1]
            xy = math.dist(rear[:2], goal[:2])
            yaw = abs(wrap_angle(rear[2]-goal[2]))
            progress = self.route_progress.update(rear[0], rear[1])
            remaining = max(0., self.route_progress.length-progress)
            tolerance = self.p['room_control_xy_tolerance_m']
            if settled:
                tolerance += self.p['room_control_settle_margin_m']
            good = (xy <= tolerance+1e-9 and remaining <= tolerance+1e-9 and
                    yaw <= self.p['room_control_heading_tolerance_rad']+1e-9)
            return good, xy, yaw, remaining, tolerance

        def finish_retreat_if_reached(self, rear=None):
            if (self.m.phase != 'NAVIGATING' or not self.retreat_active()
                    or self.route_progress is None):
                return False
            if rear is None:
                rear, problem = self.get_pose('nav_base')
                if problem or rear is None:
                    return False
            good, xy, yaw, remaining, tolerance = self.room_control_errors(rear)
            if not good:
                return False
            self.get_logger().debug(
                f'[ROOM_CONTROL_REACHED] room={self.departure_room} '
                f'control_room={self.retreat_control_room} xy_error={xy:.3f}m '
                f'yaw_error={math.degrees(yaw):.1f}deg remaining={remaining:.3f}m '
                f'xy_limit={tolerance:.3f}m')
            self.request_stop('room_control', 'room_control_reached')
            return True

        def complete_room_retreat(self):
            # Called only after terminal action cleanup and measured settling.
            rear, problem = self.get_pose('nav_base')
            if problem or rear is None:
                self.request_stop('fail', 'room_control_' + (problem or 'missing_pose'))
                return
            good, xy, yaw, remaining, tolerance = self.room_control_errors(rear, settled=True)
            self.get_logger().debug(
                f'[ROOM_HANDOFF] room={self.departure_room} '
                f'control_room={self.retreat_control_room} checks_passed={good} '
                f'xy_error={xy:.3f}m xy_limit={tolerance:.3f}m '
                f'yaw_error={math.degrees(yaw):.1f}deg '
                f'yaw_limit={math.degrees(self.p["room_control_heading_tolerance_rad"]):.1f}deg '
                f'remaining={remaining:.3f}m next_box={self.m.target}')
            if not good:
                self.request_stop('retry', 'room_control_pose_or_progress_mismatch')
                return
            self.get_logger().info(
                f'[CONTROL_REACHED] room={self.retreat_control_room} next={self.m.target}')
            self.departure_room = self.departure_colour = None
            self.retreat_control_room = None
            self.retreat_failed_controls.clear()
            self.route_stage = 'BOX'
            self.route_legs, self.leg_index = [], 0
            self.route_progress = self.selected_nav_goal = self.route_metrics = None
            self.gate.arm(self.m.target)

        def next_leg_is_linear(self):
            return (self.leg_index+1 < len(self.route_legs) and
                    self.route_legs[self.leg_index+1].get('mode') == 'linear')

        def handoff_tolerances(self, settled=False):
            if self.next_leg_is_linear():
                # The final line can start anywhere in this arrival region.
                # Require near-terminal arc, but no half-leg rule: a short
                # staging leg may already start inside the valid entry region.
                xy = self.p['final_entry_xy_tolerance_m']
                if settled:
                    # Arrival is already latched by Nav2 success or our stop
                    # request. Allow bounded pose drift during the stop.
                    xy += self.p['final_entry_settle_margin_m']
                return xy, self.p['final_entry_heading_tolerance_rad'], xy
            xy = self.p['cusp_xy_tolerance_m']
            return xy, self.p['cusp_heading_tolerance_rad'], min(xy, self.route_progress.length/2)

        def active_controller_id(self):
            if not self.route_legs:
                return 'none'
            if self.final_approach_active():
                return self.p['final_controller_id']
            return (self.p['reverse_controller_id'] if self.route_legs[self.leg_index]['gear'] < 0
                    else self.p['controller_id'])

        def navigation_timing(self, now):
            if self.retreat_active():
                return 'retreat', now-self.m.goal_ros, self.p['room_retreat_timeout_s']
            if self.final_approach_active():
                return 'final', now-self.leg_started_ros, self.p['final_approach_timeout_s']
            return 'staging', now-self.m.goal_ros, self.p['goal_timeout_s']

        def check_navigation_timeout(self, now):
            stage, elapsed, limit = self.navigation_timing(now)
            if elapsed < limit:
                return False
            self.get_logger().warning(
                f'[DEADLINE] stage={stage} elapsed={elapsed:.2f}s limit={limit:.2f}s '
                f'route_elapsed={now-self.m.goal_ros:.2f}s '
                f'mission_elapsed={now-self.m.started_ros:.2f}s '
                f'mission_limit={self.p["mission_timeout_s"]:.2f}s')
            self.request_stop('retry', 'goal_timeout')
            return True

        def send_route_leg(self):
            if self.m.phase not in ('PLANNING', 'NEED_LEG') or self.m.token is not None:
                return
            if self.ros_now()-self.m.started_ros >= self.p['mission_timeout_s']:
                self.request_stop('fail', 'mission_timeout')
                return
            if self.m.phase == 'NEED_LEG':
                # Reversals share the staging budget. Once staging has ended,
                # its stop/settle interval must not consume the final budget.
                if not self.next_leg_is_linear() and self.check_navigation_timeout(self.ros_now()):
                    return
                # Recheck after the settling interval, including heading drift.
                rear, problem = self.get_pose('nav_base')
                cusp = self.route_legs[self.leg_index]['points'][-1]
                linear_next = self.next_leg_is_linear()
                stage = 'final_entry' if linear_next else 'cusp'
                xy_tolerance, heading_tolerance, arc_tolerance = self.handoff_tolerances(settled=True)
                if problem or rear is None:
                    self.get_logger().warning(f'[HANDOFF_CHECK] stage={stage} pose_ok=False pose={problem or "missing_pose"}')
                    self.request_stop('fail' if linear_next else 'retry', problem or stage+'_missing_pose')
                    return
                xy_error = math.dist(rear[:2], cusp[:2])
                yaw_error = abs(wrap_angle(rear[2]-cusp[2]))
                progress = self.route_progress.update(rear[0], rear[1])
                remaining = max(0., self.route_progress.length-progress)
                failed = []
                if xy_error > xy_tolerance+1e-9:
                    failed.append('position')
                if yaw_error > heading_tolerance+1e-9:
                    failed.append('heading')
                if remaining > arc_tolerance+1e-9:
                    failed.append('route_progress')
                self.get_logger().debug(
                    f'[HANDOFF_CHECK] stage={stage} checks_passed={not failed} '
                    f'failed={",".join(failed) or "none"} xy_error={xy_error:.6f}m '
                    f'xy_limit={xy_tolerance:.6f}m yaw_error={math.degrees(yaw_error):.3f}deg '
                    f'yaw_limit={math.degrees(heading_tolerance):.3f}deg '
                    f'remaining={remaining:.6f}m arc_limit={arc_tolerance:.6f}m '
                    f'rear=({rear[0]:.6f},{rear[1]:.6f},{math.degrees(rear[2]):.3f}deg) '
                    f'goal=({cusp[0]:.6f},{cusp[1]:.6f},{math.degrees(cusp[2]):.3f}deg)')
                if failed:
                    suffix = 'pose_mismatch' if 'position' in failed or 'heading' in failed else 'route_progress_mismatch'
                    self.request_stop('fail' if linear_next else 'retry', stage+'_'+suffix)
                    return
                if linear_next:
                    try:
                        # Plan once from the actual stopped pose, then freeze it.
                        approach = linear_target_approach(
                            rear, self.arena.blocks[self.m.target],
                            self.p['observation_radius_m'],
                            self.route_legs[self.leg_index+1].get('minimum_straight_m', self.p['final_approach_straight_m']),
                            self.p['final_entry_heading_tolerance_rad'], self.p['final_tracking_extension_m'])
                        # Include the actual body orientation at entry, not
                        # only the ideal reference-line orientation.
                        score_route([rear]+approach['tracking_points'], self.arena)
                    except ValueError as exc:
                        self.request_stop('fail', str(exc))
                        return
                    next_leg = self.route_legs[self.leg_index+1]
                    next_leg['points'] = approach['points']
                    next_leg['path'] = self.path_with_points(next_leg['path'], approach['tracking_points'])
                    next_leg['length_m'] = approach['straight_m']
                    self.route_metrics['final_straight_m'] = approach['straight_m']
                    self.get_logger().debug(
                        f"[APPROACH] planner=linear length={approach['straight_m']:.3f}m "
                        f"entry_heading_error={math.degrees(approach['entry_heading_error_rad']):.1f}deg "
                        f"controller={self.p['final_controller_id']} path=frozen "
                        f"final_min={next_leg.get('minimum_straight_m', self.p['final_approach_straight_m']):.3f}m "
                        f"tracking_extension={self.p['final_tracking_extension_m']:.3f}m "
                        f"stop_tolerance={self.p['final_stop_tolerance_m']:.3f}m")
                    self.final_route_pub.publish(self.path_with_points(next_leg['path'], approach['points']))
                self.leg_index += 1
            leg = self.route_legs[self.leg_index]
            self.selected_nav_goal = leg['points'][-1]
            self.route_progress = RouteProgress(leg['points'])
            self.progress_mark, self.progress_ros = 0., self.ros_now()
            goal = FollowPath.Goal()
            goal.path = leg['path']
            goal.controller_id = self.active_controller_id()
            goal.goal_checker_id = (self.p['room_control_goal_checker_id'] if self.retreat_active() else
                                    self.p['alignment_goal_checker_id'] if self.next_leg_is_linear() else
                                    self.p['cusp_goal_checker_id'] if self.leg_index+1 < len(self.route_legs) else
                                    self.p['goal_checker_id'])
            self.leg_started_ros = self.ros_now()
            token = self.m.sending(self.leg_started_ros, time.monotonic())
            self.nav_goal_handle, self.cancel_sent = None, False
            self.cmd_token = None
            self.active_route_pub.publish(goal.path)
            self.get_logger().info(
                f'[DRIVE] target={self.m.target} stage={leg.get("mode", "staging")} '
                f'direction={"forward" if leg["gear"] > 0 else "reverse"} '
                f'length={leg["length_m"]:.2f}m')
            stage, elapsed, limit = self.navigation_timing(self.leg_started_ros)
            self.get_logger().debug(
                f"[LEG] target={self.m.target} index={self.leg_index+1}/{len(self.route_legs)} "
                f"mode={leg.get('mode', 'staging')} controller={goal.controller_id} "
                f"direction={'forward' if leg['gear'] > 0 else 'reverse'} length={leg['length_m']:.3f}m "
                f"timer={stage} elapsed={elapsed:.2f}s limit={limit:.2f}s "
                f"mission_remaining={max(0., self.p['mission_timeout_s']-(self.leg_started_ros-self.m.started_ros)):.2f}s")
            try:
                future = self.nav.send_goal_async(goal)
                future.add_done_callback(lambda f, tag=token: self.on_goal_response(tag, f))
            except Exception as exc:
                self.m.lock('goal_send_error_' + str(exc))

        def cancel_planning(self):
            self.plan_batch = None
            request = self.plan_request
            if request is not None and request['handle'] is not None and not request['cancel_sent']:
                request['cancel_sent'] = True
                try:
                    request['handle'].cancel_goal_async()
                except Exception as exc:
                    self.get_logger().error('[PLAN_CANCEL_ERROR] ' + str(exc))

        def on_goal_response(self, token, future):
            try:
                handle = future.result()
            except Exception as exc:
                if token == self.m.token:
                    self.m.lock('goal_response_error_' + str(exc))
                return
            current = self.m.accepted(token, handle.accepted, time.monotonic())
            if not handle.accepted:
                if current and self.final_approach_active() and self.m.outcome == 'retry':
                    self.request_stop('fail', 'linear_goal_rejected')
                return
            if not current:
                try:
                    handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().error('[CANCEL_ERROR] ' + str(exc))
                return
            self.nav_goal_handle = handle
            try:
                handle.get_result_async().add_done_callback(lambda f, tag=token: self.on_nav_result(tag, f))
            except Exception as exc:
                self.request_stop('fail', 'navigation_result_registration_error_' + str(exc))
                return
            if self.m.phase in ('STOPPING', 'LOCKED'):
                self.cancel_current_goal()

        def on_nav_result(self, token, future):
            try:
                response = future.result()
                status = response.status
            except Exception as exc:
                if token == self.m.token:
                    self.m.lock('navigation_result_error_' + str(exc))
                return
            label = {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                     GoalStatus.STATUS_CANCELED: 'cancelled',
                     GoalStatus.STATUS_ABORTED: 'aborted'}.get(status, 'unknown')
            if self.m.navigation_result(token, label, self.ros_now(), time.monotonic()):
                self.nav_goal_handle = None
                self.cmd_token = None
                # Action result fields vary across Nav2 releases. Log whatever
                # the server supplied, without guessing the cause of an abort.
                result = getattr(response, 'result', None)
                detail = ''
                for field in ('error_code', 'error_msg'):
                    value = getattr(result, field, None)
                    if value is not None and value != '':
                        detail += f' {field}={value}'
                self.get_logger().debug(
                    '[NAV_RESULT] ' + label + ' reason=' + (self.m.reason or 'none') + detail)
                if self.retreat_active() and self.m.phase == 'OBSERVING':
                    # Never observe/advance a colour at a room control point.
                    # The actual arrival pose/progress is checked after settling.
                    self.request_stop('room_control', 'room_control_reached')
                elif self.final_approach_active() and self.m.phase == 'STOPPING' and self.m.outcome == 'retry':
                    self.request_stop('fail', 'linear_navigation_' + label)
                elif self.m.phase == 'OBSERVING' and self.leg_index+1 < len(self.route_legs):
                    self.request_stop('next_leg', 'alignment_reached' if self.next_leg_is_linear() else 'direction_change')
                elif self.m.phase == 'OBSERVING' and not self.p['final_approach_enabled']:
                    # A direct Dubins viewing goal is not a FOUND event. Begin
                    # the camera observation window only after measured stopping.
                    self.request_stop('observe', 'viewpoint_reached')

        def cancel_current_goal(self):
            if self.nav_goal_handle is None or self.cancel_sent:
                return
            self.cancel_sent = True
            try:
                # A cancel acknowledgement is NOT a terminal result; keep waiting
                # on get_result_async before releasing the next mission goal.
                self.nav_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().error('[CANCEL_ERROR] ' + str(exc))

        def request_stop(self, outcome, reason):
            # Once committed, a failed approach stops instead of launching a
            # new staging manoeuvre beside the box.
            if outcome == 'retry' and self.final_approach_active() and self.m.phase in ('SENDING', 'NAVIGATING', 'OBSERVING', 'STOPPING'):
                outcome, reason = 'fail', 'linear_' + reason
            previous = (self.m.phase, self.m.outcome, self.m.reason)
            self.m.stop(outcome, reason, time.monotonic())
            if previous != (self.m.phase, self.m.outcome, self.m.reason):
                log = (self.get_logger().warning if self.m.outcome in ('retry', 'fail')
                       else self.get_logger().debug)
                log(
                    f'[STOP] target={self.m.target} outcome={self.m.outcome} '
                    f'reason={self.m.reason or "none"} confirmation={self.gate.state}')
            self.cancel_planning()
            self.cmd_token = None
            self.cmd_pub.publish(Twist())
            self.last_motion_output = (0., 0.)
            self.last_motion_gate = 'stopping'
            self.cancel_current_goal()

        def on_detection(self, msg):
            self.last_detection_message = msg.data[:120]
            try:
                self.visible = parse_colours(msg.data)
                self.last_detection_error = None
            except ValueError as exc:
                self.last_detection_error = str(exc)
                self.visible.clear()
                self.gate.invalidate('invalid_detection_message')
                return
            self.detection_wall = time.monotonic()
            if self.retreat_active():
                # Camera freshness still matters, but a nearby next colour
                # cannot skip the required reverse-to-control stage.
                self.gate.invalidate('room_retreat')
                return
            if not (self.m.phase in ('NEED_GOAL', 'PLANNING', 'NEED_LEG', 'NAVIGATING', 'OBSERVING')
                    or (self.m.phase == 'STOPPING' and self.m.outcome in ('next_leg', 'observe'))):
                return
            pose, reason = self.get_pose()
            # Assignment C1-C3: camera detection, base_link proximity and log.
            # Heading guides navigation but is not an extra FOUND condition.
            event = self.gate.observe(self.visible, None if pose is None else pose[:2],
                                      self.ros_now(), self.detection_wall, reason)
            if event is None or not self.m.confirm(event['colour'], self.detection_wall):
                return
            # Persists across separate commands as well as a multi-colour list.
            # Remain at this box until the next target is requested.
            self.departure_room = self.arena.block_rooms[event['colour']]
            self.departure_colour = event['colour']
            self.retreat_attempts = 0
            self.retreat_control_room = None
            self.retreat_failed_controls.clear()
            event['mission_id'] = self.m.mission_id
            event['command_index'] = self.m.index + 1
            event['elapsed_s'] = max(0., self.ros_now()-self.m.started_ros)
            self.found_pub.publish(String(data=json.dumps(event, allow_nan=False)))
            self.get_logger().info(
                f"[FOUND] colour={event['colour']} t={event['elapsed_s']:.2f}s "
                f"x={event['base_x']:.3f} y={event['base_y']:.3f} distance={event['distance_m']:.3f}m")
            self.cmd_token = None
            self.cmd_pub.publish(Twist())
            self.last_motion_output = (0., 0.)
            self.last_motion_gate = 'confirmed'
            self.cancel_planning()
            self.cancel_current_goal()

        def on_nav_cmd(self, msg):
            self.raw_nav_cmd = (msg.linear.x, msg.angular.z)
            self.raw_nav_cmd_wall = time.monotonic()
            if self.m.phase != 'NAVIGATING':
                return
            try:
                extra = (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)
                if any(not math.isfinite(x) or abs(x) > 1e-6 for x in extra):
                    raise ValueError('unsupported_axis')
                self.cmd = bounded_velocity(msg.linear.x, msg.angular.z,
                                             self.p['final_approach_speed_m_s'] if self.final_approach_active() else self.p['max_speed_m_s'],
                                             self.controller_minimum_radius_m)
            except ValueError:
                self.request_stop('fail', 'invalid_navigation_velocity')
                return
            # A lookahead point just behind an already-reached cusp can make
            # RPP request the opposite sign. Complete that leg before treating
            # the command as a tracking fault; never forward it on this goal.
            if (self.finish_retreat_if_reached() or self.check_final_tracking()
                    or self.finish_direction_leg_if_reached()):
                return
            if self.route_legs and self.cmd[0]*self.route_legs[self.leg_index]['gear'] < -1e-6:
                # Suppress tiny opposite-sign command residuals, never reverse them.
                # Sustained zero motion still fails the normal progress timeout.
                if abs(self.cmd[0]) <= self.p['direction_deadband_m_s']:
                    self.cmd = (0., 0.)
                    self.cmd_wall, self.cmd_token = time.monotonic(), self.m.token
                    self.cmd_pub.publish(Twist())
                    self.last_motion_output, self.last_motion_gate = (0., 0.), 'direction_deadband'
                    return
                self.get_logger().warning(
                    f"[DIRECTION_MISMATCH] controller={self.active_controller_id()} "
                    f"expected={'forward' if self.route_legs[self.leg_index]['gear'] > 0 else 'reverse'} "
                    f"nav_v={msg.linear.x:.4f} nav_w={msg.angular.z:.4f}")
                self.request_stop('retry', 'controller_direction_mismatch')
                return
            self.cmd_wall, self.cmd_token = time.monotonic(), self.m.token

        def finish_direction_leg_if_reached(self, rear=None, progress=None):
            """Stop at a reached intermediate cusp, then use normal cancel/settle.

            Humble RPP zeros curvature inside sqrt(.001) m of its carrot. A
            1 cm arrival tolerance can therefore leave a small lateral miss
            that flips its velocity sign. Require pose AND near-terminal arc
            progress; proximity alone must not skip a loop or a short leg.
            This never treats arrival at a viewing goal as colour confirmation.
            """
            if (self.m.phase != 'NAVIGATING' or self.route_progress is None or
                    self.leg_index+1 >= len(self.route_legs)):
                return False
            if rear is None:
                rear, problem = self.get_pose('nav_base')
                if problem or rear is None:
                    return False
            leg = self.route_legs[self.leg_index]
            goal = leg['points'][-1]
            if progress is None:
                progress = self.route_progress.update(rear[0], rear[1])
            remaining = max(0., self.route_progress.length-progress)
            xy_error = math.dist(rear[:2], goal[:2])
            yaw_error = abs(wrap_angle(rear[2]-goal[2]))
            tolerance, heading_tolerance, arc_tolerance = self.handoff_tolerances()
            if (xy_error > tolerance or yaw_error > heading_tolerance or
                    remaining > arc_tolerance+1e-9):
                return False
            if self.next_leg_is_linear():
                bx, by = self.arena.blocks[self.m.target]
                # The line is anchored at the measured stop, so check its
                # bearing as well as the nominal staging endpoint's yaw.
                bearing_error = abs(wrap_angle(math.atan2(by-rear[1], bx-rear[0])-rear[2]))
                if bearing_error > heading_tolerance:
                    return False
            event_name = 'ALIGNMENT_REACHED' if self.next_leg_is_linear() else 'CUSP_REACHED'
            self.get_logger().debug(
                f'[{event_name}] leg={self.leg_index+1}/{len(self.route_legs)} '
                f'xy_error={xy_error:.3f}m yaw_error={math.degrees(yaw_error):.1f}deg '
                f'remaining={remaining:.3f}m xy_limit={tolerance:.3f}m '
                f'yaw_limit={math.degrees(heading_tolerance):.1f}deg')
            self.request_stop('next_leg', 'alignment_reached' if self.next_leg_is_linear() else 'cusp_reached')
            return True

        def check_final_tracking(self, rear=None):
            if self.m.phase != 'NAVIGATING' or not self.final_approach_active():
                return False
            if rear is None:
                rear, problem = self.get_pose('nav_base')
                if problem or rear is None:
                    self.request_stop('fail', 'linear_' + (problem or 'missing_pose'))
                    return True
            points = self.route_legs[self.leg_index]['points']
            start, end = points[0], points[-1]
            c, s = math.cos(start[2]), math.sin(start[2])
            dx, dy = rear[0]-start[0], rear[1]-start[1]
            lateral = -dx*s+dy*c
            along = dx*c+dy*s
            yaw_error = abs(wrap_angle(rear[2]-start[2]))
            if (abs(lateral) > self.p['final_cross_track_limit_m'] or
                    yaw_error > self.p['final_heading_limit_rad']):
                self.get_logger().warning(
                    f'[FINAL_TRACKING_ERROR] lateral={lateral:.4f}m '
                    f'cross_limit={self.p["final_cross_track_limit_m"]:.4f}m '
                    f'heading_error={math.degrees(yaw_error):.2f}deg '
                    f'heading_limit={math.degrees(self.p["final_heading_limit_rad"]):.2f}deg')
                self.request_stop('fail', 'linear_tracking_error')
                return True
            if along > math.dist(start[:2], end[:2])+.02:
                self.request_stop('fail', 'linear_endpoint_passed')
                return True
            remaining = math.dist(start[:2], end[:2])-along
            if remaining <= self.p['final_stop_tolerance_m']:
                # Stop at the viewing plane even with residual yaw. Do not let
                # the controller drive toward the lookahead-only reference tail.
                self.get_logger().debug(
                    f'[FINAL_STOP] remaining={remaining:.4f}m lateral={lateral:.4f}m '
                    f'heading_error={math.degrees(yaw_error):.2f}deg '
                    f'confirmation={self.gate.state}')
                self.request_stop('observe', 'linear_endpoint_reached')
                return True
            return False

        def on_tick(self):
            wall, now = time.monotonic(), self.ros_now()
            if self.m.active and now < self.last_ros:
                self.request_stop('fail', 'simulation_clock_reset')
            self.last_ros = now
            problem = self.sensor_problem(wall)
            if self.m.active and self.m.phase != 'STOPPING':
                if problem:
                    if self.unhealthy_since is None:
                        self.unhealthy_since = wall
                    self.gate.invalidate(problem)
                    if wall-self.unhealthy_since >= self.p['sensor_failure_timeout_s']:
                        self.request_stop('fail', problem)
                else:
                    self.unhealthy_since = None
                if now-self.m.started_ros >= self.p['mission_timeout_s']:
                    self.request_stop('fail', 'mission_timeout')
                elif self.m.phase == 'SENDING' and wall-self.m.goal_wall >= self.p['goal_response_timeout_s']:
                    self.request_stop('fail', 'goal_response_timeout')
                elif self.m.phase == 'NAVIGATING':
                    self.check_navigation_timeout(now)
                elif self.m.phase == 'OBSERVING' and now-self.m.observe_ros >= self.p['observation_timeout_s']:
                    self.request_stop('retry', 'camera_not_confirmed_at_goal')

            if (self.m.phase == 'PLANNING' and self.plan_request is not None
                    and wall-self.plan_request['sent'] >= self.p['planning_result_timeout_s']):
                self.request_stop('fail', 'planner_result_timeout')
            if self.m.phase == 'NAVIGATING' and not problem and self.route_progress is not None:
                rear, rear_problem = self.get_pose('nav_base')
                if rear_problem is None:
                    progress = self.route_progress.update(rear[0], rear[1])
                    if (not self.finish_retreat_if_reached(rear) and not self.check_final_tracking(rear)
                            and not self.finish_direction_leg_if_reached(rear, progress)):
                        if progress-self.progress_mark >= self.p['route_progress_min_m']:
                            self.progress_mark, self.progress_ros = progress, now
                        elif now-self.progress_ros >= self.p['route_progress_timeout_s']:
                            self.request_stop('retry', 'no_progress_along_selected_route')

            if self.m.phase == 'STOPPING':
                self.cancel_planning()
                self.cancel_current_goal()
                stopped = (self.odom_wall is not None and wall-self.odom_wall <= self.p['sensor_freshness_s']
                           and self.odom_speed < .02 and self.odom_yaw_rate < .10)
                if not stopped:
                    self.stopped_since = None
                elif self.stopped_since is None:
                    self.stopped_since = wall
                if self.m.token is None and self.plan_request is None and self.stopped_since is not None and wall-self.stopped_since >= self.p['stop_settle_s']:
                    if self.m.outcome == 'room_control':
                        self.complete_room_retreat()
                    if (self.retreat_active() and self.m.outcome == 'retry'
                            and self.retreat_control_room is not None):
                        # Retry only after the old action is terminal and the
                        # robot has stopped, including controller aborts/rejections.
                        self.retreat_failed_controls.add(self.retreat_control_room)
                        self.get_logger().warning(
                            f'[RETREAT_FALLBACK] departure_room={self.departure_room} '
                            f'failed_control={self.retreat_control_room} reason={self.m.reason}')
                    self.m.settled(now)
                    self.stopped_since = None
                elif wall-self.m.stop_wall >= self.p['cancel_timeout_s']:
                    self.m.lock((self.m.reason or 'confirmation_stop') + '_stop_or_cancel_unconfirmed')
            else:
                self.stopped_since = None

            if self.m.phase == 'NEED_GOAL' and not problem:
                self.send_next_goal()
            if self.m.phase == 'NEED_LEG' and not problem:
                self.send_route_leg()
            output = Twist()
            if (self.m.phase == 'NAVIGATING' and not problem and
                    self.cmd_token == self.m.token and wall-self.cmd_wall <= self.p['cmd_timeout_s']):
                output.linear.x, output.angular.z = self.cmd
                motion_gate = 'forwarding'
            elif self.m.phase != 'NAVIGATING':
                motion_gate = self.m.phase.lower()
            elif problem:
                motion_gate = problem
            elif self.cmd_token != self.m.token:
                motion_gate = 'waiting_for_goal_velocity'
            else:
                motion_gate = 'stale_navigation_velocity'
            self.cmd_pub.publish(output)
            self.last_motion_output = (output.linear.x, output.angular.z)
            self.last_motion_gate = motion_gate
            self.publish_status()

        def on_health(self):
            if self.m.active:
                self.log_confirmation()
                self.log_motion()
            for name, client in self.lifecycle_clients.items():
                if name in self.lifecycle_pending or not client.service_is_ready():
                    continue
                self.lifecycle_pending.add(name)
                future = client.call_async(GetState.Request())
                future.add_done_callback(lambda f, n=name: self.on_lifecycle(n, f))
            if self.m.can_start:
                problem = self.readiness()
                if problem != self.ready_logged:
                    self.get_logger().info('[WAIT] ' + problem if problem else '[READY] Enter a command: find red and blue')
                    self.ready_logged = problem
            if self.m.active and self.count_publishers(self.p['output_cmd_topic']) != 1:
                self.request_stop('fail', 'multiple_cmd_vel_publishers')
            if self.m.active:
                wall = time.monotonic()
                if any(state != 3 or wall-stamp > 4. for state, stamp in self.lifecycle.values()):
                    self.request_stop('fail', 'navigation_lifecycle_inactive')

        def on_lifecycle(self, name, future):
            self.lifecycle_pending.discard(name)
            try:
                self.lifecycle[name] = (future.result().current_state.id, time.monotonic())
            except Exception:
                self.lifecycle[name] = (0, time.monotonic())

        def log_confirmation(self):
            """Read-only diagnostics; never advance or reset the confirmation gate."""
            if not self.get_logger().is_enabled_for(logging.DEBUG):
                return
            target = self.m.target
            if target is None:
                return
            wall, now = time.monotonic(), self.ros_now()
            pose, pose_problem = self.get_pose()
            bx, by = self.arena.blocks[target]
            distance = None if pose is None else math.hypot(pose[0]-bx, pose[1]-by)
            age = None if self.detection_wall is None else max(0., wall-self.detection_wall)
            # A cached visible colour is not continuing camera evidence.
            fresh = age is not None and age <= self.p['sensor_freshness_s']
            hold = (0. if self.gate.support_start is None else
                    max(0., (self.gate.last_ros or now)-self.gate.support_start))
            dist_text = 'unknown' if distance is None else f'{distance:.3f}m'
            age_text = 'never' if age is None else f'{age:.3f}s'
            base_text = 'unknown' if pose is None else f'({pose[0]:.3f},{pose[1]:.3f})'
            visible = ','.join(sorted(self.visible)) or 'none'
            heading = ('unknown' if pose is None else
                       f'{math.degrees(wrap_angle(math.atan2(by-pose[1], bx-pose[0])-pose[2])):.1f}deg')
            self.get_logger().debug(
                f'[CONFIRM] target={target} phase={self.m.phase} stage={self.route_stage} state={self.gate.state} '
                f'distance={dist_text} limit={self.gate.radius:.3f}m '
                f'visible={visible} fresh={fresh} detection_age={age_text} '
                f'hold={hold:.2f}/{self.gate.hold:.2f}s pose={pose_problem or "ok"} '
                f'base={base_text} block=({bx:.3f},{by:.3f}) heading_error={heading} '
                f'heading_gate=disabled '
                f'raw={json.dumps(self.last_detection_message)} '
                f'parse_error={json.dumps(self.last_detection_error)}')

        def log_motion(self):
            """Compare requested/output motion with odometry without changing control."""
            if not self.get_logger().is_enabled_for(logging.DEBUG):
                return
            wall = time.monotonic()
            age = None if self.raw_nav_cmd_wall is None else max(0., wall-self.raw_nav_cmd_wall)
            odom_age = None if self.odom_wall is None else max(0., wall-self.odom_wall)
            rear, rear_problem = self.get_pose('nav_base')
            rear_text = ('unknown' if rear is None else
                         f'({rear[0]:.3f},{rear[1]:.3f},{math.degrees(rear[2]):.1f}deg)')
            goal = self.selected_nav_goal
            goal_text = ('none' if goal is None else
                         f'({goal[0]:.3f},{goal[1]:.3f},{math.degrees(goal[2]):.1f}deg)')
            error = None if rear is None or goal is None else math.dist(rear[:2], goal[:2])
            error_text = 'unknown' if error is None else f'{error:.3f}m'
            age_text = 'never' if age is None else f'{age:.3f}s'
            odom_age_text = 'never' if odom_age is None else f'{odom_age:.3f}s'
            endpoint_text = 'goal_body=unknown'
            if rear is not None and goal is not None:
                dx, dy = goal[0]-rear[0], goal[1]-rear[1]
                c, s = math.cos(rear[2]), math.sin(rear[2])
                endpoint_text = (f'goal_forward={c*dx+s*dy:.3f}m goal_lateral={-s*dx+c*dy:.3f}m '
                                 f'goal_yaw_error={math.degrees(wrap_angle(goal[2]-rear[2])):.1f}deg')
            raw_v, raw_w = self.raw_nav_cmd
            requested_radius = 'straight' if abs(raw_w) < 1e-6 else f'{abs(raw_v/raw_w):.3f}m'
            radius_limited = abs(raw_w) > abs(raw_v)/self.controller_minimum_radius_m+1e-4
            self.get_logger().debug(
                f'[MOTION] phase={self.m.phase} stage={self.route_stage} '
                f'leg={self.leg_index+1}/{len(self.route_legs)} gate={self.last_motion_gate} '
                f'controller={self.active_controller_id()} '
                f'nav_v={self.raw_nav_cmd[0]:.3f} nav_w={self.raw_nav_cmd[1]:.3f} '
                f'sent_v={self.last_motion_output[0]:.3f} sent_w={self.last_motion_output[1]:.3f} '
                f'controller_radius_limit={self.controller_minimum_radius_m:.4f}m '
                f'controller_max_steering={self.p["controller_max_steering_deg"]:.2f}deg '
                f'requested_radius={requested_radius} radius_limited={radius_limited} '
                f'odom_speed={self.odom_speed:.3f} nav_age={age_text} odom_age={odom_age_text} '
                f'odom_vx={self.odom_vx:.3f} odom_wz={self.odom_wz:.3f} odom_frame={self.odom_child_frame} '
                f'rear={rear_text} goal={goal_text} goal_error={error_text} '
                f'{endpoint_text} '
                f'pose={rear_problem or "ok"}')

        def publish_status(self):
            status = {'mission_id': self.m.mission_id, 'phase': self.m.phase,
                      'colours': self.m.colours, 'found': self.m.found, 'target': self.m.target,
                      'attempt': len(self.tried), 'reason': self.m.reason,
                      'visible_colours': sorted(self.visible), 'confirmation_state': self.gate.state,
                      'distance_m': None if self.gate.distance is None else round(self.gate.distance, 3),
                      'selected_route': self.route_metrics,
                      'departure_room': self.departure_room, 'departure_colour': self.departure_colour,
                      'retreat_attempt': self.retreat_attempts,
                      'retreat_control_room': self.retreat_control_room,
                      'final_approach_enabled': self.p['final_approach_enabled'],
                      'navigation_stage': ('ROOM_RETREAT' if self.retreat_active() else
                                           'FINAL_LINEAR' if self.final_approach_active() else 'BOX_ROUTE')}
            text = json.dumps(status)
            if text != self.last_status:
                self.status_pub.publish(String(data=text))
                self.last_status = text
            terminal_key = (self.m.mission_id, self.m.phase)
            if self.m.phase in ('SUCCEEDED', 'FAILED', 'LOCKED') and self.reported_terminal != terminal_key:
                if self.m.phase in ('FAILED', 'LOCKED'):
                    self.log_confirmation()
                success = self.m.phase == 'SUCCEEDED'
                self.get_logger().info('[MISSION] status=' + ('SUCCESS' if success else 'FAIL') +
                                       ('' if success else ' reason=' + self.m.reason))
                self.reported_terminal = terminal_key

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    stopping = False

    def stop_signal(signum, frame):
        nonlocal stopping
        stopping = True

    handlers = {sig: signal.signal(sig, stop_signal) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        node = Task3MissionNode()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        if node is not None and rclpy.ok():
            node.destroy_subscription(node.command_sub)
            node.destroy_subscription(node.speech_sub)
            node.tick_timer.cancel()
            node.health_timer.cancel()
            if node.m.active:
                node.request_stop('fail', 'shutdown')
                node.m.lock('shutdown')
                node.publish_status()
            deadline = time.monotonic()+.5
            while rclpy.ok() and time.monotonic() < deadline:
                node.cmd_pub.publish(Twist())
                node.cancel_current_goal()
                rclpy.spin_once(node, timeout_sec=.05)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, old in handlers.items():
            signal.signal(sig, old)


if __name__ == '__main__':
    main()
