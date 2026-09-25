#!/usr/bin/env python3
# Task 3 owner: Mohammad Asif Bin Abdul Sahid (A0313732M), Group 13.
"""EE5112: connect the existing camera detector to proximity confirmation.

Development milestone only: observes the robot; does not send navigation goals.
Developed with AI coding assistance.
"""
import json
import math
import time


COLOURS = ('Red', 'Orange', 'Yellow', 'Green', 'Blue', 'Purple', 'Black')
BLOCKS = {
    'Red': (0.45, 1.70), 'Orange': (0.95, 2.00),
    'Yellow': (1.80, 1.85), 'Green': (2.35, 2.05),
    'Blue': (3.45, 2.15), 'Purple': (3.70, 2.00), 'Black': (3.55, 1.45),
}


def parse_colours(text):
    """Exact parser for the current detector's String contract; fail closed."""
    if text.strip().lower() in ('', 'none'):
        return set()
    names = [part.strip().title() for part in text.split(',')]
    if any(name not in COLOURS for name in names):
        raise ValueError('Expected comma-separated colour names, or None.')
    return set(names)


class ConfirmationGate:
    """Evaluate only NEW detector messages, never replay a cached detection."""

    def __init__(self, blocks=None, radius=0.50, hold=0.30, gap=0.50):
        self.blocks = dict(BLOCKS if blocks is None else blocks)
        if not (math.isfinite(radius) and 0 < radius <= 0.50):
            raise ValueError('confirmation_radius_m must be in (0, 0.50].')
        if not (math.isfinite(hold) and math.isfinite(gap) and hold > 0 and gap > 0):
            raise ValueError('Timing parameters must be finite and positive.')
        for name in COLOURS:
            xy = self.blocks[name]
            if len(xy) != 2 or not all(math.isfinite(v) for v in xy):
                raise ValueError('Each block position must contain two finite coordinates.')
        self.radius, self.hold, self.gap = radius, hold, gap
        self.target = ''
        self.revision = 0
        self.arm('')

    def arm(self, target):
        target = target.strip().title()
        if target == 'None':
            target = ''
        if target and target not in COLOURS:
            raise ValueError('Select one of the seven colours, or None to disarm.')
        self.target = target
        self.revision += 1
        self.confirmed = False
        self.last_ros = None
        self.last_wall = None
        self.support_start = None
        self.distance = None
        self.state = 'waiting_for_detection' if target else 'disarmed'

    def invalidate(self, reason):
        self.support_start = None
        self.distance = None
        if not self.confirmed:
            self.state = reason if self.target else 'disarmed'

    def observe(self, colours, xy, ros_now, wall_now, pose_problem=None):
        if not all(math.isfinite(t) for t in (ros_now, wall_now)):
            self.invalidate('invalid_time')
            return None
        if self.last_ros is not None and ros_now < self.last_ros:
            # Restarted simulation needs a deliberate new target selection.
            self.arm('')
            self.state = 'clock_reset_rearm_target'
            return None
        if not self.target or self.confirmed:
            return None
        if self.last_wall is not None and wall_now - self.last_wall > self.gap:
            self.support_start = None
        if self.last_ros is not None and ros_now - self.last_ros > self.gap:
            self.support_start = None
        self.last_ros, self.last_wall = ros_now, wall_now
        if ros_now <= 0:
            self.invalidate('waiting_for_clock')
            return None
        if pose_problem or xy is None or not all(math.isfinite(v) for v in xy):
            self.invalidate(pose_problem or 'no_pose')
            return None
        bx, by = self.blocks[self.target]
        self.distance = math.hypot(xy[0] - bx, xy[1] - by)
        if self.target not in colours:
            self.support_start = None
            self.state = 'target_not_visible'
            return None
        if self.distance > self.radius:
            self.support_start = None
            self.state = 'visible_but_too_far'
            return None
        if self.support_start is None:
            self.support_start = ros_now
        self.state = 'confirming'
        if ros_now - self.support_start < self.hold:
            return None
        self.confirmed = True
        self.state = 'confirmed'
        return {
            'colour': self.target, 'selection_id': self.revision,
            'sim_time_s': ros_now, 'base_x': xy[0], 'base_y': xy[1],
            'block_x': bx, 'block_y': by, 'distance_m': self.distance,
            'distance_source': 'map_TF_base_link_and_configured_block_centre',
            'camera_evidence': 'fresh_detector_colour_names',
        }


def main(args=None):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from rcl_interfaces.msg import ParameterDescriptor
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException

    class ColourConfirmationNode(Node):
        def __init__(self):
            super().__init__('colour_confirmation')
            defaults = {
                'target_colour': 'Red',
                'detections_topic': '/detected_colours',
                'target_topic': '/colour_confirmation/target',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'confirmation_radius_m': 0.50,
                'confirmation_hold_s': 0.30,
                'detection_timeout_s': 0.50,
                'tf_max_age_s': 0.40,
                'tf_future_tolerance_s': 0.20,
            }
            descriptor = ParameterDescriptor(read_only=True)
            for key, value in defaults.items():
                self.declare_parameter(key, value, descriptor)
            for colour, xy in BLOCKS.items():
                self.declare_parameter('blocks.' + colour.lower(), list(xy), descriptor)
            self.p = {key: self.get_parameter(key).value for key in defaults}
            for key in ('tf_max_age_s', 'tf_future_tolerance_s'):
                if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                    raise ValueError(key + ' must be finite and positive.')
            if self.p['base_frame'] != 'base_link':
                raise ValueError('The assignment measures proximity from base_link, not nav_base.')
            blocks = {c: self.get_parameter('blocks.' + c.lower()).value for c in COLOURS}
            self.gate = ConfirmationGate(blocks, self.p['confirmation_radius_m'],
                                         self.p['confirmation_hold_s'], self.p['detection_timeout_s'])
            self.gate.arm(self.p['target_colour'])
            self.buffer = Buffer(cache_time=Duration(seconds=5.0))
            self.listener = TransformListener(self.buffer, self)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            self.found_pub = self.create_publisher(String, '/colour_confirmation/found', qos)
            self.status_pub = self.create_publisher(String, '/colour_confirmation/status', qos)
            self.sub = self.create_subscription(String, self.p['detections_topic'], self.on_detection, qos)
            self.target_sub = self.create_subscription(String, self.p['target_topic'], self.on_target, qos)
            self.last_received = None
            self.visible = set()
            self.last_log_state = None
            self.start_ros = self.get_clock().now().nanoseconds / 1e9
            self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(0.5, self.on_health, clock=self.wall_clock)
            self.get_logger().info(
                '[CONFIRM_READY] target=' + (self.gate.target or 'None') +
                ' input=' + self.p['detections_topic'] +
                ' proximity_frame=base_link; development confirmation only')

        def on_target(self, msg):
            try:
                self.gate.arm(msg.data)
            except ValueError as exc:
                self.get_logger().error('[TARGET_REJECTED] ' + str(exc))
                return
            self.start_ros = self.get_clock().now().nanoseconds / 1e9
            self.visible.clear()
            self.last_received = None
            self.get_logger().info('[TARGET] colour=' + (self.gate.target or 'None'))
            self.publish_status()

        def get_pose(self, now):
            try:
                transform = self.buffer.lookup_transform(
                    self.p['map_frame'], self.p['base_frame'], Time())
            except TransformException:
                return None, 'waiting_for_map_tf'
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
            age = now - stamp
            if stamp <= 0 or age > self.p['tf_max_age_s']:
                return None, 'stale_map_tf'
            if age < -self.p['tf_future_tolerance_s']:
                return None, 'future_map_tf'
            t = transform.transform.translation
            return (t.x, t.y), None

        def on_detection(self, msg):
            wall_now = time.monotonic()
            self.last_received = wall_now
            try:
                self.visible = parse_colours(msg.data)
            except ValueError:
                self.visible = set()
                self.gate.invalidate('invalid_detection_message')
                self.publish_status()
                return
            now = self.get_clock().now().nanoseconds / 1e9
            xy, problem = self.get_pose(now)
            event = self.gate.observe(self.visible, xy, now, wall_now, problem)
            if event is not None:
                event['elapsed_s'] = max(0.0, now - self.start_ros)
                self.found_pub.publish(String(data=json.dumps(event, allow_nan=False)))
                self.get_logger().info(
                    f"[FOUND] colour={event['colour']} t={event['elapsed_s']:.2f}s "
                    f"x={event['base_x']:.3f} y={event['base_y']:.3f} "
                    f"distance={event['distance_m']:.3f}m")
            self.publish_status()

        def on_health(self):
            if self.last_received is None or time.monotonic() - self.last_received > self.gate.gap:
                self.visible.clear()
                self.gate.invalidate('waiting_for_fresh_detections')
            self.publish_status()

        def publish_status(self):
            age = None if self.last_received is None else time.monotonic() - self.last_received
            status = {'target': self.gate.target or None, 'state': self.gate.state,
                      'visible_colours': sorted(self.visible),
                      'distance_m': self.gate.distance, 'confirmed': self.gate.confirmed,
                      'detection_receipt_age_s': age, 'selection_id': self.gate.revision}
            self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
            state = (self.gate.target, self.gate.state)
            if state != self.last_log_state:
                self.get_logger().info('[CONFIRM] ' + json.dumps(status, allow_nan=False))
                self.last_log_state = state

    rclpy.init(args=args)
    node = None
    try:
        node = ColourConfirmationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
