#!/usr/bin/env python3
# Task 3 owner: Mohammad Asif Bin Abdul Sahid (A0313732M), Group 13.
"""Type an English command for Task 3; the mission starts automatically."""
import select
import sys
import time


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('task3_terminal')
    pub = node.create_publisher(String, '/task3/command', 10)
    print('Examples: find red | find red and blue | find the red block and the blue block')
    print('cancel stops the current mission. q cancels and exits. Wait for [READY] in the mission terminal.')
    try:
        print('Task 3> ', end='', flush=True)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            readable, _, _ = select.select([sys.stdin], [], [], 0)
            if not readable:
                continue
            line = sys.stdin.readline()
            if not line or line.strip().lower() in ('q', 'quit', 'exit'):
                break
            command = line.strip()
            if not command:
                print('[CMD_REJECTED] reason=Empty command. Use English, e.g. find red and blue.')
            elif pub.get_subscription_count() == 0:
                print('Mission node is not connected. Start task3.launch.py first.')
            else:
                pub.publish(String(data=command))
            print('Task 3> ', end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            pub.publish(String(data='cancel'))
            deadline = time.monotonic()+.4
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
