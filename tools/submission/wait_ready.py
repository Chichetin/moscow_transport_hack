"""Wait until every TOPIC has at least one subscriber in the ROS graph (judge_runner_inside.sh).

    python3 wait_ready.py TIMEOUT_S TOPIC [TOPIC ...]

Prints the seconds it waited; exit 0 when all are subscribed, 1 on timeout (the topics still
without a subscriber are printed). A subscriber in the graph is the endpoint only: the node
owning it has run its constructor, which is what bag play needs to be received.
"""
import sys
import time

import rclpy


def main(argv) -> int:
    if len(argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2
    timeout, topics = float(argv[1]), argv[2:]
    rclpy.init()
    node = rclpy.create_node('judge_runner_wait_ready')
    start = time.monotonic()
    try:
        while True:
            missing = [t for t in topics if node.count_subscribers(t) < 1]
            waited = time.monotonic() - start
            if not missing:
                print(f'ready {waited:.2f} s')
                return 0
            if waited > timeout:
                print(f'timeout {waited:.2f} s, no subscriber: {" ".join(missing)}')
                return 1
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main(sys.argv))
