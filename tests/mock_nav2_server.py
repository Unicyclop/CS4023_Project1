#!/usr/bin/env python3
"""Mock Nav2 server for testing the tour-guide program.

This server simulates navigation only. It does not move a robot.
"""
import argparse
import time
import threading

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class MockNav2Server(Node):
    def __init__(self, fail_stop=None, accept_delay=0.0):
        super().__init__("mock_nav2_server")

        self.fail_stop = fail_stop
        self.accept_delay = accept_delay
        self._lock = threading.Lock()
        self._goal_active = False

        self._server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(
            "Mock Nav2 server ready. Waiting for navigation goals."
        )

    def goal_callback(self, goal_request):
        if self.accept_delay > 0:
            self.get_logger().info(
                f"Delaying goal acceptance by {self.accept_delay} seconds."
            )
            time.sleep(self.accept_delay)

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancellation request received.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        x = goal_handle.request.pose.pose.position.x
        y = goal_handle.request.pose.pose.position.y

        self.get_logger().info(
            f"Received navigation goal: x={x:.2f}, y={y:.2f}"
        )

        # Simulate navigation for approximately 20 seconds.
        for step in range(40):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn("Navigation canceled.")
                return NavigateToPose.Result()

            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = max(0.0, 20.0 - 0.5 * (step + 1))
            goal_handle.publish_feedback(feedback)

            time.sleep(0.5)


        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().warn("Navigation canceled.")

        elif self.fail_stop is not None and (
            (self.fail_stop == 1 and abs(x - 1.0) < 0.001 and abs(y) < 0.001)
            or (self.fail_stop == 2 and abs(x - 2.0) < 0.001 and abs(y - 1.0) < 0.001)
            or (self.fail_stop == 3 and abs(x) < 0.001 and abs(y - 2.0) < 0.001)
        ):
            goal_handle.abort()
            self.get_logger().error(
                "Simulated navigation failure at Stop 2."
            )

        else:
            goal_handle.succeed()
            self.get_logger().info(
                "Navigation goal completed successfully."
            )

        return NavigateToPose.Result()


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Mock Nav2 server for TurtleBot 4 tour-guide testing."
    )
    parser.add_argument(
        "--fail-stop",
        type=int,
        choices=[1, 2, 3],
        default=None,
        help="Simulate navigation failure at the selected stop.",
    )

    parser.add_argument(
        "--accept-delay",
        type=float,
        default=0.0,
        help="Delay goal acceptance by the specified number of seconds.",
    )

    options = parser.parse_args(args)

    rclpy.init()
    node = MockNav2Server(
        fail_stop=options.fail_stop,
        accept_delay=options.accept_delay,
        )

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
