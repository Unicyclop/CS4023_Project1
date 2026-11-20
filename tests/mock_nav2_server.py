#!/usr/bin/env python3
"""Mock Nav2 server for testing the tour-guide program.

This server simulates navigation only. It does not move a robot.
"""

import time
import threading

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class MockNav2Server(Node):
    def __init__(self):
        super().__init__("mock_nav2_server")

        self._lock = threading.Lock()
        self._goal_active = False

        self._server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(
            "Mock Nav2 server ready. Waiting for navigation goals."
        )

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
        else:
            goal_handle.succeed()
            self.get_logger().info("Navigation goal completed successfully.")

        return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)

    node = MockNav2Server()
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
