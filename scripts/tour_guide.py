#!/usr/bin/env python3
"""Interactive TurtleBot 4 tour manager using Nav2."""

import math
import os
import threading

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def load_tour_stops():
    """Load tour destinations from the installed YAML configuration."""
    package_share = get_package_share_directory("reactive_turtlebot")

    config_path = os.path.join(
        package_share, "config", "tour_stops.yaml"
    )

    with open(config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    if not isinstance(config, dict):
        raise ValueError("Tour configuration must be a YAML mapping.")

    stops = config.get("tour_stops")

    if not isinstance(stops, list) or not stops:
        raise ValueError("Configuration must contain a nonempty tour_stops list.")

    tour_stops = {}

    for stop in stops:
        if not isinstance(stop, dict):
            raise ValueError("Each tour stop must be a mapping.")

        name = stop["name"]

        if not isinstance(name, str) or not name.strip():
            raise ValueError("Each tour stop needs a valid name.")

        if name in tour_stops:
            raise ValueError(f"Duplicate tour stop: {name}")

        coordinates = (stop["x"], stop["y"], stop["yaw"])

        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for value in coordinates
        ):
            raise ValueError(f"Invalid coordinates for {name}")

        tour_stops[name] = tuple(float(value) for value in coordinates)

    return tour_stops


# Loaded once at startup.
TOUR_STOPS = None


class TourGuide(Node):
    def __init__(self):
        super().__init__("tour_guide")
        self.nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )
        self.lock = threading.Lock()
        self.goal_handle = None
        self.busy = False
        self.cancel_requested = False
        self.full_tour = False
        self.tour_queue = []
        self.last_feedback_distance = None

        self.get_logger().info("Tour guide ready. Waiting for a command.")

    def show_stops(self):
        print("\nAvailable tour stops:")
        for index, (name, coords) in enumerate(TOUR_STOPS.items(), 1):
            x, y, yaw = coords
            print(f"{index}. {name}: x={x}, y={y}, yaw={yaw}")

    def start_navigation(self, stop_names, full_tour=False):
        with self.lock:
            if self.busy:
                print("Navigation is already active.")
                return

            if not stop_names:
                print("No tour stops selected.")
                return

            self.busy = True
            self.cancel_requested = False
            self.full_tour = full_tour
            self.tour_queue = list(stop_names)

        self._send_next_goal()

    def _send_next_goal(self):
        with self.lock:
            if self.cancel_requested or not self.tour_queue:
                self._reset_locked()
                return

            stop_name = self.tour_queue.pop(0)

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server is unavailable.")
            self._finish_navigation()
            return

        # Cancellation may have been requested while waiting for Nav2.
        with self.lock:
            if self.cancel_requested:
                self.get_logger().info(
                    "Navigation canceled before goal submission."
                )
                self._reset_locked()
                return

        x, y, yaw = TOUR_STOPS[stop_name]

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Coordinate goal submission with cancellation requests.
        with self.lock:
            if self.cancel_requested:
                self._reset_locked()
                self.get_logger().info(
                    "Navigation canceled before goal submission."
                )
                return

            self.get_logger().info(f"Sending navigation goal: {stop_name}")

            future = self.nav_client.send_goal_async(
                goal, feedback_callback=self._feedback_callback
            )

        # Register the callback outside the lock.
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Goal request failed: {exc}")
            self._finish_navigation()
            return

        if not handle.accepted:
            self.get_logger().error("Nav2 rejected the navigation goal.")
            self._finish_navigation()
            return

        with self.lock:
            self.goal_handle = handle
            should_cancel = self.cancel_requested

        self.get_logger().info("Navigation goal accepted.")

        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

        if should_cancel:
            handle.cancel_goal_async()

    def _feedback_callback(self, message):
        distance = message.feedback.distance_remaining

        with self.lock:
            previous = self.last_feedback_distance
            if previous is not None and abs(distance - previous) < 2.0:
                return
            self.last_feedback_distance = distance

        self.get_logger().info(f"Distance remaining: {distance:.2f} m")

    def _result_callback(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Navigation result error: {exc}")
            self._finish_navigation()
            return

        with self.lock:
            canceled = self.cancel_requested
            self.goal_handle = None
            self.last_feedback_distance = None

        if canceled or result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Tour navigation canceled.")
            self._finish_navigation()
        elif result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Tour stop reached.")

            with self.lock:
                remaining_stops = len(self.tour_queue)

            if remaining_stops == 0:
                if self.full_tour:
                    self.get_logger().info(
                        "Tour completed successfully. All destinations reached."
                    )
                else:
                    self.get_logger().info(
                        "Destination reached successfully."
                    )

            self._send_next_goal()
        else:
            self.get_logger().error(
                f"Navigation failed (status {result.status}). Tour stopped."
            )
            self._finish_navigation()

    def cancel_navigation(self):
        with self.lock:
            if not self.busy:
                print("No active navigation to cancel.")
                return

            self.cancel_requested = True
            self.tour_queue.clear()
            handle = self.goal_handle

        if handle is not None:
            handle.cancel_goal_async()
            print("Navigation cancellation requested.")
        else:
            print("Cancellation queued while goal is being submitted.")

    def _reset_locked(self):
        self.busy = False
        self.cancel_requested = False
        self.full_tour = False
        self.tour_queue.clear()
        self.goal_handle = None
        self.last_feedback_distance = None

    def _finish_navigation(self):
        with self.lock:
            self._reset_locked()


    def shutdown_navigation(self, timeout=5.0):
        """Request cancellation and wait briefly before shutting down."""
        with self.lock:
            active = self.busy

        if not active:
            return True

        self.get_logger().info(
            "Active navigation detected. Requesting cancellation."
        )
        self.cancel_navigation()

        # Keep the ROS executor running so cancellation callbacks
        # can be processed while we wait.
        import time

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            with self.lock:
                if not self.busy:
                    self.get_logger().info(
                        "Navigation finished. Safe to shut down the tour manager."
                    )
                    return True

            time.sleep(0.1)

        self.get_logger().error(
            "Timed out waiting for navigation cancellation. "
            "Robot stop is not confirmed."
        )
        return False


def main(args=None):
    global TOUR_STOPS

    TOUR_STOPS = load_tour_stops()

    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = TourGuide()

    executor_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    executor_thread.start()

    try:
        while rclpy.ok():
            print("\n========== TurtleBot 4 Tour Guide ==========")
            print("1. Show available tour stops")
            print("2. Navigate to a selected stop")
            print("3. Start the complete tour")
            print("4. Cancel current navigation")
            print("5. Exit")

            choice = input("Select an option: ").strip()

            if choice == "1":
                node.show_stops()

            elif choice == "2":
                node.show_stops()
                selection = input("Enter stop number: ").strip()

                if not selection.isdigit():
                    print("Please enter a valid number.")
                    continue

                index = int(selection) - 1
                names = list(TOUR_STOPS)

                if 0 <= index < len(names):
                    node.start_navigation([names[index]])
                else:
                    print("Unknown stop number.")

            elif choice == "3":
                node.start_navigation(list(TOUR_STOPS), full_tour=True)

            elif choice == "4":
                node.cancel_navigation()

            elif choice == "5":
                with node.lock:
                    active = node.busy

                if active:
                    print("Cancel the active navigation before exiting.")
                else:
                    break

            else:
                print("Invalid menu option.")


    except (KeyboardInterrupt, EOFError):
        print("\nShutdown requested.")
    finally:
        shutdown_ok = node.shutdown_navigation(timeout=5.0)

        if shutdown_ok:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            executor_thread.join(timeout=2.0)
        else:
            print(
                "WARNING: Navigation cancellation was not confirmed. "
                "Do not assume the robot has stopped."
            )
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            executor_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
