#!/usr/bin/env python3
"""Action client node that drives diffdrive_bot in a square by commanding
DriveDistance and Rotate action goals sequentially for each side."""

import math
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):

    def __init__(self):
        super().__init__("square_driver")
        self.declare_parameter("side_length", 0.5)

        cb_group = ReentrantCallbackGroup()
        self.drive_client = ActionClient(
            self, DriveDistance, "drive_distance", callback_group=cb_group
        )
        self.rotate_client = ActionClient(
            self, Rotate, "rotate", callback_group=cb_group
        )

        self.worker_thread = threading.Thread(target=self.run_square, daemon=True)
        self.worker_thread.start()

    def run_square(self):
        side_length = float(self.get_parameter("side_length").value)
        self.get_logger().info(f"Starting square driver with side_length = {side_length:.2f} m")

        self.get_logger().info("Waiting for /drive_distance action server...")
        if not self.drive_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("Action server /drive_distance not available. Aborting.")
            return

        self.get_logger().info("Waiting for /rotate action server...")
        if not self.rotate_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("Action server /rotate not available. Aborting.")
            return

        self.get_logger().info("Both action servers connected. Beginning square trajectory.")

        for side in range(1, 5):
            # 1. Drive forward along the side
            self.get_logger().info(f"[Side {side}/4] Driving forward {side_length:.2f} m")
            success, travelled = self.execute_drive(side_length)
            if not success:
                self.get_logger().error(f"Failed to complete drive on side {side} (travelled {travelled:.2f} m). Stopping.")
                return
            self.get_logger().info(f"[Side {side}/4] Drive reached: travelled {travelled:.2f} m")

            # Small pause between moves
            time.sleep(0.5)

            # 2. Rotate a quarter turn (90 degrees = pi / 2 rad) to the left
            turn_angle = math.pi / 2.0  # ~1.5708 rad
            self.get_logger().info(f"[Side {side}/4] Rotating quarter turn left ({turn_angle:.3f} rad)")
            success, turned = self.execute_rotate(turn_angle)
            if not success:
                self.get_logger().error(f"Failed to complete rotate on side {side} (turned {turned:.3f} rad). Stopping.")
                return
            self.get_logger().info(f"[Side {side}/4] Rotate reached: turned {turned:.3f} rad")

            time.sleep(0.5)

        self.get_logger().info("Successfully completed full 4-sided square trajectory!")

    def execute_drive(self, distance):
        goal = DriveDistance.Goal()
        goal.distance = float(distance)

        send_future = self.drive_client.send_goal_async(goal)
        while not send_future.done():
            time.sleep(0.05)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("DriveDistance goal was rejected by server")
            return False, 0.0

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        wrapped_result = result_future.result()
        status = wrapped_result.status
        result = wrapped_result.result
        success = (status == GoalStatus.STATUS_SUCCEEDED and getattr(result, "reached", True))
        return success, getattr(result, "travelled", 0.0)

    def execute_rotate(self, angle):
        goal = Rotate.Goal()
        goal.angle = float(angle)

        send_future = self.rotate_client.send_goal_async(goal)
        while not send_future.done():
            time.sleep(0.05)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Rotate goal was rejected by server")
            return False, 0.0

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        wrapped_result = result_future.result()
        status = wrapped_result.status
        result = wrapped_result.result
        success = (status == GoalStatus.STATUS_SUCCEEDED)
        return success, getattr(result, "turned", 0.0)


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

