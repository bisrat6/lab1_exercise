#!/usr/bin/env python3
"""Action server that turns diffdrive_bot on the spot for a specified angle,
tracking accumulated rotation via wheel odometry to avoid yaw wrap-around issues."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from my_robot_interfaces.action import Rotate


def yaw_from_quaternion(q):
    """Rotation about the vertical axis, in rad, of an orientation quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_difference(target, current):
    """target - current, wrapped to [-pi, pi]."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


class RotateServer(Node):

    def __init__(self):
        super().__init__("rotate_server")
        self.declare_parameter("turn_speed", 0.5)

        self.yaw = None

        group = ReentrantCallbackGroup()
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10, callback_group=group)

        self.action_server = ActionServer(
            self,
            Rotate,
            "rotate",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=group,
        )
        self.get_logger().info("rotate_server initialized and ready")

    def on_odom(self, msg: Odometry):
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def on_goal(self, goal):
        # Reject goal if angle is zero or odometry has not yet been received
        if abs(goal.angle) < 1e-4 or self.yaw is None:
            self.get_logger().warn(f"Rejecting goal: angle={goal.angle}, yaw={self.yaw}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        target_angle = goal_handle.request.angle
        turn_speed = abs(float(self.get_parameter("turn_speed").value))
        self.get_logger().info(f"Rotating {target_angle:.3f} rad at {turn_speed:.2f} rad/s")

        twist = Twist()
        # Positive angle turns left (positive angular.z), negative turns right
        twist.angular.z = turn_speed if target_angle > 0.0 else -turn_speed

        feedback = Rotate.Feedback()
        result = Rotate.Result()

        total_turned = 0.0
        last_yaw = self.yaw

        rate_sleep = 0.05

        while rclpy.ok():
            current_yaw = self.yaw
            if current_yaw is not None and last_yaw is not None:
                # Delta yaw from last reading wrapped to [-pi, pi]
                delta_yaw = angle_difference(current_yaw, last_yaw)
                total_turned += delta_yaw
                last_yaw = current_yaw

            result.turned = float(total_turned)
            feedback.turned = float(total_turned)
            goal_handle.publish_feedback(feedback)

            if goal_handle.is_cancel_requested:
                self.cmd_vel.publish(Twist())
                goal_handle.canceled()
                self.get_logger().info(f"Rotation canceled after {total_turned:.3f} rad")
                return result

            # Check if target reached
            if target_angle > 0.0 and total_turned >= target_angle:
                break
            elif target_angle < 0.0 and total_turned <= target_angle:
                break

            self.cmd_vel.publish(twist)
            time.sleep(rate_sleep)

        # Stop robot
        self.cmd_vel.publish(Twist())
        goal_handle.succeed()
        result.turned = float(total_turned)
        self.get_logger().info(f"Rotation complete. Turned {result.turned:.3f} rad")
        return result


def main():
    rclpy.init()
    node = RotateServer()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

