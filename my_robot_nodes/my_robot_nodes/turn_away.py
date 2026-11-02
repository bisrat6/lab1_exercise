#!/usr/bin/env python3
"""Node that turns diffdrive_bot away from obstacles with dynamic speed parameters,
status publishing, and services to enable/disable operation and configure turn speed."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed


class TurnAway(Node):

    def __init__(self):
        super().__init__("turn_away")

        # Declare parameters with default values
        self.declare_parameter("forward_speed", 0.2)
        self.declare_parameter("turn_speed", 0.5)
        self.declare_parameter("max_turn_speed", 1.5)

        self.enabled = False

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.status_pub = self.create_publisher(RobotStatus, "robot_status", 10)

        # Subscriber
        self.create_subscription(ObstacleInfo, "obstacle", self.on_obstacle, 10)

        # Services
        self.create_service(SetBool, "turn_away_enable", self.on_enable)
        self.create_service(SetTurnSpeed, "set_turn_speed", self.on_set_turn_speed)

        self.get_logger().info("turn_away node initialized (currently disabled)")

    def on_enable(self, request, response):
        if request.data:
            self.enabled = True
            response.success = True
            response.message = "turn_away enabled"
        else:
            self.enabled = False
            # Send one zero velocity when switched off
            self.cmd_vel_pub.publish(Twist())
            response.success = True
            response.message = "turn_away disabled"

        self.get_logger().info(response.message)
        return response

    def on_set_turn_speed(self, request, response):
        max_turn_speed = self.get_parameter("max_turn_speed").value
        if not 0.0 < request.turn_speed <= max_turn_speed:
            response.success = False
            response.message = (
                f"turn_speed must be above 0 and at most {max_turn_speed:.2f} rad/s"
            )
        else:
            self.set_parameters([Parameter("turn_speed", value=float(request.turn_speed))])
            response.success = True
            response.message = f"turn_speed set to {request.turn_speed:.2f} rad/s"

        self.get_logger().info(response.message)
        return response

    def on_obstacle(self, msg: ObstacleInfo):
        # Read parameters dynamically each time a message arrives
        forward_speed = float(self.get_parameter("forward_speed").value)
        turn_speed = float(self.get_parameter("turn_speed").value)

        status_msg = RobotStatus()
        status_msg.obstacle_distance = float(msg.distance)

        if not self.enabled:
            status_msg.state = "stopped"
            self.status_pub.publish(status_msg)
            return

        twist = Twist()

        if msg.too_close:
            status_msg.state = "turning"
            twist.linear.x = 0.0
            # Obstacle to the left (angle > 0) -> turn right (negative angular.z)
            # Obstacle to the right or ahead (angle <= 0) -> turn left (positive angular.z)
            if msg.angle > 0.0:
                twist.angular.z = -turn_speed
            else:
                twist.angular.z = turn_speed
        else:
            status_msg.state = "forward"
            twist.linear.x = forward_speed
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)
        self.status_pub.publish(status_msg)


def main():
    rclpy.init()
    node = TurnAway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

