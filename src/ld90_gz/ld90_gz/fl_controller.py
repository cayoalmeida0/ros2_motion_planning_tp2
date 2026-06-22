#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseArray


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(value, vmax))


class FeedbackLinearizationController(Node):
    """
    Feedback linearization controller using Gazebo ground-truth pose.
    """

    def __init__(self) -> None:
        super().__init__("feedback_linearization_controller")

        # Goal
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)

        # Controller
        self.declare_parameter("a", 0.10)
        self.declare_parameter("kx", 0.6)
        self.declare_parameter("ky", 0.4)

        # Saturation
        self.declare_parameter("v_max", 0.5)
        self.declare_parameter("w_max", 0.5)

        # Stop condition
        self.declare_parameter("goal_tolerance", 0.02)

        # Topics
        self.declare_parameter("pose_topic", "/ld90_gt_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.a = float(self.get_parameter("a").value)
        self.kx = float(self.get_parameter("kx").value)
        self.ky = float(self.get_parameter("ky").value)
        self.v_max = float(self.get_parameter("v_max").value)
        self.w_max = float(self.get_parameter("w_max").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)

        pose_topic = str(self.get_parameter("pose_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        if self.a <= 0.0:
            raise ValueError("Parameter 'a' must be > 0.")

        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_yaw: Optional[float] = None

        self.shutdown_requested = False
        self.shutdown_timer = None

        self.pose_sub = self.create_subscription(
            PoseArray, pose_topic, self.pose_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"FL ground-truth controller started. "
            f"Goal=({self.goal_x:.2f}, {self.goal_y:.2f}), "
            f"a={self.a:.2f}, kx={self.kx:.2f}, ky={self.ky:.2f}"
        )

    def pose_callback(self, msg: PoseArray) -> None:
        if not msg.poses:
            return

        p = msg.poses[0]

        self.current_x = float(p.position.x)
        self.current_y = float(p.position.y)
        self.current_yaw = quat_to_yaw(
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w
        )

    def publish_stop(self) -> None:
        if not rclpy.ok():
            return

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def shutdown_node(self) -> None:
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
            self.shutdown_timer = None

        self.get_logger().info("Finalizando nó...")
        self.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    def control_loop(self) -> None:
        if self.shutdown_requested:
            return

        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return

        x = self.current_x
        y = self.current_y
        theta = self.current_yaw

        x_p = x + self.a * math.cos(theta)
        y_p = y + self.a * math.sin(theta)

        e_x = self.goal_x - x_p
        e_y = self.goal_y - y_p

        dist = math.hypot(e_x, e_y)
        if dist < self.goal_tolerance:
            self.get_logger().info("Alvo atingido. Encerrando o nó.")

            self.publish_stop()
            self.shutdown_requested = True
            self.control_timer.cancel()
            self.shutdown_timer = self.create_timer(0.2, self.shutdown_node)
            return

        u_x = self.kx * e_x
        u_y = self.ky * e_y

        v = math.cos(theta) * u_x + math.sin(theta) * u_y
        w = (-math.sin(theta) / self.a) * u_x + (math.cos(theta) / self.a) * u_y

        v = clamp(v, -self.v_max, self.v_max)
        w = clamp(w, -self.w_max, self.w_max)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FeedbackLinearizationController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok() and not node.shutdown_requested:
                node.publish_stop()
        except Exception:
            pass

        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()