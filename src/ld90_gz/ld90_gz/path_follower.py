#!/usr/bin/env python3

"""
path_follower.py

Nó ROS 2 para seguir um caminho planejado em coordenadas do mundo.

Entrada:
- Arquivo .npy contendo waypoints [x, y];
- Pose do robô no tópico /ld90_gt_pose;
- Publicação de velocidade no tópico /cmd_vel.

Controle:
- Seguimento de waypoints por realimentação linearizada;
- O ponto de controle fica a uma distância l à frente do centro do robô;
- Publica Twist(v, w) para navegação diferencial/uniciclo.
"""

import math
import os
import time
from typing import List, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from tf_transformations import euler_from_quaternion


class Pose2D:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def update_from_pose_msg(self, pose_msg):
        self.x = pose_msg.position.x
        self.y = pose_msg.position.y

        quat = [
            pose_msg.orientation.x,
            pose_msg.orientation.y,
            pose_msg.orientation.z,
            pose_msg.orientation.w,
        ]

        self.yaw = euler_from_quaternion(quat)[2]


class PathFollowerNode(Node):
    def __init__(self):
        super().__init__("path_follower_node")

        # ---------------------------------------------------------------------
        # Parâmetros
        # ---------------------------------------------------------------------
        self.declare_parameter("map_name", "tp2_map1")
        self.declare_parameter("algorithm", "astar")
        self.declare_parameter("results_dir", "results")
        self.declare_parameter("path_file", "")

        self.declare_parameter("pose_topic", "/ld90_gt_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("controller_k", 1.2)
        self.declare_parameter("feedback_linearization_l", 0.25)

        self.declare_parameter("max_linear_speed", 0.6)
        self.declare_parameter("max_angular_speed", 1.2)

        self.declare_parameter("waypoint_tolerance", 0.20)
        self.declare_parameter("final_tolerance", 0.25)
        self.declare_parameter("waypoint_stride", 5)

        self.declare_parameter("control_period", 0.05)
        self.declare_parameter("path_wait_timeout", 20.0)

        self.map_name = self.get_parameter("map_name").value
        self.algorithm = self.get_parameter("algorithm").value
        self.results_dir = self.get_parameter("results_dir").value
        self.path_file = self.get_parameter("path_file").value

        self.pose_topic = self.get_parameter("pose_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.controller_k = float(self.get_parameter("controller_k").value)
        self.feedback_linearization_l = float(
            self.get_parameter("feedback_linearization_l").value
        )

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)

        self.waypoint_tolerance = float(self.get_parameter("waypoint_tolerance").value)
        self.final_tolerance = float(self.get_parameter("final_tolerance").value)
        self.waypoint_stride = int(self.get_parameter("waypoint_stride").value)

        self.control_period = float(self.get_parameter("control_period").value)
        self.path_wait_timeout = float(self.get_parameter("path_wait_timeout").value)

        # ---------------------------------------------------------------------
        # Caminho
        # ---------------------------------------------------------------------
        self.path = self.load_path()
        self.current_index = 0

        # ---------------------------------------------------------------------
        # Estado do robô
        # ---------------------------------------------------------------------
        self.pose = Pose2D()
        self.pose_received = False
        self.finished = False

        # ---------------------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------------------
        self.pose_subscriber = self.create_subscription(
            PoseArray,
            self.pose_topic,
            self.pose_callback,
            10,
        )

        self.vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        self.control_timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )

        self.get_logger().info("Path follower iniciado.")
        self.get_logger().info(f"Mapa: {self.map_name}")
        self.get_logger().info(f"Algoritmo: {self.algorithm}")
        self.get_logger().info(f"Waypoints carregados: {len(self.path)}")
        self.get_logger().info(f"Tópico de pose: {self.pose_topic}")
        self.get_logger().info(f"Tópico de velocidade: {self.cmd_vel_topic}")

    # -------------------------------------------------------------------------
    # Carregamento do caminho
    # -------------------------------------------------------------------------
    def resolve_path_file(self) -> str:
        if self.path_file:
            return self.path_file

        return os.path.join(
            self.results_dir,
            self.map_name,
            f"{self.map_name}_{self.algorithm}_path_world.npy",
        )

    def load_path(self) -> List[Tuple[float, float]]:
        path_file = self.resolve_path_file()

        start_time = time.time()

        while not os.path.exists(path_file):
            elapsed = time.time() - start_time

            if elapsed > self.path_wait_timeout:
                raise FileNotFoundError(
                    f"Arquivo de caminho não encontrado após "
                    f"{self.path_wait_timeout:.1f} s: {path_file}"
                )

            self.get_logger().warn(
                f"Aguardando arquivo de caminho: {path_file}"
            )
            time.sleep(1.0)

        raw_path = np.load(path_file)

        if raw_path.ndim != 2 or raw_path.shape[1] < 2:
            raise ValueError(
                "O arquivo de caminho deve possuir dimensão N x 2, "
                "com colunas [x, y]."
            )

        path = [(float(p[0]), float(p[1])) for p in raw_path]

        if len(path) == 0:
            raise ValueError("O caminho carregado está vazio.")

        # Reduz a densidade de waypoints para suavizar o controle.
        # O A* gera um ponto por célula, portanto pode ser denso demais.
        if self.waypoint_stride > 1 and len(path) > 2:
            sparse_path = path[:: self.waypoint_stride]

            if sparse_path[-1] != path[-1]:
                sparse_path.append(path[-1])

            path = sparse_path

        return path

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------
    def pose_callback(self, msg: PoseArray):
        if len(msg.poses) == 0:
            return

        self.pose.update_from_pose_msg(msg.poses[0])
        self.pose_received = True

    # -------------------------------------------------------------------------
    # Controle
    # -------------------------------------------------------------------------
    def control_loop(self):
        if not self.pose_received:
            return

        if self.finished:
            self.publish_velocity(0.0, 0.0)
            return

        final_point = self.path[-1]
        distance_to_final = self.distance_to_point(final_point)

        if distance_to_final <= self.final_tolerance:
            self.finished = True
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info("Objetivo final alcançado.")
            return

        # Avança waypoints já atingidos
        while self.current_index < len(self.path) - 1:
            target = self.path[self.current_index]
            distance = self.distance_to_point(target)

            if distance > self.waypoint_tolerance:
                break

            self.current_index += 1

        target = self.path[self.current_index]

        vx_h, vy_h = self.compute_control_point_velocity(target)
        v, w = self.inverse_feedback_linearization(vx_h, vy_h)

        v = self.clamp(v, -self.max_linear_speed, self.max_linear_speed)
        w = self.clamp(w, -self.max_angular_speed, self.max_angular_speed)

        self.publish_velocity(v, w)

    def compute_control_point_velocity(self, target: Tuple[float, float]) -> Tuple[float, float]:
        x_h, y_h = self.feedback_linearization_point()

        error_x = target[0] - x_h
        error_y = target[1] - y_h

        vx_h = self.controller_k * error_x
        vy_h = self.controller_k * error_y

        speed_norm = math.sqrt(vx_h * vx_h + vy_h * vy_h)

        if speed_norm > self.max_linear_speed:
            scale = self.max_linear_speed / speed_norm
            vx_h *= scale
            vy_h *= scale

        return vx_h, vy_h

    def feedback_linearization_point(self) -> Tuple[float, float]:
        x_h = self.pose.x + self.feedback_linearization_l * math.cos(self.pose.yaw)
        y_h = self.pose.y + self.feedback_linearization_l * math.sin(self.pose.yaw)

        return x_h, y_h

    def inverse_feedback_linearization(self, vx_h: float, vy_h: float) -> Tuple[float, float]:
        theta = self.pose.yaw
        l = self.feedback_linearization_l

        v = math.cos(theta) * vx_h + math.sin(theta) * vy_h
        w = (-math.sin(theta) * vx_h + math.cos(theta) * vy_h) / l

        return v, w

    def distance_to_point(self, point: Tuple[float, float]) -> float:
        dx = point[0] - self.pose.x
        dy = point[1] - self.pose.y

        return math.sqrt(dx * dx + dy * dy)

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    def publish_velocity(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)

        self.vel_publisher.publish(cmd)

    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)

    node = PathFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
