"""*****************************************************************************
* IMPORTS
*****************************************************************************"""
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import math
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, PoseArray
import numpy as np
from sensor_msgs.msg import LaserScan


"""*****************************************************************************
* HELPER CLASSES
*****************************************************************************"""

class Pose2D:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def update_from_msg(self, p):
        self.x = p.position.x
        self.y = p.position.y
        quat = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        self.yaw = euler_from_quaternion(quat)[2]


class SpeedTracker:
    def __init__(self):
        self.current_pose = Pose2D()
        self.prev_pose = Pose2D()
        self.vx = 0.0
        self.vy = 0.0
        self.v_yaw = 0.0
        self.last_time = None

    def update(self, pose_msg, time):        
        quat = [
            pose_msg.orientation.x, 
            pose_msg.orientation.y, 
            pose_msg.orientation.z, 
            pose_msg.orientation.w
        ]
        new_yaw = euler_from_quaternion(quat)[2]
        if self.last_time is not None:
            dt = (time - self.last_time).nanoseconds / 1e9
            
            if dt > 0:
                new_x = pose_msg.position.x
                new_y = pose_msg.position.y
                
                self.vx = (new_x - self.current_pose.x) / dt
                self.vy = (new_y - self.current_pose.y) / dt

                delta_yaw = new_yaw - self.current_pose.yaw
                delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))
                self.v_yaw = delta_yaw / dt

        self.current_pose.update_from_msg(pose_msg)
        self.last_time = time

    def get_point_speed(self):
        return self.vx, self.vy
    
    def get_yaw_speed(self):
        return self.v_yaw






"""*****************************************************************************
* NODE CLASS
*****************************************************************************"""
class PFFollowerNode(Node):
    # Constructor --------------------------------------------------------------
    def __init__(self):
        super().__init__("potential_fields_follower_node")
        
        # 1. Declare parameters WITH the configurations passed from launch
        self.declare_parameter("curve_offset", 0.0)
        self.declare_parameter("vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("pose_topic", "/pose")
        self.declare_parameter("neighbor_topics", [""])
        self.declare_parameter("controller_k", 1.0)
        self.declare_parameter("feedback_linearization_l", 0.3)
        self.declare_parameter("curve_size_multiplier", 5.0)
        self.declare_parameter("curve_time_multiplier", 0.2)
        self.declare_parameter("attractive_gain", 1.0)
        self.declare_parameter("repulsive_gain", 1.0)
        self.declare_parameter("repulsive_threshold", 2.0)
        self.declare_parameter("max_speed", 1.0)

        self.curve_offset = self.get_parameter("curve_offset").value
        self.vel_topic = self.get_parameter("vel_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.pose_topic_list = self.get_parameter("neighbor_topics").value
        self.controller_k = self.get_parameter("controller_k").value
        self.feedback_linearization_l = self.get_parameter("feedback_linearization_l").value
        self.curve_size_multiplier = self.get_parameter("curve_size_multiplier").value
        self.curve_time_multiplier = self.get_parameter("curve_time_multiplier").value
        self.attractive_gain = self.get_parameter("attractive_gain").value
        self.repulsive_gain = self.get_parameter("repulsive_gain").value
        self.repulsive_threshold = self.get_parameter("repulsive_threshold").value
        self.max_speed = self.get_parameter("max_speed").value

        self.pose = Pose2D()
        self.speed_trk = SpeedTracker()
        self.scan = None

        self.create_subscription(PoseArray, self.pose_topic, self.pose_callback, 10)
        
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, self.vel_topic, 10)
        
        self.neighbor_poses = [Pose2D() for _ in range(len(self.pose_topic_list))]
        for i, topic in enumerate(self.pose_topic_list):
            self.create_subscription(
                PoseArray, 
                topic, 
                partial(self.neighbor_pose_callback, robot_index=i), 
                10
            )

        self.create_timer(0.05, self.control_loop)



    def neighbor_pose_callback(self, msg,robot_index):
        if not msg.poses:
            return
        p = msg.poses[0]
        self.neighbor_poses[robot_index].update_from_msg(p)


    # Callbacks ----------------------------------------------------------------
    def pose_callback(self, msg):
        now_time = self.get_clock().now()
        p = msg.poses[0]
        self.pose.update_from_msg(p)
        self.speed_trk.update(p,now_time)

    """*************************************************************************
    * CURVE FOLLOWER
    *************************************************************************"""


    def get_correction_fl_point_speed(self,time:Time):
        desired_speed = self.get_curve_velocity_at_time(time)
        position_error = self.get_position_error(time)
        return tuple(a - self.controller_k * b for a, b in zip(desired_speed, position_error))

    def get_position_error(self,time:Time):
        desired_point = self.get_curve_point_at_time(time)
        fl_point = self.get_feedback_linearization_point_position()
        x_error = fl_point[0]-desired_point[0]
        y_error = fl_point[1]-desired_point[1]
        return (x_error,y_error)

    def get_speed_error(self,time):
        desired_speed = self.get_curve_velocity_at_time(time)
        fl_speed = self.get_feedback_linearization_point_speed()
        vx_error = fl_speed[0]-desired_speed[0]
        vy_error = fl_speed[1]-desired_speed[1]
        return (vx_error,vy_error)

    def get_feedback_linearization_point_position(self):
        x = self.pose.x + self.feedback_linearization_l * math.cos(self.pose.yaw)
        y = self.pose.y + self.feedback_linearization_l * math.sin(self.pose.yaw)
        return (x,y)
    
    def get_feedback_linearization_point_speed(self):
        center_vx, center_vy = self.speed_trk.get_point_speed()
        vx = center_vx - self.feedback_linearization_l * math.sin(self.pose.yaw) * self.speed_trk.get_yaw_speed()
        vy = center_vy + self.feedback_linearization_l * math.cos(self.pose.yaw) * self.speed_trk.get_yaw_speed()
        return (vx,vy)


    def get_curve_point_at_time(self,time:Time):
        
        theta = time.nanoseconds/1e9 * self.curve_time_multiplier + self.curve_offset
        sin_t = math.sin(theta)
        cos_t = math.cos(theta)
        denom = 1 + sin_t**2

        x = (self.curve_size_multiplier * cos_t)/denom
        y = (self.curve_size_multiplier * sin_t * cos_t)/denom
        return (x,y)

    def get_curve_velocity_at_time(self, time:Time):
        
        theta = time.nanoseconds/1e9 * self.curve_time_multiplier + self.curve_offset
        sin_t = math.sin(theta)
        denom = 1 + sin_t**2
        d_theta = self.curve_time_multiplier
        
        dx_dtheta = self.curve_size_multiplier * (sin_t * (sin_t**2 - 3)) / (denom**2)
        vx = dx_dtheta * d_theta
        
        dy_dtheta = self.curve_size_multiplier * (1 - 3 * sin_t**2) / (denom**2)
        vy = dy_dtheta * d_theta
        return (vx, vy)


    """*************************************************************************
    * POTENTIAL FIELDS
    *************************************************************************"""


    def scan_index_to_rad(self, index):
        if self.scan is None:
            return None
        return self.scan.angle_min + (index * self.scan.angle_increment)    
        

    def get_min_closest_obstacle(self,scan):
        if scan is None or len(scan.ranges) == 0:
            return (float('inf'), 0.0)

        ranges = np.array(scan.ranges)
        ranges[(ranges < scan.range_min) | (ranges > scan.range_max)] = float('inf')

        min_dist = np.min(ranges)
        
        if min_dist == float('inf'):
            return (float('inf'), 0.0)

        min_index = np.argmin(ranges)
        angle = self.scan_index_to_rad(min_index)

        return (min_dist, angle)

    def decompose_vector(self, magnitude, angle):
        fx = magnitude * math.cos(angle)
        fy = magnitude * math.sin(angle)
        return (fx, fy)

    def local_angle_to_global_angle(self, angle):
        return self.pose.yaw + angle

    def publish_vel(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.vel_publisher.publish(cmd)

    def inverse_fl(self, vx_h, vy_h):
        theta = self.pose.yaw
        l = self.feedback_linearization_l
        v = math.cos(theta) * vx_h + math.sin(theta) * vy_h
        omega = (-math.sin(theta) * vx_h + math.cos(theta) * vy_h) / l
        
        return (v, omega)

    def get_attractive_force(self,goal):
        fx = -self.attractive_gain*(self.pose.x-goal[0])
        fy = -self.attractive_gain*(self.pose.y-goal[1])
        return (fx,fy)

    def get_repulsive_force_magnitute(self, dist):
        if dist < 0.01: 
            dist = 0.01
            
        if dist > self.repulsive_threshold:
            return 0.0
            
        return self.repulsive_gain * (1/dist - 1/self.repulsive_threshold) * 1/(dist**2)

    def scan_callback(self, msg):
        self.scan = msg











    def publish_vel(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.vel_publisher.publish(cmd)


    def control_loop(self):
        now_time = self.get_clock().now()
        vx_total, vy_total = self.get_correction_fl_point_speed(now_time)

        dist, local_angle = self.get_min_closest_obstacle(self.scan)
        rep_mag = self.get_repulsive_force_magnitute(dist)
        rep_angle_global = self.local_angle_to_global_angle(local_angle) 
        rvx, rvy = self.decompose_vector(rep_mag, rep_angle_global)
        vx_total -= rvx
        vy_total -= rvy

        for n_pose in self.neighbor_poses:
            dx = self.pose.x - n_pose.x
            dy = self.pose.y - n_pose.y
            dist_to_neighbor = math.sqrt(dx**2 + dy**2)
            rep_mag = self.get_repulsive_force_magnitute(dist_to_neighbor)
            angle_to_neighbor = math.atan2(dy, dx) 
            rvx, rvy = self.decompose_vector(rep_mag, angle_to_neighbor)
            vx_total += rvx
            vy_total += rvy

        current_magnitude = math.sqrt(vx_total**2 + vy_total**2)
        if current_magnitude > self.max_speed:
            scaling_factor = self.max_speed / current_magnitude
            vx_total *= scaling_factor
            vy_total *= scaling_factor

        v, omega = self.inverse_fl(vx_total, vy_total)
        
        self.publish_vel(v, omega)

    def stop(self):
        """Emergency stop helper for shutdown."""
        self.publish_vel(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = PFFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.stop()
        except Exception:
            pass

        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()