#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Point, Pose2D, Vector3, Pose

import math

from ros2_aruco_interfaces.msg import ArucoMarkers
from brickpi3 import BrickPi3

# Customizing the string representation of Pose2D and Point classes
Pose2D.__repr__ = lambda self: f"x: {self.x}, y: {self.y}, theta: {self.theta}"
Point.__repr__ = lambda self: f"x: {self.x}, y: {self.y}, z: {self.z}"


class Pose2D(Pose2D):
    def __init__(self, /, follow_angle=False, **kwargs):
        self.follow_angle = follow_angle
        super().__init__(**kwargs)

    def __repr__(self):
        return f"Pose2D({'{'}x: {self.x}, y: {self.y}, theta: {self.theta}{'}'})"

    def distance_to(self, other: Pose2D | None) -> float | None:
        if not other:
            return None
        return np.hypot(self.x - other.x, self.y - other.y)


# Converts Pose (from ArucoMarker) to Pose2D
def Pose_to_Pose2D(pose: Pose):
    pose2d = Pose2D()

    q = pose.orientation

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)

    pose2d.theta = np.arctan2(siny_cosp, cosy_cosp)

    if np.abs(q.x * q.y + q.z * q.w) == 0.5:
        pose2d.theta = 0.0

    pose2d.x = pose.position.x
    pose2d.y = pose.position.y

    return pose2d


class DifferentialDrive(Node):
    def __init__(self):
        super().__init__("cord_node")

        # Publish robots movement to topic '/cmd'
        self.cmd_publisher = self.create_publisher(Twist, "cmd", 10)

        # Gets the current position and rotation of the robot
        self.odom_subscription = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

        # Gets the current position of the marker
        self.location_subscription = self.create_subscription(
            ArucoMarkers, "/robot_location", self.location_callback, 10
        )

        self.color_right_subscription = self.create_subscription(
            String, "right/color", self.color_right_callback, 10
        )

        self.color_left_subscription = self.create_subscription(
            String, "left/color", self.color_left_callback, 10
        )

        # List of positions for robot to travel to
        self.target_list: list[Pose2D | None] = []

        # Starting index (initialised to 0)
        self.target_index = 0

        # Keeps track of the position of each marker
        self.marker_track_dict: dict[int, Pose2D] = {}

    # Controls the velocity of the robot
    def set_velocity(self, l_v=0.0, a_v=0.0):
        twist = Twist()
        twist.linear.x = float(l_v)
        twist.angular.z = float(a_v)
        self.cmd_publisher.publish(twist)

    def get_marker(self, id: int):
        return self.marker_track_dict.get(id, None)

    # Gets the location of each id in marker_track_dict and store it
    def location_callback(self, msg: ArucoMarkers):
        # self.get_logger().info(f"{self.marker_track_dict.keys() = }")

        for id in self.marker_track_dict:
            if id not in msg.marker_ids:
                self.get_logger().warn(
                    f"-----------LOST #{id}-----------\n{msg.marker_ids}"
                )

        for index, id in enumerate(msg.marker_ids):
            # self.get_logger().info(f"ID:{id}, Index: {index}, msg.poses length: {len(msg.poses)}")
            self.marker_track_dict[id] = Pose_to_Pose2D(msg.poses[index])

    right_color = None
    left_color = None

    # Read color_right module
    def color_right_callback(self, msg: String):
        self.get_logger().info(f"color_right: {msg.data}", throttle_duration_sec=0.5)
        self.right_color = msg.data

    # Read color_left module
    def color_left_callback(self, msg: String):
        self.get_logger().info(f"color_left: {msg.data}", throttle_duration_sec=0.5)
        self.left_color = msg.data

    def odom_callback(self, msg: Odometry):
        current = Pose_to_Pose2D(msg.pose.pose)

        # Marker that the robot is currently at
        # current = self.get_marker(7)

        # Add each marker the robot needs to travel to
        self.target_list = [
            # Pose2D(x=1.0, y=0.0),
            Pose2D(x=1.0, y=0.0),
            # Pose2D(x=0.0, y=1.0),
            Pose2D(x=0.0, y=0.0),
            # Pose2D(x=0.5, y=0.5),
            # self.get_marker(2),
            # Pose2D(x=2.0, y=2.0),
            # self.get_marker(1),
            # Pose2D(x=1.0, y=1.0),
            # current,
        ]

        target = self.target_list[self.target_index]

        if not target or not current:
            self.get_logger().warn(
                "************LOST************", throttle_duration_sec=0.5
            )
            self.get_logger().info(
                f"Target List: {self.target_list}", throttle_duration_sec=0.5
            )
            self.get_logger().info(
                f"Markers: {self.marker_track_dict}", throttle_duration_sec=0.5
            )

        if target is current:
            self.set_velocity(0, 0)
            return

        # self.go_from_current_to_target(current, target)
        # return

        if self.left_color == "red" or self.right_color == "red":
            scalar = 1
            if self.left_color != "red":
                theta = -np.pi/2
            elif self.right_color != "red":
                theta = np.pi/2
            else:
                theta = 0
                scalar = -1

            self.sub_target = Pose2D(x=current.x, y=current.y)
            self.sub_target.x = current.x * np.cos(theta) - current.y * np.sin(theta)
            self.sub_target.y = current.y * np.cos(theta) + current.x * np.sin(theta)
            # self.sub_target.x += scalar * np.cos(current.theta + theta)
            # self.sub_target.y += scalar * np.sin(current.theta + theta)
            # self.sub_target.x *= scalar
            # self.sub_target.y *= scalar

            self.go_from_current_to_target(current, self.sub_target)

        # self.get_logger().info(f"{current.distance_to(self.sub_target)=}")

        if self.sub_target is None:
            self.go_from_current_to_target(current, target)
        elif current.distance_to(self.sub_target) <= 0.1:
            self.sub_target = None

        # self.get_logger().info(f"{self.sub_target=}")

    sub_target: Pose2D = None

    def go_from_current_to_target(self, current: Pose2D, target: Pose2D):
        if not target or not current:
            return

        # difference in x,y between current position and target
        d_x = target.x - current.x
        d_y = target.y - current.y
        # euclidean distance - line the robot needs to travel
        distance = np.hypot(d_x, d_y)

        # arctan2 calculates the angle between the current position and the target position
        # this tells the direction the robot will need to face
        # by subtracting theta (where the robot is currently facing) we know how much further the robot needs to rotate to face the target
        a_v = np.arctan2(d_y, d_x) - current.theta

        # This keeps the angular velocity within the range negative pi -> pi (180 degrees = pi radians)
        # This keeps the angular velocity within the range -180deg -> 180deg (360 degrees total)
        if a_v > np.pi:
            a_v = a_v - 2 * np.pi
        elif a_v < -np.pi:
            a_v = a_v + 2 * np.pi

        # Start travelling toward the target until distance is around 0.05
        # if distance >= 0.05:
        if distance >= 0.2:
            # self.set_velocity(0.2, np.sign(a_v) * a_v**2.0)
            # self.set_velocity(0.0, 1 * np.sign(a_v))

            # a_v = np.sign(a_v) * np.abs((a_v / np.pi + 1) / 2) ** 2
            # a_v = (a_v * 2 - 1) * np.pi

            # a_v *= 1.5
            self.set_velocity(0.2, 3 * a_v)

            # if abs(a_v) >= np.deg2rad(45):
            #     self.set_velocity(0.0, a_v)
            # else:
            #     self.set_velocity(0.2, a_v / 2)

        else:
            # The robot is within a close enough range and should move on to the next target (or loop back to the first)
            self.target_index += 1
            self.target_index %= len(self.target_list)

        output = ""
        for key, value in {
            "distance": distance,
            "target": target,
            "current": current,
            "dx": d_x,
            "dy": d_y,
            "av": a_v,
        }.items():
            output += f"{key}: {value}\n"

        self.get_logger().info(output, throttle_duration_sec=0.5)


# Main function
def main(args=None):
    rclpy.init(args=args)

    try:
        diff_drive = DifferentialDrive()
        try:
            rclpy.spin(diff_drive)
        finally:
            diff_drive.destroy_node()
            rclpy.try_shutdown()
            BrickPi3().reset_all()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
