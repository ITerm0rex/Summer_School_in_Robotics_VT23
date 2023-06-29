#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Point, Pose2D, Vector3, Pose


# from ros2_aruco.ros2_aruco_interfaces.msg import ArucoMarkers
from ros2_aruco_interfaces.msg import ArucoMarkers

Pose2D.__repr__ = lambda self: f"x: {self.x}, y: {self.y}, theta: {self.theta}"
Point.__repr__ = lambda self: f"x: {self.x}, y: {self.y}, z: {self.z}"
# Pose.__repr__ = lambda self: f"x: {self.position.x}, y: {self.y}, z: {self.z}"


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

        self.cmd_publisher = self.create_publisher(Twist, "cmd", 10)

        self.odom_subscription = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

        self.location_subscription = self.create_subscription(
            ArucoMarkers, "/robot_location", self.location_callback, 10
        )

    last_pose = None

    def location_callback(self, msg: ArucoMarkers):
        # print(f"Marker IDs: {msg.marker_ids}")
        # self.get_logger().info(
        #     f"Marker IDs: {msg.marker_ids}", throttle_duration_sec=0.5
        # )
        try:
            index = msg.marker_ids.index(7)
            # self.last_pose = msg.poses.index(0)[index].position
            self.last_pose = msg.poses[index].position
        except:
            self.get_logger().warn("LOST.................")

        self.get_logger().info(
            f"Current Position: {self.last_pose}", throttle_duration_sec=0.5
        )

        # self.get_logger().info(f"Current Position: {msg.marker_ids.index(2)}")
        # print(f"Current Position: {current_position}")

    def set_velocity(self, l_v=0.0, a_v=0.0):
        twist = Twist()
        twist.linear.x = float(l_v)
        twist.angular.z = float(a_v)
        self.cmd_publisher.publish(twist)

    target_list: list[Pose2D | None] = [
        # Pose2D(x=-1.0, y=0.0),
        Pose2D(x=1.0, y=0.0),
        Pose2D(x=1.0, y=1.0),
        Pose2D(x=0.0, y=1.0),
        Pose2D(x=0.0, y=0.0),
        # None,
    ]

    target_index = 0

    def odom_callback(self, msg: Odometry):
        current = Pose_to_Pose2D(msg.pose.pose)
        target = self.target_list[self.target_index]

        if not target:
            self.set_velocity(0, 0)
            return

        d_x = target.x / 2 - current.x
        d_y = target.y / 2 - current.y
        distance = np.hypot(d_x, d_y)

        a_v = np.arctan2(d_y, d_x) - current.theta
        if a_v > np.pi:
            a_v = a_v - 2 * np.pi
        elif a_v < -np.pi:
            a_v = a_v + 2 * np.pi

        if distance >= 0.05:
            # self.set_velocity(0.2, np.sign(a_v) * a_v**2.0)
            # self.set_velocity(0.0, 1 * np.sign(a_v))
            # self.set_velocity(0.2, a_v)
            # self.set_velocity(0.0, 2 * np.sign(a_v))

            # self.set_velocity(0.2, 3 * a_v)

            if abs(a_v) >= np.deg2rad(90 / 4):
                self.set_velocity(0.0, 4 * a_v)
            else:
                self.set_velocity(0.2, a_v)

        else:
            # if self.target_index == len(self.target_list) - 1:
            #     self.set_velocity(0.0, 0.0)
            # else:
            #     self.target_index += 1

            self.target_index += 1
            self.target_index %= len(self.target_list)

        self.get_logger().info(
            str(
                {
                    "target": target,
                    "current": current,
                    "distance": distance,
                    "dx": d_x,
                    "dy": d_y,
                    "av": a_v,
                    # "av": a_v / np.pi * 360,
                }
            ),
            throttle_duration_sec=0.1,
        )

        # self.get_logger().info(
        #     f"Odom twist: {msg.twist.twist}", throttle_duration_sec=1
        # )

        # self.get_logger().info(f"Odom pos: {msg.pose.pose}", throttle_duration_sec=1)


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
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
