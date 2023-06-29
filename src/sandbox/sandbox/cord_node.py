#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Point, Pose2D, Vector3, Pose

from ros2_aruco_interfaces.msg import ArucoMarkers
from brickpi3 import BrickPi3


Pose2D.__repr__ = lambda self: f"x: {self.x}, y: {self.y}, theta: {self.theta}"
Point.__repr__ = lambda self: f"x: {self.x}, y: {self.y}, z: {self.z}"


class Pose2D(Pose2D):
    def __init__(self, /, follow_angle=False, **kwargs):
        self.follow_angle = follow_angle
        super().__init__(**kwargs)

    def __repr__(self):
        return f"Pose2D({'{'}x: {self.x}, y: {self.y}, theta: {self.theta}{'}'})"

    def distance_to(self, other: Pose2D):
        return np.hypot(self.x - other.x, self.y - other.y)


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

        # self.odom_subscription = self.create_subscription(
        #     Odometry, "odom", self.odom_callback, 10
        # )

        # self.location_subscription = self.create_subscription(
        #     ArucoMarkers, "/robot_location", self.location_callback, 10
        # )

        # def cb_odom(odom: Odometry):
        #     self.get_logger().info(str(Pose_to_Pose2D(odom.pose.pose)))

        # self.odom_subscription_2 = self.create_subscription(
        #     Odometry, "/zed2i/zed_node/odom", cb_odom, 10
        # )

        # self.odom_subscription_2
        

        self.target_list: list[Pose2D | None] = []

        self.target_index = 0

        self.marker_track_dict: dict[int, Pose2D | None] = {
            2: None,
            7: None,
            8: None,
        }
        
        self.get_marker(3)

    def set_velocity(self, l_v=0.0, a_v=0.0):
        twist = Twist()
        twist.linear.x = float(l_v)
        twist.angular.z = float(a_v)
        self.cmd_publisher.publish(twist)

    def location_callback(self, msg: ArucoMarkers):
        # for id in self.marker_track_dict:
        for id in msg.marker_ids:
            try:
                # index = msg.marker_ids.index(id)
                # self.marker_track_dict[id] = Pose_to_Pose2D(msg.poses[index])
                self.marker_track_dict[id] = Pose_to_Pose2D(msg.poses[id])
            except ValueError:
                self.get_logger().warn(
                    f"-----------LOST #{id}-----------\n{msg.marker_ids}",
                    throttle_duration_sec=0.5,
                )

    def get_marker(self, id: int):
        if id in self.marker_track_dict:
            return 
            self.get_logger().info(str(id))
        else:
            self.get_logger().error(str(id))

    def odom_callback(self, msg: Odometry):
        # current = Pose_to_Pose2D(msg.pose.pose)
        # current_wheel = Pose_to_Pose2D(msg.pose.pose)

        current = self.marker_track_dict.get(7)

        self.target_list = [
            Pose2D(x=1.0, y=0.0),
            Pose2D(x=1.0, y=1.0),
            Pose2D(x=0.0, y=1.0),
            Pose2D(x=0.5, y=0.5),
            # self.marker_track_dict.get(2),
            # Pose2D(x=2.4, y=2.4),
            # self.marker_track_dict.get(8),
            # current,
        ]

        # target = self.marker_track_dict.get(8, None)
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

        self.go_from_current_to_target(current, target)

    def go_from_current_to_target(self, current: Pose2D, target: Pose2D):
        if not target or not current:
            return

        d_x = target.x - current.x
        d_y = target.y - current.y
        distance = np.hypot(d_x, d_y)

        a_v = np.arctan2(d_y, d_x) - current.theta

        # a_v *= 3

        if a_v > np.pi:
            a_v = a_v - 2 * np.pi
        elif a_v < -np.pi:
            a_v = a_v + 2 * np.pi

        # if distance >= 0.05:
        if distance >= 0.2:
            # self.set_velocity(0.2, np.sign(a_v) * a_v**2.0)
            # self.set_velocity(0.0, 1 * np.sign(a_v))
            # self.set_velocity(0.2, a_v)
            # self.set_velocity(0.0, 2 * np.sign(a_v))

            a_v *= 1.5
            # if a_v > np.pi:
            #     # a_v = np.pi
            #     a_v /= 2
            # elif a_v < -np.pi:
            #     # a_v = -np.pi
            #     a_v /= 2

            self.set_velocity(0.3, a_v)

            # if abs(a_v) >= np.deg2rad(90 / 4):
            #     self.set_velocity(0.0, 2 * a_v)
            # else:
            #     self.set_velocity(0.2, a_v)

        else:
            self.target_index += 1
            self.target_index %= len(self.target_list)

        output = ""
        for key, value in {
            # ".": "----------------------------------------",
            # "distance_to": current.distance_to(target),
            "distance": distance,
            # "current_wheel": current_wheel,
            "target": target,
            "current": current,
            "dx": d_x,
            "dy": d_y,
            "av": a_v,
        }.items():
            output += f"{key}: {value}\n"

        self.get_logger().info(output, throttle_duration_sec=0.5)

        # self.get_logger().info(
        #     str(
        #         {
        #             "target": target,
        #             "current": current,
        #             "distance": distance,
        #             "dx": d_x,
        #             "dy": d_y,
        #             "av": a_v,
        #             # "av": a_v / np.pi * 360,
        #         }
        #     ),
        #     throttle_duration_sec=0.2,
        # )

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
            BrickPi3().reset_all()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
