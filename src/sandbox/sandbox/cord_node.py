#!/usr/bin/env python3

import array
import threading
import time
import numpy as np
import math
import rclpy
from rclpy.task import Future
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Int32, Float32, String
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, Point, Pose2D, Pose

import std_msgs.msg as stdMsg
import nav_msgs.msg as navMsg
import geometry_msgs.msg as geoMsg

from ros2_path_planning_customized.srv import PlanTrajectory2D
from ros2_aruco_interfaces.msg import ArucoMarkers
from brickpi3 import BrickPi3


# Customizing the string representation of Pose2D and Point classes
Point.__repr__ = lambda self: f"x: {self.x}, y: {self.y}, z: {self.z}"

Pose2D.__repr__ = (
    lambda self: f"Pose2D({'{'}x: {self.x}, y: {self.y}, theta: {self.theta}{'}'})"
)
Pose2D.distance_to = (
    lambda self, other: np.hypot(self.x - other.x, self.y - other.y) if other else None
)


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


class PathPlanningNode(Node):
    def __init__(self):
        super().__init__("cord_node")

        # Publish robots movement to topic '/cmd'
        self.cmd_publisher = self.create_publisher(Twist, "cmd", 10)

        # self.declare_parameter("grid_size", "large")
        self.declare_parameter("grid_size", "small")
        grid_size = self.get_parameter("grid_size").get_parameter_value().string_value

        self.occupancy_grid_subscription = self.create_subscription(
            OccupancyGrid,
            f"/map/occupancy/grid/{grid_size}",
            self.occupancy_grid_callback,
            1,
        )

        self.map_publisher = self.create_publisher(
            geoMsg.PoseArray, "/map/trajectory", 10
        )

        # Gets the current position and rotation of the robot
        # self.odom_subscription = self.create_subscription(
        #     Odometry, "odom", self.odom_callback, 10
        # )

        # Gets the current position of the marker
        self.location_subscription = self.create_subscription(
            ArucoMarkers, "/robot_location", self.location_callback, 1
        )

        # self.color_right_subscription = self.create_subscription(
        #     String, "right/color", self.color_right_callback, 1
        # )

        # self.color_left_subscription = self.create_subscription(
        #     String, "left/color", self.color_left_callback, 1
        # )

        # List of positions for robot to travel to
        self.target_list: list[Pose2D | None] = []

        # Starting index (initialised to 0)
        self.target_index = 0

        # Keeps track of the position of each marker
        self.marker_track_dict: dict[int, Pose2D] = {}

        self.sub_target: Pose2D = None

        self.right_color: str = None
        self.left_color: str = None

        self.grid_map: OccupancyGrid = None
        self.robot_position: Pose2D = None
        self.target_position: Pose2D = None

        self.cli = self.create_client(PlanTrajectory2D, "plan_trajectory")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        self.plan_request = PlanTrajectory2D.Request()
        self.plan_response = PlanTrajectory2D.Response()

        self.timer_update_trajectory = self.create_timer(
            timer_period_sec=3.0, callback=self.update_trajectory
        )

        self.timer_event_loop = self.create_timer(
            timer_period_sec=3.0, callback=self.print_info
        )

        self.log_filter = [
            "trajectory",
            "color",
            "lost_marker",
            "odom_path",
            "lost_lost",
        ]

    def print_info(self):
        if "trajectory" not in self.log_filter:
            self.get_logger().debug(
                f"{self.get_trajectory() =} ", throttle_duration_sec=0.5
            )
        self.show_grid(self.plan_request.grid_map)

    def get_trajectory(self) -> list[Pose2D]:
        return self.plan_response.trajectory

    def future_callback(self, future: Future):
        try:
            self.plan_response = future.result()
        except:
            pass
        # return
        tpa = geoMsg.PoseArray()
        for t in self.get_trajectory():
            p = Pose()
            p.position.x = t.x
            p.position.y = t.y
            tpa.poses.append(p)
        self.map_publisher.publish(tpa)

        self.marker_track_dict.clear()

    def update_trajectory(self):
        try:
            if not self.future.done():
                return
        except:
            pass

        self.robot_position = self.get_marker(7)
        self.target_position = self.get_marker(2)

        if self.grid_map:
            self.plan_request.grid_map = self.grid_map
        else:
            return

        if self.robot_position:
            self.plan_request.robot_position = self.robot_position
        else:
            return

        if self.target_position:
            self.plan_request.target_position = self.target_position
        else:
            return

        try:
            self.future = self.cli.call_async(self.plan_request)
            self.future.add_done_callback(self.future_callback)
        except:
            pass

    def get_marker(self, id: int):
        return self.marker_track_dict.get(id, None)

    # Convert position to grid cell coordinates
    def position_to_grid_coordinates(self, position: Pose2D, grid_map: OccupancyGrid):
        x = (position.x - grid_map.info.origin.position.x) / grid_map.info.resolution
        y = (position.y - grid_map.info.origin.position.y) / grid_map.info.resolution
        return int(x), int(y)

    def show_grid(self, grid_map: OccupancyGrid):
        data = ""
        index = 0
        grid_map_data = list(grid_map.data)

        if 0 < grid_map.info.width and grid_map.info.width <= 60:
            for id, marker in self.marker_track_dict.items():
                x, y = self.position_to_grid_coordinates(marker, grid_map)
                grid_map_data[y * grid_map.info.width + x] = str(id)

            for id, path_point in enumerate(self.plan_response.trajectory):
                x, y = self.position_to_grid_coordinates(path_point, grid_map)
                grid_map_data[y * grid_map.info.width + x] = "." + str(id)

            for id, position in {
                "R": self.plan_request.robot_position,
                "T": self.plan_request.target_position,
            }.items():
                x, y = self.position_to_grid_coordinates(position, grid_map)
                grid_map_data[y * grid_map.info.width + x] = id

            for cell in grid_map_data:
                # data += str(cell) + " "
                # data += " # " if cell else "   "
                data += (
                    "#" if cell == 1 else cell if isinstance(cell, str) else " "
                ).center(2)
                index += 1
                if index % grid_map.info.width == 0:
                    data += "\n"
            self.get_logger().info(
                f"OccupancyGrid data: \n{data}",
                throttle_duration_sec=1,
            )
        else:
            self.get_logger().info(
                f"OccupancyGrid info: {grid_map.info}",
                throttle_duration_sec=1,
            )

    # Gets the location of each id in marker_track_dict and store it
    def location_callback(self, msg: ArucoMarkers):
        for index, id in enumerate(msg.marker_ids):
            if id <= 20:
                self.marker_track_dict[id] = Pose_to_Pose2D(msg.poses[index])
        if "lost_marker" not in self.log_filter:
            for id in self.marker_track_dict:
                if id not in msg.marker_ids:
                    self.get_logger().warn(
                        f"-----------LOST #{id}-----------\n{msg.marker_ids}"
                    )
        # self.path_list_logic()
        self.follow_trajectory()

    def occupancy_grid_callback(self, msg: OccupancyGrid):
        for index in range(len(msg.data)):
            msg.data[index] = 0 if msg.data[index] > 100 else 1
        self.grid_map = msg

    def follow_trajectory(self):
        # Marker that the robot is currently at
        # current = self.get_marker(7)
        current = self.robot_position

        # Add each marker the robot needs to travel to
        self.target_list = [
            # Pose2D(x=1.0, y=0.0),
            # Pose2D(x=1.0, y=0.0),
            # Pose2D(x=0.0, y=1.0),
            # Pose2D(x=0.0, y=0.0),
            # Pose2D(x=0.5, y=0.5),
            # self.get_marker(8),
            # Pose2D(x=2.0, y=2.0),
            # self.get_marker(0),
            *self.get_trajectory(),
            # Pose2D(x=1.0, y=1.0),
            current,
        ]

        target = None
        try:
            self.target_index %= len(self.target_list)
            target = self.target_list[self.target_index]
        except:
            pass

        self.get_logger().info(
            f"{len(self.get_trajectory()) = } {self.target_index = }",
            throttle_duration_sec=0.5,
        )

        # if current:
        #     self.robot_position = current

        # if target:
        #     self.target_position = target

        if not target or not current:
            if "lost_lost" not in self.log_filter:
                self.get_logger().warn(
                    "************LOST************", throttle_duration_sec=0.5
                )
                self.get_logger().info(
                    f"Target List: {self.target_list}", throttle_duration_sec=0.5
                )
                self.get_logger().info(
                    f"Markers: {self.marker_track_dict}", throttle_duration_sec=0.5
                )
            return

        if target is current:
            self.set_velocity(0, 0)
            return
        
        
        return
        if self.go_from_current_to_target(current, target, delta=0.3):
            self.target_index += 1

        if self.left_color == "red" or self.right_color == "red":
            scalar = 1
            if self.left_color != "red":
                theta = -np.pi / 2
            elif self.right_color != "red":
                theta = np.pi / 2
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

    def go_from_current_to_target(self, current: Pose2D, target: Pose2D, delta: float):
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

        if "odom_path" not in self.log_filter:
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

        # Start travelling toward the target until distance is around 0.05
        if distance >= delta:
            if abs(a_v) >= np.deg2rad(45):
                self.set_velocity(0.0, a_v)
            else:
                self.set_velocity(0.1, a_v)
        else:
            return True
        return False

    def path_list_logic(self):
        # Marker that the robot is currently at
        current = self.get_marker(7)

        # Add each marker the robot needs to travel to
        self.target_list = [
            # Pose2D(x=1.0, y=0.0),
            # Pose2D(x=1.0, y=0.0),
            # Pose2D(x=0.0, y=1.0),
            # Pose2D(x=0.0, y=0.0),
            # Pose2D(x=0.5, y=0.5),
            self.get_marker(8),
            # Pose2D(x=2.0, y=2.0),
            self.get_marker(0),
            # Pose2D(x=1.0, y=1.0),
            # current,
        ]

        target = self.target_list[self.target_index]

        if current:
            self.robot_position = current

        if target:
            self.target_position = target

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
            return

        if target is current:
            self.set_velocity(0, 0)
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
            # self.set_velocity(0.2, 3 * a_v)

            if abs(a_v) >= np.deg2rad(45):
                self.set_velocity(0.0, a_v)
            else:
                self.set_velocity(0.1, a_v / 2)

        else:
            # The robot is within a close enough range and should move on to the next target (or loop back to the first)
            self.target_index += 1
            self.target_index %= len(self.target_list)

        if "odom_path" not in self.log_filter:
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

    def odom_callback(self, msg: Odometry):
        self.robot_position = Pose_to_Pose2D(msg.pose.pose)
        # self.path_logic()

    # Read color_right module
    def color_right_callback(self, msg: String):
        if "color" not in self.log_filter:
            self.get_logger().info(
                f"color_right: {msg.data}", throttle_duration_sec=0.5
            )
        self.right_color = msg.data

    # Read color_left module
    def color_left_callback(self, msg: String):
        if "color" not in self.log_filter:
            self.get_logger().info(f"color_left: {msg.data}", throttle_duration_sec=0.5)
        self.left_color = msg.data

    # Controls the velocity of the robot
    def set_velocity(self, l_v=0.0, a_v=0.0):
        twist = Twist()
        twist.linear.x = float(l_v)
        twist.angular.z = float(a_v)
        self.cmd_publisher.publish(twist)


# Main function
def main(args=None):
    try:
        rclpy.init(args=args)
        diff_drive = PathPlanningNode()
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
