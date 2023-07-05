#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2_path_planning_customized.srv import PlanTrajectory2D

# from ros2_path_planning.srv import PlanTrajectory
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid

# Some imports that might be of interest
import numpy as np
from heapq import heappop, heappush
import random


# Path planning service class
class PathPlanningService(Node):
    def __init__(self):
        super().__init__("path_planning_service")

        # Setup ROS service
        self.srv = self.create_service(
            PlanTrajectory2D, "plan_trajectory", self.service_callback
        )

    # Callback function for handle a service call
    def service_callback(
        self, request: PlanTrajectory2D.Request, response: PlanTrajectory2D.Response
    ):
        self.get_logger().info(
            # f"Got positions: robot = {request.robot_position.position}, target = {request.target_position.position}"
            f"Got positions: robot = {request.robot_position}, target = {request.target_position}",
            throttle_duration_sec=0.5,
        )
        self.get_logger().info(
            f"Got map dimensions: width = {request.grid_map.info.width}, height = {request.grid_map.info.height}",
            throttle_duration_sec=0.5,
        )

        try:
            # Perform brute force random search
            trajectory = self.random_search_algorithm(
                request.robot_position, request.target_position, request.grid_map
            )

            # Set the resulting trajectory in the response
            response.trajectory = trajectory

        except Exception as e:
            self.get_logger().error(f"Service error: {e}")

        # Send response
        return response

    # Naive brute force random search algorithm
    def random_search_algorithm(
        self,
        robot_position: Pose2D,
        target_position: Pose2D,
        grid_map: OccupancyGrid,
        num_attempts=100,
    ):
        trajectory: list[Pose2D] = []

        # Get the robot's grid cell coordinates
        robot_x, robot_y = self.position_to_grid_coordinates(robot_position, grid_map)

        # Get the target's grid cell coordinates
        target_x, target_y = self.position_to_grid_coordinates(
            target_position, grid_map
        )

        for _ in range(num_attempts):
            # Generate random grid coordinates between the robot and target positions
            x = random.randint(min(robot_x, target_x), max(robot_x, target_x))
            y = random.randint(min(robot_y, target_y), max(robot_y, target_y))

            # Check if the grid cell is unoccupied (cell value >= 0)
            if grid_map.data[y * grid_map.info.width + x] >= 0:
                # Calculate the corresponding position in the map frame
                position = self.grid_coordinates_to_position(x, y, grid_map)

                # Create a pose from the position
                pose = Pose2D(x=position.x, y=position.y, theta=position.theta)

                # Add the pose to the trajectory
                trajectory.append(pose)

                # Update robot position
                robot_x, robot_y = x, y
                if robot_x == target_x and robot_y == target_y:
                    break

        return trajectory

    # Convert position to grid cell coordinates
    def position_to_grid_coordinates(self, position: Pose2D, grid_map: OccupancyGrid):
        x = int(
            (position.x - grid_map.info.origin.position.x) / grid_map.info.resolution
        )
        y = int(
            (position.y - grid_map.info.origin.position.y) / grid_map.info.resolution
        )
        return x, y

    # Convert grid cell coordinates to position
    def grid_coordinates_to_position(self, x, y, grid_map: OccupancyGrid):
        position = Pose2D()
        position.x = (
            grid_map.info.origin.position.x
            + x * grid_map.info.resolution
            + grid_map.info.resolution / 2
        )
        position.y = (
            grid_map.info.origin.position.y
            + y * grid_map.info.resolution
            + grid_map.info.resolution / 2
        )
        return position


# Main function
def main(args=None):
    # Create and run the service
    rclpy.init(args=args)
    planner = PathPlanningService()
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
