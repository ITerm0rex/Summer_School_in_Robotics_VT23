#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2_path_planning.srv import PlanTrajectory2D

import numpy as np
from heapq import heappop, heappush


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
            f"Got positions: robot = {request.robot_position}, target = {request.target_position}"
        )
        self.get_logger().info(
            f"Got map dimensions: width = {request.grid_map.info.width}, height = {request.grid_map.info.height}"
        )

        # -----------------------------
        # Add your path planning algorithm here.
        # ----------------------------------------

        response.trajectory.append(request.robot_position)
        response.trajectory.append(request.target_position)

        # Send resutling trajectory
        return response


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
