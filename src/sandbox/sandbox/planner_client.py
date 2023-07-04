#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from ros2_path_planning.srv import PlanTrajectory2D

import numpy as np
from heapq import heappop, heappush


# Path planning Client class
class PathPlanningClient(Node):
    def __init__(self):
        super().__init__("path_planning_client")

        self.cli = self.create_client(PlanTrajectory2D, "path_planning_client")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        self.req = PlanTrajectory2D.Request()

    def send_request(self, a, b):
        self.req.grid_map = ...
        self.req.robot_position = ...
        self.req.target_position = ...

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


# Main function
def main(args=None):
    # Create and run the Client
    rclpy.init(args=args)
    try:
        planner = PathPlanningClient()
        try:
            response = planner.send_request(int(sys.argv[1]), int(sys.argv[2]))
            planner.get_logger().info(
                "Result of add_two_ints: for %d + %d = %d"
                % (int(sys.argv[1]), int(sys.argv[2]), response.sum)
            )
        finally:
            planner.destroy_node()
            rclpy.try_shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
