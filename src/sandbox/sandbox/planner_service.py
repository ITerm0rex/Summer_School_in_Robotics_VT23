#!/usr/bin/env python3
import time
import rclpy
import math
from rclpy.node import Node
from ros2_path_planning_customized.srv import PlanTrajectory2D
from geometry_msgs.msg import Point, Pose2D
from nav_msgs.msg import OccupancyGrid

import numpy as np
from heapq import heappop, heappush


from rclpy.duration import Duration


# Node class
class TreeNode:
    def __init__(self, f, g, grid_coordinates, parent=None):
        self.f = f
        self.g = g
        self.h = f + g
        self.grid_coordinates = grid_coordinates
        self.parent = parent

    def __eq__(self, __value: object) -> bool:
        try:
            return self.grid_coordinates == __value.grid_coordinates
        except:
            return False


# Path planning service class
class PathPlanningService(Node):
    def __init__(self):
        super().__init__("path_planning_service")

        # Setup ROS service
        self.srv = self.create_service(
            PlanTrajectory2D, "plan_trajectory", self.service_callback
        )

    # fmt: off
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

    # Calculate straight line distance between a starting grid position and a target grid position
    def euclidean_distance(self, position, target):
        difference_in_x = target[0] - position[0]
        difference_in_y = target[1] - position[1]
        return np.hypot(difference_in_x, difference_in_y)

    def manhattan_distance(self, position, target):
        difference_in_x = abs(target[0] - position[0])
        difference_in_y = abs(target[1] - position[1])
        return difference_in_x + difference_in_y
    
    def man_euc_distance(self, position, target, exp = 1.0):
        difference_in_x = target[0] - position[0]
        difference_in_y = target[1] - position[1]
        return np.power(np.power(abs(difference_in_x), exp) + np.power(abs(difference_in_y), exp), 1/exp)
        # return self.manhattan_distance(position, target) * 0.3 + self.euclidean_distance(position, target) * 0.7
    
    # A* PATHFINDING ALGORITHM
    # g = cost from starting point to current node (cost so far)
    # f = straight line distance from current node to target (euclidian distance)
    # h = f + g

    def a_star(
        self, robot_position: Pose2D, target_position: Pose2D, grid_map: OccupancyGrid
    ):
        distFunc = self.man_euc_distance
        # distFunc = self.manhattan_distance
        # distFunc = self.euclidean_distance
        delta = 1.0
        
        
        
        
        # nodes to explore
        open_list: list[TreeNode] = []
        # nodes already explored
        closed_list: list[TreeNode] = []

        nodes_explored = 0

        # Get the grid coordinates for the robot and the target
        robot_grid_coordinates = self.position_to_grid_coordinates(
            robot_position, grid_map
        )
        target_grid_coordinates = self.position_to_grid_coordinates(
            target_position, grid_map
        )
        self.get_logger().info(f"ROBOT_POSITION: {robot_grid_coordinates}", throttle_duration_sec=0.5)
        self.get_logger().info(f"TARGET_POSITION: {target_grid_coordinates}", throttle_duration_sec=0.5)
        
        # Calculate straight line distance between starting position and target
        f = distFunc(robot_grid_coordinates, target_grid_coordinates)
        g = 0
        self.get_logger().info(f"STARTING NODE: f:{f}, g:{g}", throttle_duration_sec=0.5)
        # Create starting node and add to open array
        starting_node = TreeNode(f, g, robot_grid_coordinates)
        open_list.append(starting_node)

        # create target node
        target_node = TreeNode(0, 0, target_grid_coordinates)

        # Find the shortest path
        while len(open_list) > 0:
            # self.get_logger().info(f" OPEN LIST: {open_list}")
            # Find the best node to travel to
            current_node = open_list[0]
            current_index = 0

            for index, node in enumerate(open_list):
                if node.h < current_node.h:
                    current_node = node
                    current_index = index

            # Remove from open
            closed_list.append(open_list.pop(current_index))
            nodes_explored += 1 
            self.get_logger().info(f"Nodes Explored: {nodes_explored}", throttle_duration_sec=0.5)
            
            # If goal is found return the path taken
            # ? if current_node == target_node:
            # ? if current_node.grid_coordinates == target_grid_coordinates:

            if distFunc(current_node.grid_coordinates, target_grid_coordinates) <= delta:
                path: list[Pose2D] = []
                current = current_node
                while current != None:
                    # Convert coordiantes to positions
                    x = current.grid_coordinates[0]
                    y = current.grid_coordinates[1]

                    position = self.grid_coordinates_to_position(x, y, grid_map)
                    path.append(position)
                    current = current.parent

                final_path = path[::-1]
                #? self.get_logger().info(f"PATH FOUND: {final_path}")

                return final_path
            else:
                # Generate children
                children: list[TreeNode] = []
                for new_coordinates in [
                    (0, -1),
                    (0, 1),
                    (-1, 0),
                    (1, 0),
                    (-1, -1),
                    (-1, 1),
                    (1, -1),
                    (1, 1)
                ]:
                    x = current_node.grid_coordinates[0] + new_coordinates[0]
                    y = current_node.grid_coordinates[1] + new_coordinates[1]
                    node_coordinates = (x, y)

                    if not self.is_valid(grid_map, node_coordinates):
                        continue

                    # Calculate f,g,h for new node
                    f = distFunc(node_coordinates, target_node.grid_coordinates)
                    g = current_node.g + 1
                    #  Create new node for child
                    new_node = TreeNode(f, g, node_coordinates, current_node)
                    # Add child to children array
                    children.append(new_node)
                # ? self.get_logger().info(f"children {children}")
                # Add children array to open array
                for child in children:
                    # If child has already been explored then don't add it
                    if child in closed_list:
                        self.get_logger().info("CHILD IN CLOSED LIST", throttle_duration_sec=0.5)
                        continue
                    # If child is already in the list and the current path is slower then don't add it
                    if (
                        child in open_list
                        and child.g > open_list[open_list.index(child)].g
                    ):
                        self.get_logger().info("CHILD IN OPEN LIST", throttle_duration_sec=0.5)
                        continue

                    open_list.append(child)
        return None
    

    def is_valid(self, grid_map: OccupancyGrid, coordinates):
        x = coordinates[0]
        y = coordinates[1]
        height = grid_map.info.height
        width = grid_map.info.width
    
        if (
            x >= 0
            and x < width
            and y >= 0
            and y < height
            and grid_map.data[y * width + x] == 0
        ):
            #? self.get_logger().info("PASSED IS_VALID")
            return True
        else:
            return False
    # fmt: on

    # Callback function for handle a service call
    def service_callback(
        self, request: PlanTrajectory2D.Request, response: PlanTrajectory2D.Response
    ):
        self.get_logger().info(f"|||INFO|||: {request.grid_map.info = } ")
        self.get_logger().info(
            f"Got positions: robot = {request.robot_position}, target = {request.target_position}"
        )
        # self.get_logger().info(
        #     f"Got map dimensions: width = {request.grid_map.info.width}, height = {request.grid_map.info.height}"
        # )

        # ----------------------------------------
        # Add your path planning algorithm here.
        # ----------------------------------------

        debug = not True
        if not debug:
            path = self.a_star(
                request.robot_position, request.target_position, request.grid_map
            )

            if path == None:
                self.get_logger().info(f"FINAL PATH NOT FOUND")
            else:
                # self.get_logger().info(f"FINAL PATH: {path}")
                self.get_logger().info(f"FINAL PATH #: {len(path)}")
                # self.get_logger().info(f"FINAL PATH #: {path}")

            if path:
                for index, p in enumerate(path):
                    if index % 2 == 0:
                        response.trajectory.append(p)

        else:
            # time.sleep(2)
            # self.get_clock().sleep_for(Duration(seconds=2))
            response.trajectory.append(Pose2D(x=1.0, y=1.0))
            response.trajectory.append(Pose2D(x=1.2, y=1.2))
            response.trajectory.append(Pose2D(x=1.4, y=1.4))
            response.trajectory.append(Pose2D(x=1.6, y=1.2))

        # response.trajectory.append(request.robot_position)
        # response.trajectory.append(request.target_position)

        # Send resutling trajectory
        self.get_logger().info(
            f"!!!!!!!!!!!!!!!!!!!!!!!!!{len(response.trajectory) = } "
        )

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
