#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from ros2_path_planning_customized.srv import PlanTrajectory2D
from geometry_msgs.msg import Point, Pose2D
from nav_msgs.msg import OccupancyGrid

import numpy as np
from heapq import heappop, heappush

# Node class 
class TreeNode():
    def __init__(self, f, g, grid_coordinates, parent=None):
        self.f = f
        self.g = g
        self.h = f + g
        self.grid_coordinates = grid_coordinates
        self.parent = parent

# Path planning service class
class PathPlanningService(Node):
    def __init__(self):
        super().__init__("path_planning_service")

        # Setup ROS service
        self.srv = self.create_service(
            PlanTrajectory2D, "plan_trajectory", self.service_callback
        )

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
    
    # A* PATHFINDING ALGORITHM
    # g = cost from starting point to current node (cost so far)  
    # f = straight line distance from current node to target (euclidian distance) 
    # h = f + g 

    def a_star(self, robot_position: Pose2D, target_position: Pose2D, grid_map: OccupancyGrid):
        # nodes to explore
        open_list: list[TreeNode] = []
        # nodes already explored
        closed_list: list[TreeNode] = []
        
        # Get the grid coordinates for the robot and the target
        robot_grid_coordinates = self.position_to_grid_coordinates(robot_position, grid_map) 
        target_grid_coordinates = self.position_to_grid_coordinates(target_position, grid_map)
        self.get_logger().info(f"ROBOT_POSITION: {robot_grid_coordinates}")
        self.get_logger().info(f"TARGET_POSITION: {target_grid_coordinates}")
        
        # Calculate straight line distance between starting position and target
        f = self.euclidean_distance(robot_grid_coordinates, target_grid_coordinates)
        g = 0
        self.get_logger().info(f"STARTING NODE: f:{f}, g:{g}")
        # Create starting node and add to open array
        starting_node = TreeNode(f,g, robot_grid_coordinates)
        open_list.append(starting_node)

        # create target node
        target_node = TreeNode(0,0,target_grid_coordinates)

        # Find the shortest path
        while(len(open_list) > 0):
            self.get_logger().info(f" OPEN LIST: {open_list}")
            # Find the best node to travel to
            current_node = open_list[0]
            current_index = 0
            
            for index, node in enumerate(open_list):
                if node.h < current_node.h:
                    current_node = node
                    current_index = index
            
            # Remove from open
            closed_list.append(open_list.pop(current_index))

            # If goal is found return the path taken
            # if current_node == target_node:
            if current_node.grid_coordinates == target_grid_coordinates:
                path: list[tuple[float, float]] = []

                while current_node != None:
                    # Convert coordiantes to positions
                    position = self.grid_coordinates_to_position(current_node.grid_coordinates)
                    path.append(position)
                    current_node = current_node.parent

                final_path = path[::-1]
                self.get_logger().info(f"PATH FOUND: {final_path}")

                return final_path
            else:
                # Generate children
                children: list[TreeNode] = []
                for new_coordinates in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: 
                    x = current_node.grid_coordinates[0] + new_coordinates[0]
                    y = current_node.grid_coordinates[1] + new_coordinates[1]
                    node_coordinates = (x, y)

                    if not self.is_valid(grid_map, node_coordinates):
                        continue
                    
                    # Calculate f,g,h for new node
                    f = self.euclidean_distance(node_coordinates, target_node.grid_coordinates)
                    g = current_node.g + 1
                    #  Create new node for child 
                    new_node = TreeNode(f, g, node_coordinates, current_node)
                    # Add child to children array
                    children.append(new_node)
                #? self.get_logger().info(f"children {children}")
                # Add children array to open array
                for child in children:
                    # If child has already been explored then don't add it 
                    if child in closed_list:
                        continue
                    # If child is already in the list and the current path is slower then don't add it
                    if child in open_list and child.g > open_list[open_list.index(child)].g:
                        continue
                    
                    open_list.append(child)                

    def is_valid(self, grid_map: OccupancyGrid, coordinates):
        x = coordinates[0]
        y = coordinates[1]
        height = grid_map.info.height
        width = grid_map.info.width
        #? self.get_logger().info(f"X: {x}, Y: {y}")
        #! nothing can currently pass this condition
        #todo Fix condition to check if position is within the maze and not an obstacle 
        if x >= 0 and x <= width and y >= 0 and y <= height and grid_map.data[y * grid_map.info.width + x] == 0:
            return True
        else:
            return False
        
    # Callback function for handle a service call
    def service_callback(self, request: PlanTrajectory2D.Request, response: PlanTrajectory2D.Response):
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
        # path = self.a_star(request.robot_position, request.target_position, request.grid_map)
        
        # if path == None:
        #     self.get_logger().info(f"FINAL PATH NOT FOUND")
        # else:
        #     self.get_logger().info(f"FINAL PATH: {path}")
        
        
        
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
