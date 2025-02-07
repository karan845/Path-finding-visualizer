#!/usr/bin/env python3
"""
Bidirectional Search for Path Finding with Persistent Animation.
This node runs two BFS searches (from start and goal) until they meet.
"""

import rclpy
from rclpy.node import Node
from collections import deque
from path_finding_visualizer.grid import Grid
import time
import matplotlib.pyplot as plt


class BidirectionalNode(Node):
    def __init__(self):
        super().__init__('bidirectional_node')
        self.grid_obj = Grid()
        self.bidirectional_search()

    def bidirectional_search(self):
        grid = self.grid_obj
        start = grid.start
        goal = grid.goal

        frontier_start = deque([start])
        frontier_goal = deque([goal])
        came_from_start = {start: None}
        came_from_goal = {goal: None}
        visited_start = set([start])
        visited_goal = set([goal])
        meeting_point = None

        while frontier_start and frontier_goal:
            # Expand from start side.
            current_start = frontier_start.popleft()
            for neighbor in grid.get_neighbors(current_start):
                if neighbor not in visited_start:
                    visited_start.add(neighbor)
                    came_from_start[neighbor] = current_start
                    frontier_start.append(neighbor)
                    if neighbor in visited_goal:
                        meeting_point = neighbor
                        break
            if meeting_point:
                break

            # Expand from goal side.
            current_goal = frontier_goal.popleft()
            for neighbor in grid.get_neighbors(current_goal):
                if neighbor not in visited_goal:
                    visited_goal.add(neighbor)
                    came_from_goal[neighbor] = current_goal
                    frontier_goal.append(neighbor)
                    if neighbor in visited_start:
                        meeting_point = neighbor
                        break
            if meeting_point:
                break

            grid.draw(visited=visited_start.union(visited_goal), pause_time=0.1)
            time.sleep(0.01)

        path = []
        if meeting_point:
            # Reconstruct path from start to meeting point.
            node = meeting_point
            path_start = []
            while node is not None:
                path_start.append(node)
                node = came_from_start[node]
            path_start.reverse()
            # Reconstruct path from meeting point to goal.
            node = came_from_goal[meeting_point]
            path_goal = []
            while node is not None:
                path_goal.append(node)
                node = came_from_goal[node]
            path = path_start + path_goal
            self.get_logger().info("Path found using Bidirectional Search!")
        else:
            self.get_logger().info("No path found using Bidirectional Search.")

        grid.draw(path=path, visited=visited_start.union(visited_goal), pause_time=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = BidirectionalNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    plt.ioff()        # Turn off interactive mode.
    plt.show()        # Block the execution until the user closes the window.

    rclpy.shutdown()

if __name__ == '__main__':
    main()
