#!/usr/bin/env python3
"""
A* Algorithm Implementation for Path Finding with Persistent Animation.
"""

import rclpy
from rclpy.node import Node
import heapq
from path_finding_visualizer.grid import Grid
import time
import matplotlib.pyplot as plt


class AStarNode(Node):
    def __init__(self):
        super().__init__('astar_node')
        self.grid_obj = Grid()
        self.astar_search()

    def heuristic(self, a, b):
        # Manhattan distance.
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def astar_search(self):
        grid = self.grid_obj
        start = grid.start
        goal = grid.goal

        frontier = []
        heapq.heappush(frontier, (0, 0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        visited = set()

        while frontier:
            _, current_cost, current_node = heapq.heappop(frontier)
            visited.add(current_node)
            if current_node == goal:
                break
            for neighbor in grid.get_neighbors(current_node):
                new_cost = cost_so_far[current_node] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(frontier, (priority, new_cost, neighbor))
                    came_from[neighbor] = current_node
            grid.draw(visited=visited, pause_time=0.1)
        
        path = []
        if goal in came_from:
            node = goal
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            self.get_logger().info("Path found using A* algorithm!")
        else:
            self.get_logger().info("No path found using A* algorithm.")
        grid.draw(path=path, visited=visited, pause_time=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()

    plt.ioff()        # Turn off interactive mode.
    plt.show()        # Block the execution until the user closes the window.

    rclpy.shutdown()

if __name__ == '__main__':
    main()
