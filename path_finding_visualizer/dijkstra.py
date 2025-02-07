#!/usr/bin/env python3
"""
Dijkstra Algorithm Implementation for Path Finding with Persistent Animation.
"""

import rclpy
from rclpy.node import Node
import heapq   # For the priority queue.
from path_finding_visualizer.grid import Grid
import time
import matplotlib.pyplot as plt


class DijkstraNode(Node):
    def __init__(self):
        super().__init__('dijkstra_node')
        self.grid_obj = Grid()  # Create the grid.
        self.dijkstra_search()  # Run the algorithm.

    def dijkstra_search(self):
        grid = self.grid_obj
        start = grid.start
        goal = grid.goal
        frontier = []
        heapq.heappush(frontier, (0, start))
        cost_so_far = {start: 0}
        came_from = {start: None}
        visited = set()

        while frontier:
            current_cost, current_node = heapq.heappop(frontier)
            visited.add(current_node)
            if current_node == goal:
                break
            for neighbor in grid.get_neighbors(current_node):
                new_cost = cost_so_far[current_node] + 1  # Cost per move = 1.
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    heapq.heappush(frontier, (new_cost, neighbor))
                    came_from[neighbor] = current_node
            # Update the persistent window.
            grid.draw(visited=visited, pause_time=0.1)

        # Reconstruct the path.
        path = []
        if goal in came_from:
            node = goal
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            self.get_logger().info("Path found using Dijkstra's algorithm!")
        else:
            self.get_logger().info("No path found using Dijkstra's algorithm.")
        
        grid.draw(path=path, visited=visited, pause_time=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = DijkstraNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    plt.ioff()        # Turn off interactive mode.
    plt.show()        # Block the execution until the user closes the window.

    rclpy.shutdown()

if __name__ == '__main__':
    main()
