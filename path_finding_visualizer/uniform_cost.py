#!/usr/bin/env python3
"""
Uniform Cost Search (UCS) for Path Finding with Persistent Animation.
This algorithm is equivalent to Dijkstra’s algorithm in our grid.
"""

import rclpy
from rclpy.node import Node
import heapq
from path_finding_visualizer.grid import Grid
import time
import matplotlib.pyplot as plt

class UniformCostNode(Node):
    def __init__(self):
        super().__init__('uniform_cost_node')
        self.grid_obj = Grid()
        self.uniform_cost_search()

    def uniform_cost_search(self):
        grid = self.grid_obj
        start = grid.start
        goal = grid.goal

        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        visited = set()

        while frontier:
            current_cost, current = heapq.heappop(frontier)
            visited.add(current)
            if current == goal:
                break
            for neighbor in grid.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    heapq.heappush(frontier, (new_cost, neighbor))
                    came_from[neighbor] = current
            grid.draw(visited=visited, pause_time=0.1)
            time.sleep(0.01)
        
        path = []
        if goal in came_from:
            node = goal
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            self.get_logger().info("Path found using Uniform Cost Search!")
        else:
            self.get_logger().info("No path found using Uniform Cost Search.")
        grid.draw(path=path, visited=visited, pause_time=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = UniformCostNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    plt.ioff()        # Turn off interactive mode.
    plt.show()        # Block the execution until the user closes the window.

    rclpy.shutdown()

if __name__ == '__main__':
    main()
