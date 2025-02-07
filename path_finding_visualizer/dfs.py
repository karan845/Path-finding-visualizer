#!/usr/bin/env python3
"""
Depth-First Search (DFS) for Path Finding with Persistent Animation.
"""

import rclpy
from rclpy.node import Node
from path_finding_visualizer.grid import Grid
import time
import matplotlib.pyplot as plt


class DFSNode(Node):
    def __init__(self):
        super().__init__('dfs_node')
        self.grid_obj = Grid()
        self.found = False
        self.came_from = {}
        self.visited = set()
        self.dfs(self.grid_obj.start)
        self.reconstruct_path()

    def dfs(self, current):
        grid = self.grid_obj
        self.visited.add(current)
        if current == grid.goal:
            self.found = True
            return
        for neighbor in grid.get_neighbors(current):
            if neighbor not in self.visited:
                self.came_from[neighbor] = current
                self.dfs(neighbor)
                if self.found:
                    return
            grid.draw(visited=self.visited, pause_time=0.1)
            time.sleep(0.01)

    def reconstruct_path(self):
        grid = self.grid_obj
        path = []
        if self.found:
            node = grid.goal
            while node != grid.start:
                path.append(node)
                node = self.came_from[node]
            path.append(grid.start)
            path.reverse()
            self.get_logger().info("Path found using DFS!")
        else:
            self.get_logger().info("No path found using DFS.")
        grid.draw(path=path, visited=self.visited, pause_time=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = DFSNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    plt.ioff()        # Turn off interactive mode.
    plt.show()        # Block the execution until the user closes the window.

    rclpy.shutdown()

if __name__ == '__main__':
    main()
