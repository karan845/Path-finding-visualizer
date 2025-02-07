#!/usr/bin/env python3
"""
Breadth-First Search (BFS) for Path Finding with Persistent Animation.
"""

import rclpy
from rclpy.node import Node
from collections import deque
from path_finding_visualizer.grid import Grid
import time
import matplotlib.pyplot as plt


class BFSNode(Node):
    def __init__(self):
        super().__init__('bfs_node')
        self.grid_obj = Grid()
        self.bfs_search()

    def bfs_search(self):
        grid = self.grid_obj
        start = grid.start
        goal = grid.goal
        queue = deque([start])
        came_from = {start: None}
        visited = set([start])

        while queue:
            current = queue.popleft()
            if current == goal:
                break
            for neighbor in grid.get_neighbors(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
                    came_from[neighbor] = current
            grid.draw(visited=visited, pause_time=0.1)
        
        path = []
        if goal in came_from:
            node = goal
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            self.get_logger().info("Path found using BFS!")
        else:
            self.get_logger().info("No path found using BFS.")
        grid.draw(path=path, visited=visited, pause_time=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = BFSNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    plt.ioff()        # Turn off interactive mode.
    plt.show()        # Block the execution until the user closes the window.

    rclpy.shutdown()

if __name__ == '__main__':
    main()
