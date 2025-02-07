#!/usr/bin/env python3
"""
Recursive Best-First Search (RBFS) for Path Finding with Persistent Animation.
This node demonstrates a simplified RBFS algorithm with animation.
"""

import rclpy
from rclpy.node import Node
from path_finding_visualizer.grid import Grid
import math
import time
import matplotlib.pyplot as plt


class RBFSNode(Node):
    def __init__(self):
        super().__init__('rbfs_node')
        self.grid_obj = Grid()
        self.came_from = {}
        self.visited = set()
        success, _ = self.rbfs(self.grid_obj.start, math.inf, 0)
        if success:
            self.get_logger().info("Path found using Recursive Best-First Search!")
        else:
            self.get_logger().info("No path found using Recursive Best-First Search.")
        self.reconstruct_path()

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def rbfs(self, current, f_limit, g):
        grid = self.grid_obj
        f = g + self.heuristic(current, grid.goal)
        if f > f_limit:
            return False, f
        if current == grid.goal:
            return True, f
        successors = []
        for neighbor in grid.get_neighbors(current):
            if neighbor not in self.visited:
                self.visited.add(neighbor)
                grid.draw(visited=self.visited, pause_time=0.1)
                time.sleep(0.01)
                cost = g + 1
                f_neighbor = cost + self.heuristic(neighbor, grid.goal)
                successors.append((neighbor, cost, f_neighbor))
                self.came_from[neighbor] = current
        if not successors:
            return False, math.inf
        while True:
            successors.sort(key=lambda x: x[2])
            best = successors[0]
            if best[2] > f_limit:
                return False, best[2]
            alternative = successors[1][2] if len(successors) > 1 else math.inf
            result, best_f = self.rbfs(best[0], min(f_limit, alternative), best[1])
            successors[0] = (best[0], best[1], best_f)
            if result:
                return True, best_f

    def reconstruct_path(self):
        grid = self.grid_obj
        path = []
        if grid.goal in self.came_from:
            node = grid.goal
            while node is not None:
                path.append(node)
                node = self.came_from.get(node, None)
            path.reverse()
        grid.draw(path=path, visited=self.visited, pause_time=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = RBFSNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    plt.ioff()        # Turn off interactive mode.
    plt.show()        # Block the execution until the user closes the window.

    rclpy.shutdown()

if __name__ == '__main__':
    main()
