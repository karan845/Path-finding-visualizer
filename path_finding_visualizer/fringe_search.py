#!/usr/bin/env python3
"""
Fringe Search for Path Finding with Persistent Animation.
This node demonstrates a simplified Fringe Search algorithm with animation.
"""

import rclpy
from rclpy.node import Node
from path_finding_visualizer.grid import Grid
import math
import time
import matplotlib.pyplot as plt


class FringeSearchNode(Node):
    def __init__(self):
        super().__init__('fringe_search_node')
        self.grid_obj = Grid()
        self.fringe_search()

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def fringe_search(self):
        grid = self.grid_obj
        start = grid.start
        goal = grid.goal
        threshold = self.heuristic(start, goal)
        fringe = [(start, 0)]
        came_from = {start: None}
        visited = set([start])
        found = False

        while not found:
            next_fringe = []
            min_threshold = float('inf')
            for current, g in fringe:
                f = g + self.heuristic(current, goal)
                if f > threshold:
                    min_threshold = min(min_threshold, f)
                    continue
                if current == goal:
                    found = True
                    break
                for neighbor in grid.get_neighbors(current):
                    if neighbor not in visited:
                        visited.add(neighbor)
                        came_from[neighbor] = current
                        next_fringe.append((neighbor, g + 1))
                time.sleep(0.01)
            grid.draw(visited=visited, pause_time=0.1)
            if found:
                break
            if not next_fringe:
                break
            threshold = min_threshold
            fringe = next_fringe

        path = []
        if found:
            node = goal
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            self.get_logger().info("Path found using Fringe Search!")
        else:
            self.get_logger().info("No path found using Fringe Search.")
        grid.draw(path=path, visited=visited, pause_time=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = FringeSearchNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    plt.ioff()        # Turn off interactive mode.
    plt.show()        # Block the execution until the user closes the window.

    rclpy.shutdown()

if __name__ == '__main__':
    main()
