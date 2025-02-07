#!/usr/bin/env python3
"""
Greedy Best-First Search for Path Finding with Persistent Animation.
"""

import rclpy
from rclpy.node import Node
import heapq
from path_finding_visualizer.grid import Grid
import time
import matplotlib.pyplot as plt


class GreedyBFSNode(Node):
    def __init__(self):
        super().__init__('greedy_bfs_node')
        self.grid_obj = Grid()
        self.greedy_search()

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def greedy_search(self):
        grid = self.grid_obj
        start = grid.start
        goal = grid.goal

        frontier = []
        heapq.heappush(frontier, (self.heuristic(start, goal), start))
        came_from = {start: None}
        visited = set()

        while frontier:
            _, current = heapq.heappop(frontier)
            visited.add(current)
            if current == goal:
                break
            for neighbor in grid.get_neighbors(current):
                if neighbor not in visited and neighbor not in [item[1] for item in frontier]:
                    heapq.heappush(frontier, (self.heuristic(neighbor, goal), neighbor))
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
            self.get_logger().info("Path found using Greedy Best-First Search!")
        else:
            self.get_logger().info("No path found using Greedy Best-First Search.")
        grid.draw(path=path, visited=visited, pause_time=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = GreedyBFSNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    plt.ioff()        # Turn off interactive mode.
    plt.show()        # Block the execution until the user closes the window.

    rclpy.shutdown()

if __name__ == '__main__':
    main()
