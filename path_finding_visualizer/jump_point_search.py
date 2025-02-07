#!/usr/bin/env python3
"""
Jump Point Search (JPS) for Path Finding with Persistent Animation.
This node demonstrates a simplified version of JPS with a persistent visualization.
"""

import rclpy
from rclpy.node import Node
import heapq
from path_finding_visualizer.grid import Grid
import time
import matplotlib.pyplot as plt


class JumpPointSearchNode(Node):
    def __init__(self):
        super().__init__('jump_point_search_node')
        self.grid_obj = Grid()
        self.jps_search()

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def jump(self, current, direction, goal, grid):
        next_node = (current[0] + direction[0], current[1] + direction[1])
        if not (0 <= next_node[0] < grid.rows and 0 <= next_node[1] < grid.cols):
            return None
        if grid.grid[next_node[0]][next_node[1]] == 1:
            return None
        if next_node == goal:
            return next_node
        # Check for forced neighbors.
        if direction[0] != 0:  # Vertical movement.
            if (next_node[1] - 1 >= 0 and grid.grid[next_node[0]][next_node[1] - 1] == 1) or \
               (next_node[1] + 1 < grid.cols and grid.grid[next_node[0]][next_node[1] + 1] == 1):
                return next_node
        elif direction[1] != 0:  # Horizontal movement.
            if (next_node[0] - 1 >= 0 and grid.grid[next_node[0] - 1][next_node[1]] == 1) or \
               (next_node[0] + 1 < grid.rows and grid.grid[next_node[0] + 1][next_node[1]] == 1):
                return next_node
        return self.jump(next_node, direction, goal, grid)

    def identify_successors(self, current, grid, goal, open_list, cost_so_far, came_from, visited):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for direction in directions:
            jump_point = self.jump(current, direction, goal, grid)
            if jump_point is not None:
                new_cost = cost_so_far[current] + self.heuristic(current, jump_point)
                if jump_point not in cost_so_far or new_cost < cost_so_far[jump_point]:
                    cost_so_far[jump_point] = new_cost
                    priority = new_cost + self.heuristic(jump_point, goal)
                    heapq.heappush(open_list, (priority, new_cost, jump_point))
                    came_from[jump_point] = current
                    visited.add(jump_point)

    def jps_search(self):
        grid = self.grid_obj
        start = grid.start
        goal = grid.goal

        open_list = []
        heapq.heappush(open_list, (0, 0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        visited = set([start])

        while open_list:
            _, current_cost, current = heapq.heappop(open_list)
            if current == goal:
                break
            self.identify_successors(current, grid, goal, open_list, cost_so_far, came_from, visited)
            grid.draw(visited=visited, pause_time=0.1)
            time.sleep(0.01)

        path = []
        if goal in came_from:
            node = goal
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            self.get_logger().info("Path found using Jump Point Search!")
        else:
            self.get_logger().info("No path found using Jump Point Search.")
        grid.draw(path=path, visited=visited, pause_time=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = JumpPointSearchNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    plt.ioff()        # Turn off interactive mode.
    plt.show()        # Block the execution until the user closes the window.

    rclpy.shutdown()

if __name__ == '__main__':
    main()
