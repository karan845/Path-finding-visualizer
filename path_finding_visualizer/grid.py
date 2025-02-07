#!/usr/bin/env python3
"""
Module for creating and visualizing a 40x40 grid with obstacles.
This Grid class is used by path-finding algorithms and supports a persistent
Matplotlib window that updates in place with vibrant, custom colors and
enhanced aesthetics.
"""

import matplotlib.pyplot as plt
import numpy as np
import random
from matplotlib.colors import ListedColormap

class Grid:
    def __init__(self, rows=40, cols=40, obstacle_prob=0.2, seed=42):
        """
        Initialize the grid.
        
        Parameters:
          rows, cols: Dimensions of the grid (default 40x40).
          obstacle_prob: Probability (0–1) that a given cell is an obstacle.
          seed: Random seed for reproducibility.
          
        The start and goal are set to be away from the edges.
          - start is at (rows/4, cols/4)
          - goal is at (3*rows/4, 3*cols/4)
        """
        self.rows = rows
        self.cols = cols
        random.seed(seed)
        
        # Create a grid (2D list) initialized to 0 (free space).
        self.grid = [[0 for _ in range(cols)] for _ in range(rows)]
        # Randomly add obstacles.
        for i in range(rows):
            for j in range(cols):
                if random.random() < obstacle_prob:
                    self.grid[i][j] = 1  # 1 represents an obstacle.
        
        # Set start and goal positions away from the edges.
        self.start = (rows // 4, cols // 4)         # e.g., (10, 10) for 40x40
        self.goal  = (3 * rows // 4, 3 * cols // 4)   # e.g., (30, 30) for 40x40
        # Ensure start and goal cells are free.
        self.grid[self.start[0]][self.start[1]] = 0
        self.grid[self.goal[0]][self.goal[1]] = 0

        # Set up a persistent Matplotlib figure for animation.
        plt.ion()  # Enable interactive mode.
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        
        # Set a dark background for contrast.
        self.ax.set_facecolor('#222222')
        
        # Optionally, add subtle grid lines for better visual segmentation.
        self.ax.grid(True, color='#444444', linestyle='--', linewidth=0.5)
        
        # Create an image handle for the animation (initially None).
        self.im = None

    def get_neighbors(self, node):
        """
        Get all valid neighboring cells (up, down, left, right) that are free.
        
        Parameters:
          node: A tuple (row, col).
          
        Returns:
          A list of neighboring node tuples that are not obstacles.
        """
        neighbors = []
        row, col = node
        # Define 4-connected neighbor directions.
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for d in directions:
            new_row = row + d[0]
            new_col = col + d[1]
            if 0 <= new_row < self.rows and 0 <= new_col < self.cols:
                if self.grid[new_row][new_col] == 0:
                    neighbors.append((new_row, new_col))
        return neighbors

    def draw(self, path=None, visited=None, pause_time=0.2):
        """
        Update the persistent grid visualization with vibrant colors.
        
        Coloring scheme:
          - 0 (free space): white (#FFFFFF)
          - 1 (obstacle): black (#000000)
          - 2 (visited): bright cyan (#00FFFF)
          - 3 (final path): bright blue (#0000FF)
          - 4 (start): neon green (#00FF00)
          - 5 (goal): neon red (#FF0000)
        
        Parameters:
          path: List of nodes representing the final path.
          visited: Set of nodes that have been visited.
          pause_time: Duration (in seconds) to pause for the update (default 0.2 sec).
        """
        # Create a copy of the grid as a NumPy array for visualization.
        visual = np.array(self.grid)
        
        # Mark visited cells (only if the cell is free).
        if visited:
            for (r, c) in visited:
                if visual[r][c] == 0:
                    visual[r][c] = 2  # 2 denotes visited.
        
        # Mark the final path cells.
        if path:
            for (r, c) in path:
                if visual[r][c] not in (4, 5):  # Do not override start/goal.
                    visual[r][c] = 3  # 3 denotes the final path.
        
        # Mark start and goal positions.
        visual[self.start[0]][self.start[1]] = 4  # 4 denotes start.
        visual[self.goal[0]][self.goal[1]] = 5      # 5 denotes goal.
        
        # Define a custom vibrant colormap.
        # Order corresponds to: free space, obstacle, visited, final path, start, goal.
        cmap = ListedColormap(['#FFFFFF', '#000000', '#00FFFF', '#0000FF', '#00FF00', '#FF0000'])
        
        # If no image has been drawn yet, create one; otherwise, update the image data.
        if self.im is None:
            self.im = self.ax.imshow(visual, cmap=cmap)
            self.ax.set_title("Vibrant Path Finding Visualization", fontsize=16, color='#EEEEEE')
            # Remove axis ticks for a cleaner look.
            self.ax.set_xticks([])
            self.ax.set_yticks([])
        else:
            self.im.set_data(visual)
        
        # Redraw the canvas to update the display.
        self.fig.canvas.draw_idle()
        plt.pause(pause_time)
