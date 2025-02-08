# Path Finding Visualizer

**Path Finding Visualizer** is a ROS2 package that demonstrates ten popular path-finding algorithms through vibrant, animated visualizations on a grid with obstacles. The package uses Python, rclpy, and Matplotlib to display the progress of each algorithm in real time on a 40×40 grid. Start and goal positions are placed away from the grid edges to provide a clear demonstration of each algorithm's capabilities.

## Features

- **Multiple Algorithms:**  
  - Dijkstra's Algorithm  
  - A* Algorithm  
  - Breadth-First Search (BFS)  
  - Depth-First Search (DFS)  
  - Greedy Best-First Search  
  - Uniform Cost Search  
  - Bidirectional Search  
  - Jump Point Search  
  - Recursive Best First Search  
  - Fringe Search

- **Enhanced Visualization:**  
  - The grid is 40×40 with randomly placed obstacles.  
  - Start and goal positions are not located at the edges (e.g., start at (10, 10) and goal at (30, 30) for a 40×40 grid).  
  - A persistent Matplotlib window with vibrant, custom colors is used for animation.  
  - A custom colormap provides a vibrant visual experience with a dark background and subtle grid lines for improved contrast.
  - The display updates every 0.2 seconds to create a smooth animation of the algorithm’s progress.

- **ROS2 Integration:**  
  - Easily run any algorithm using the `ros2 run` command.
  - Each algorithm is implemented in its own Python node for modularity.

## Project Structure

```
path_finding_visualizer/
├── package.xml
├── setup.py
└── path_finding_visualizer/
    ├── __init__.py
    ├── grid.py                  # Grid generation and visualization (40x40 grid with custom vibrant colormap)
    ├── dijkstra.py              # Dijkstra's Algorithm
    ├── astar.py                 # A* Algorithm
    ├── bfs.py                   # Breadth-First Search
    ├── dfs.py                   # Depth-First Search
    ├── greedy_bfs.py            # Greedy Best-First Search
    ├── uniform_cost.py          # Uniform Cost Search
    ├── bidirectional.py         # Bidirectional Search
    ├── jump_point_search.py     # Jump Point Search (simplified)
    ├── recursive_best_first_search.py  # Recursive Best First Search
    └── fringe_search.py         # Fringe Search
```

## Installation

### Prerequisites

- **ROS2** (e.g., Humble or Foxy) installed on Linux.
- **Python 3** and **Matplotlib** (install via pip if necessary):
  ```bash
  pip install matplotlib
  ```

### Build Instructions

1. **Create a ROS2 Workspace (if needed):**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```
2. **Clone the Repository:**
   ```bash
   git clone https://github.com/karan845/Path-finding-visualizer.git
   ```
3. **Build the Package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select path_finding_visualizer
   ```
4. **Source the Workspace:**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

Run any algorithm node individually using the `ros2 run` command. For example:

- **Dijkstra's Algorithm:**
  ```bash
  ros2 run path_finding_visualizer dijkstra
  ```
- **A* Algorithm:**
  ```bash
  ros2 run path_finding_visualizer astar
  ```
- **Recursive Best First Search (rbfs):**
  ```bash
  ros2 run path_finding_visualizer rbfs
  ```
- Similarly, you can run `bfs`, `dfs`, `greedy_bfs`, `uniform_cost`, `bidirectional`, `jump_point_search`, and `fringe_search`.

After launching any node, a persistent Matplotlib window will appear and update the grid animation in real time. Once the algorithm completes, the window remains open so you can inspect the final path and grid state.

## Algorithm Descriptions

- **Dijkstra's Algorithm:** Uses a priority queue to find the shortest path.
- **A* Algorithm:** Combines cost and heuristic estimates for efficient path planning.
- **Breadth-First Search (BFS):** Explores neighbors level by level.
- **Depth-First Search (DFS):** Uses recursion or a stack to traverse the grid.
- **Greedy Best-First Search:** Selects the next node based solely on the heuristic.
- **Uniform Cost Search:** Similar to Dijkstra's in an unweighted grid.
- **Bidirectional Search:** Simultaneously searches from the start and goal until they meet.
- **Jump Point Search:** A simplified version of an optimized search that "jumps" over nodes.
- **Recursive Best First Search:** Implements recursion with a heuristic, updating dynamically.
- **Fringe Search:** Gradually expands the frontier with a simplified approach.

## Customization

- **Grid Parameters:** Modify the grid dimensions, obstacle probability, or random seed in `grid.py`.
- **Colormap and Aesthetics:** Adjust the custom colormap colors, background, and grid line settings to change the look and feel.
- **Animation Timing:** Change the `pause_time` parameter (default 0.2 sec) in `grid.py` to speed up or slow down the animation.

## Contributing

Contributions, issues, and suggestions are welcome! Please feel free to open an issue or submit a pull request with improvements or bug fixes.

## License

This project is licensed under the [Apache License 2.0](LICENSE).
