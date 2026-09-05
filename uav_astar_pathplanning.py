"""
UAV Autonomous Path Planning using A* (A-Star) Algorithm
=========================================================
This program implements the A* pathfinding algorithm for a UAV operating
in a 2D grid environment. The UAV must navigate from a start position (S)
to a target position (G) while avoiding obstacles (#).

Author: UAV Path Planning Assignment
Language: Python 3
"""

import time
import heapq
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Tuple, Optional, Set, Dict


class Node:
    """
    Represents a node in the A* search tree.

    Each node stores:
    - position: (row, col) coordinate in the grid
    - parent: reference to the predecessor node (for path reconstruction)
    - g: actual cost from start node to this node
    - h: heuristic estimate from this node to goal
    - f: total estimated cost (f = g + h)
    """
    def __init__(self, position: Tuple[int, int], parent: Optional['Node'] = None):
        self.position = position
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)

    def __lt__(self, other):
        """For priority queue ordering. Tie-breaker uses h (closer to goal first)."""
        return self.f < other.f or (self.f == other.f and self.h < other.h)


def manhattan_distance(a: Tuple[int, int], b: Tuple[int, int]) -> int:
    """
    Manhattan Distance Heuristic.

    h(n) = |x1 - x2| + |y1 - y2|

    Why Manhattan Distance for 4-directional movement?
    --------------------------------------------------
    1. ADMISSIBLE: It never overestimates the actual cost because with only
       Up/Down/Left/Right movement, the minimum number of moves to reach the
       goal is exactly the sum of horizontal and vertical distances.

    2. CONSISTENT (MONOTONIC): For any node n and its successor n',
       h(n) <= cost(n, n') + h(n'). Since each move changes either row or col
       by 1, the Manhattan distance decreases by exactly 1 per valid move.
       This guarantees A* will find the optimal path without re-processing nodes.

    3. EFFICIENT: O(1) computation per node.

    Note: Euclidean distance would be INADMISSIBLE here because the straight-line
    distance can be less than the actual grid path cost (e.g., diagonal would be
    sqrt(2) but we can only move in 4 directions, so actual cost is 2).
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid: List[List[str]], start: Tuple[int, int], goal: Tuple[int, int]) -> Tuple[
    Optional[List[Tuple[int, int]]],  # path (ordered coordinates)
    int,                               # total path cost
    int,                               # nodes explored
    float,                             # execution time in seconds
    Set[Tuple[int, int]]               # all explored nodes
]:
    """
    A* Pathfinding Algorithm Implementation.

    Algorithm Overview:
    -------------------
    A* maintains two sets:
    - OPEN LIST: Nodes to be evaluated (priority queue ordered by f = g + h)
    - CLOSED LIST: Nodes already evaluated

    At each step:
    1. Pop node with lowest f from open list
    2. If it's the goal, reconstruct and return path
    3. Otherwise, generate all valid neighbors
    4. For each neighbor, calculate g, h, and f
    5. If neighbor is not in open/closed or has better g, add/update it

    Time Complexity: O(b^d) worst case, but heuristic-guided search is much faster
    Space Complexity: O(b^d) for storing nodes
    where b = branching factor, d = depth of solution
    """
    start_time = time.perf_counter()

    rows, cols = len(grid), len(grid[0])

    # Four-directional movement: Up, Down, Left, Right
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    # Initialize start node
    start_node = Node(start)
    start_node.g = 0
    start_node.h = manhattan_distance(start, goal)
    start_node.f = start_node.g + start_node.h

    # Priority queue for open list (min-heap by f-value)
    open_list: List[Node] = []
    heapq.heappush(open_list, start_node)

    # Closed set: positions already expanded
    closed_set: Set[Tuple[int, int]] = set()

    # Track all explored nodes for visualization
    explored_nodes: Set[Tuple[int, int]] = set()

    # Dictionary for O(1) lookup of nodes in open list by position
    open_dict: Dict[Tuple[int, int], Node] = {start: start_node}

    nodes_explored = 0

    while open_list:
        # Get node with minimum f-value
        current_node = heapq.heappop(open_list)

        # Skip if already processed (stale entry in heap)
        if current_node.position in closed_set:
            continue

        # Mark as expanded
        closed_set.add(current_node.position)
        explored_nodes.add(current_node.position)
        nodes_explored += 1

        # Goal test
        if current_node.position == goal:
            # Reconstruct path by backtracking through parents
            path = []
            node = current_node
            while node:
                path.append(node.position)
                node = node.parent
            path.reverse()

            execution_time = time.perf_counter() - start_time
            return path, current_node.g, nodes_explored, execution_time, explored_nodes

        # Generate successors (neighbors)
        for dr, dc in directions:
            new_row = current_node.position[0] + dr
            new_col = current_node.position[1] + dc
            neighbor_pos = (new_row, new_col)

            # Boundary check
            if new_row < 0 or new_row >= rows or new_col < 0 or new_col >= cols:
                continue

            # Obstacle check
            if grid[new_row][new_col] == '#':
                continue

            # Skip if already evaluated
            if neighbor_pos in closed_set:
                continue

            # Create neighbor node
            neighbor = Node(neighbor_pos, current_node)
            neighbor.g = current_node.g + 1  # Uniform movement cost
            neighbor.h = manhattan_distance(neighbor_pos, goal)
            neighbor.f = neighbor.g + neighbor.h

            # Check if neighbor is already in open list with a better path
            if neighbor_pos in open_dict:
                existing = open_dict[neighbor_pos]
                if neighbor.g >= existing.g:
                    continue  # Existing path is better or equal
                # Found a better path to this node - update it
                existing.g = neighbor.g
                existing.f = neighbor.f
                existing.parent = current_node
                # Push updated node (old entry becomes stale, skipped later)
                heapq.heappush(open_list, existing)
            else:
                open_dict[neighbor_pos] = neighbor
                heapq.heappush(open_list, neighbor)

    # Open list empty and goal not reached -> no path exists
    execution_time = time.perf_counter() - start_time
    return None, 0, nodes_explored, execution_time, explored_nodes


def visualize_grid(grid: List[List[str]], path: Optional[List[Tuple[int, int]]], 
                   explored: Set[Tuple[int, int]], title: str, save_path: str):
    """
    Visualize the grid environment, explored region, and final path.

    Color Legend:
    - White: Traversable cells
    - Dark Blue: Obstacles
    - Light Blue: Explored nodes
    - Red: Final path
    - Green: Start position (S)
    - Orange: Goal position (G)
    """
    rows, cols = len(grid), len(grid[0])

    # Visualization matrix: 0=empty, 1=obstacle, 2=explored, 3=path, 4=start, 5=goal
    viz = np.zeros((rows, cols))

    # Mark obstacles
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == '#':
                viz[r][c] = 1

    # Mark explored nodes
    for r, c in explored:
        if viz[r][c] == 0:
            viz[r][c] = 2

    # Mark path (excluding start and goal for special coloring)
    if path:
        for r, c in path:
            if (r, c) != path[0] and (r, c) != path[-1]:
                viz[r][c] = 3

    # Mark start and goal
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 'S':
                viz[r][c] = 4
            elif grid[r][c] == 'G':
                viz[r][c] = 5

    fig, ax = plt.subplots(figsize=(max(8, cols * 0.8), max(6, rows * 0.8)))

    # Custom discrete colormap
    colors = ['#ffffff', '#2c3e50', '#5dade2', '#e74c3c', '#27ae60', '#f39c12']
    cmap = plt.matplotlib.colors.ListedColormap(colors)
    bounds = [-0.5, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5]
    norm = plt.matplotlib.colors.BoundaryNorm(bounds, cmap.N)

    ax.imshow(viz, cmap=cmap, norm=norm, interpolation='nearest')

    # Grid lines
    ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
    ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5)
    ax.tick_params(which='minor', size=0)
    ax.set_xticks([])
    ax.set_yticks([])

    # Text labels for S and G
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 'S':
                ax.text(c, r, 'S', ha='center', va='center', fontsize=14, 
                       fontweight='bold', color='white')
            elif grid[r][c] == 'G':
                ax.text(c, r, 'G', ha='center', va='center', fontsize=14, 
                       fontweight='bold', color='white')

    # Legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='#ffffff', edgecolor='gray', label='Traversable'),
        Patch(facecolor='#2c3e50', edgecolor='gray', label='Obstacle'),
        Patch(facecolor='#5dade2', edgecolor='gray', label='Explored'),
        Patch(facecolor='#e74c3c', edgecolor='gray', label='Path'),
        Patch(facecolor='#27ae60', edgecolor='gray', label='Start'),
        Patch(facecolor='#f39c12', edgecolor='gray', label='Goal'),
    ]
    ax.legend(handles=legend_elements, loc='upper center', bbox_to_anchor=(0.5, -0.05),
             ncol=6, fontsize=10)

    ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    print(f"  [SAVED] Visualization: {save_path}")


def find_positions(grid: List[List[str]]) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    """Locate Start (S) and Goal (G) positions in the grid."""
    start = goal = None
    for r, row in enumerate(grid):
        for c, cell in enumerate(row):
            if cell == 'S':
                start = (r, c)
            elif cell == 'G':
                goal = (r, c)
    if start is None or goal is None:
        raise ValueError("Grid must contain both 'S' (start) and 'G' (goal)")
    return start, goal


def run_test_case(name: str, grid_str: str, case_num: int, output_dir: str = "."):
    """
    Execute a single test case: parse grid, run A*, print results, save visualization.
    """
    print(f"\n{'='*60}")
    print(f"TEST CASE {case_num}: {name}")
    print(f"{'='*60}")

    # Parse grid string into 2D list (remove spaces used for readability)
    grid = [list(row.replace(' ', '')) for row in grid_str.strip().split('\n')]

    print("Grid Layout:")
    for row in grid:
        print('  ' + ''.join(row))

    start, goal = find_positions(grid)
    print(f"\nStart Position: {start}")
    print(f"Goal Position:  {goal}")

    # Execute A* algorithm
    path, cost, explored_count, exec_time, explored = astar(grid, start, goal)

    print(f"\n--- RESULTS ---")
    if path:
        print(f"Path Found: YES")
        print(f"Path Coordinates: {path}")
        print(f"Total Path Cost: {cost}")
    else:
        print(f"Path Found: NO")
        print(f"Path Coordinates: None")
        print(f"Total Path Cost: N/A")

    print(f"Nodes Explored: {explored_count}")
    print(f"Execution Time: {exec_time:.6f} seconds")

    # Generate and save visualization
    safe_name = name.lower().replace(" ", "_").replace("(", "").replace(")", "")
    save_path = f"{output_dir}/test_case_{case_num}_{safe_name}.png"
    title = f"Test Case {case_num}: {name}"
    visualize_grid(grid, path, explored, title, save_path)

    return {
        'case': case_num,
        'name': name,
        'path_found': path is not None,
        'path': path,
        'cost': cost,
        'explored': explored_count,
        'time': exec_time
    }


def main():
    """Main execution: run all test cases and print summary."""

    print("=" * 60)
    print("UAV AUTONOMOUS PATH PLANNING - A* ALGORITHM")
    print("=" * 60)
    print("\nThis program implements A* pathfinding for a UAV in a 2D grid.")
    print("Movement: Up, Down, Left, Right (cost = 1 per move)")
    print("Heuristic: Manhattan Distance (admissible for 4-directional grids)")

    # ============================================================
    # TEST CASE DEFINITIONS
    # ============================================================

    # Test Case 1: Simple Path (Assignment Sample)
    grid1 = """S . . . . . . .
. . # # # . . .
. . . . # . . .
. # . . # . . .
. # . . . . . .
. . . . . . . G"""

    # Test Case 2: Multiple Possible Paths
    grid2 = """S . . . . . . .
. . . . . . . .
. . # . . # . .
. . # . . # . .
. . . . . . . .
. . . . . . . G"""

    # Test Case 3: Narrow Passage
    grid3 = """S . . # . . . .
. . . # . . . .
. . . # . . . .
. . . . . . . .
. . . # . . . .
. . . # . . . G"""

    # Test Case 4: Maze-like Obstacles
    grid4 = """S # . . . . . .
. # . # # # # .
. . . . . . # .
. # # # # . # .
. . . . . . . .
# # # # # # . G"""

    # Test Case 5: No Path Exists (blocked by wall)
    grid5 = """S . . # . . . .
. . . # . . . .
# # # # # # # .
. . . # . . . .
. . . # . . . .
. . . # . . . G"""

    # Test Case 6: Complex Environment
    grid6 = """S . # . . . . .
. . # . # . . .
. . . . # . # .
. # # . . . # .
. . # . # . . .
. . . . # . . G"""

    test_cases = [
        ("Simple Path (Assignment Sample)", grid1),
        ("Multiple Possible Paths", grid2),
        ("Narrow Passage", grid3),
        ("Maze-like Obstacles", grid4),
        ("No Path Exists", grid5),
        ("Complex Environment", grid6),
    ]

    results = []
    output_dir = "."

    for i, (name, grid_str) in enumerate(test_cases, 1):
        result = run_test_case(name, grid_str, i, output_dir)
        results.append(result)

    # Print summary table
    print(f"\n{'='*60}")
    print("SUMMARY OF ALL TEST CASES")
    print(f"{'='*60}")
    print(f"{'Case':<6} {'Name':<30} {'Found':<8} {'Cost':<8} {'Explored':<10} {'Time (s)':<12}")
    print("-" * 80)
    for r in results:
        print(f"{r['case']:<6} {r['name']:<30} "
              f"{'YES' if r['path_found'] else 'NO':<8} "
              f"{r['cost'] if r['path_found'] else 'N/A':<8} "
              f"{r['explored']:<10} {r['time']:<12.6f}")

    print(f"\nAll visualizations saved as PNG files in: {output_dir}/")
    print("\nA* Algorithm Implementation Complete!")


if __name__ == "__main__":
    main()
