# UAV Autonomous Path Planning using A* Algorithm
## Technical Report & Implementation Documentation

---

## 1. Introduction

This document presents the implementation of the A* (A-Star) pathfinding algorithm for autonomous UAV navigation in a 2D grid environment. The UAV must determine an optimal path from a starting position (S) to a target position (G) while avoiding obstacles (#), using only four-directional movement (Up, Down, Left, Right) with a uniform cost of 1 per move.

---

## 2. Theoretical Background: A* Algorithm

### 2.1 What is A*?

A* (pronounced "A-Star") is a widely-used **informed search algorithm** that finds the shortest path between nodes in a graph. It was first described by Peter Hart, Nils Nilsson, and Bertram Raphael in 1968. A* combines the strengths of:

- **Dijkstra's Algorithm**: Guarantees the shortest path by tracking actual cost from start
- **Greedy Best-First Search**: Uses a heuristic to guide search toward the goal efficiently

### 2.2 Core Components

A* operates on a search tree where each **node** represents a state (grid cell) and **edges** represent valid movements between adjacent cells.

#### Cost Function: f(n) = g(n) + h(n)

| Component | Description | Formula |
|-----------|-------------|---------|
| **g(n)** | Actual cost from start node to current node n | g(n) = g(parent) + cost(move) |
| **h(n)** | Heuristic estimate of cost from n to goal | Depends on chosen heuristic |
| **f(n)** | Total estimated cost of path through n | f(n) = g(n) + h(n) |

### 2.3 Algorithm Steps

```
1. Initialize OPEN list (priority queue) with start node
2. Initialize CLOSED list (empty set)
3. While OPEN is not empty:
   a. Remove node with lowest f-value from OPEN -> current
   b. If current is goal: reconstruct and return path
   c. Add current to CLOSED
   d. For each valid neighbor of current:
      i.   Skip if in CLOSED or is obstacle
      ii.  Calculate g, h, f for neighbor
      iii. If neighbor not in OPEN or has better g:
           - Set parent to current
           - Add/update in OPEN
4. If OPEN empty and goal not found: NO PATH EXISTS
```

### 2.4 Properties of A*

| Property | Condition | Guarantee |
|----------|-----------|-----------|
| **Completeness** | Finite branching factor, positive edge costs | YES - always finds solution if one exists |
| **Optimality** | Heuristic is **admissible** | YES - finds lowest-cost path |
| **Optimality** | Heuristic is **consistent** | YES - no node re-expansion needed |

---

## 3. Heuristic Function Selection

### 3.1 Why Manhattan Distance?

For this implementation, we selected the **Manhattan Distance** (L1 norm) as the heuristic:

```
h(n) = |x_current - x_goal| + |y_current - y_goal|
```

### 3.2 Justification for 4-Directional Movement

The Manhattan Distance is the **ideal heuristic** for this problem for three critical reasons:

#### a) Admissibility
The Manhattan distance never overestimates the true cost. With only Up/Down/Left/Right movement, the minimum number of moves to reach any goal is exactly the sum of horizontal and vertical displacements. Since each move costs 1, the heuristic equals the true minimum cost.

**Example**: From (0,0) to (3,4):
- Manhattan distance = |3-0| + |4-0| = 7
- Minimum actual moves = 7 (3 down + 4 right)
- Therefore h(n) <= actual cost  **(ADMISSIBLE)**

#### b) Consistency (Monotonicity)
For any node n and its successor n':
```
h(n) <= cost(n, n') + h(n')
```
Since each valid move changes exactly one coordinate by 1, the Manhattan distance decreases by exactly 1. With cost(n, n') = 1:
```
|dx| + |dy| <= 1 + (|dx|-1 + |dy|)  [or vice versa]
```
This always holds, making the heuristic **consistent**.

#### c) Why NOT Euclidean Distance?
Euclidean distance (straight-line) would be **inadmissible** for 4-directional grids:
- Euclidean from (0,0) to (1,1) = sqrt(2) ~ 1.414
- But actual cost with 4-directional movement = 2
- Since 1.414 < 2, it IS admissible... but it's a **loose lower bound**
- More importantly, if diagonal movement were allowed, Euclidean would be exact. But since it's NOT allowed, Manhattan is tighter and more informative.

### 3.3 Heuristic Comparison Table

| Heuristic | Formula | Admissible? | Consistent? | Suitable for 4-dir? |
|-----------|---------|-------------|-------------|---------------------|
| Manhattan | |dx| + |dy| | YES | YES | **YES - Best** |
| Euclidean | sqrt(dx^2 + dy^2) | YES | YES | YES (but looser) |
| Chebyshev | max(|dx|, |dy|) | NO | NO | NO |
| Zero | 0 | YES | YES | YES (becomes Dijkstra) |

---

## 4. Implementation Details

### 4.1 Data Structures

| Structure | Purpose | Python Type |
|-----------|---------|-------------|
| Priority Queue (OPEN) | Store nodes to explore, ordered by f-value | `heapq` (min-heap) |
| Hash Set (CLOSED) | Track expanded nodes | `set()` |
| Dictionary (open_dict) | O(1) lookup of nodes in OPEN by position | `dict` |
| Node Class | Store position, parent, g, h, f values | Custom class |

### 4.2 Design Decisions

1. **Priority Queue with Stale Entry Handling**: When a better path to an existing open node is found, we push the updated node rather than decreasing the key in-place. Stale entries (with old f-values) are skipped when popped.

2. **Tie-Breaking**: When two nodes have equal f-values, we prefer the one with lower h-value (closer to goal), which typically reduces the number of explored nodes.

3. **Path Reconstruction**: After reaching the goal, we backtrack through parent pointers to reconstruct the full path from start to goal.

### 4.3 Complexity Analysis

| Aspect | Complexity | Explanation |
|--------|-----------|-------------|
| Time (worst) | O(b^d) | b = branching factor (~4), d = solution depth |
| Time (typical) | Much better | Heuristic prunes vast portions of search space |
| Space | O(b^d) | Stores all generated nodes |
| Per-node cost | O(log N) | Heap operations |

---

## 5. Test Cases and Observations

### Test Case 1: Simple Path (Assignment Sample)
```
S . . . . . . .
. . # # # . . .
. . . . # . . .
. # . . # . . .
. # . . . . . .
. . . . . . . G
```
- **Path Found**: YES
- **Path Cost**: 12
- **Nodes Explored**: 13
- **Observation**: The algorithm efficiently finds the path along the bottom edge. The heuristic guides it directly toward the goal with minimal backtracking. The explored region is tightly concentrated around the optimal path.

### Test Case 2: Multiple Possible Paths
```
S . . . . . . .
. . . . . . . .
. . # . . # . .
. . # . . # . .
. . . . . . . .
. . . . . . . G
```
- **Path Found**: YES
- **Path Cost**: 12
- **Nodes Explored**: 13
- **Observation**: With multiple equivalent paths available, A* selects one based on tie-breaking rules (lower h-value first). The algorithm does not explore all possible paths - it stops once the optimal path is confirmed.

### Test Case 3: Narrow Passage
```
S . . # . . . .
. . . # . . . .
. . . # . . . .
. . . . . . . .
. . . # . . . .
. . . # . . . G
```
- **Path Found**: YES
- **Path Cost**: 12
- **Nodes Explored**: 19
- **Observation**: The narrow passage forces the algorithm to explore more nodes. The wall creates a detour, and A* must verify that going around is indeed the best option. More nodes are explored compared to open environments.

### Test Case 4: Maze-like Obstacles
```
S # . . . . . .
. # . # # # # .
. . . . . . # .
. # # # # . # .
. . . . . . . .
# # # # # # . G
```
- **Path Found**: YES
- **Path Cost**: 12
- **Nodes Explored**: 13
- **Observation**: Despite the maze-like structure, the heuristic effectively guides the search. The algorithm quickly identifies the corridor along the left side and bottom as the viable route.

### Test Case 5: No Path Exists
```
S . . # . . . .
. . . # . . . .
# # # # # # # .
. . . # . . . .
. . . # . . . .
. . . # . . . G
```
- **Path Found**: NO
- **Path Cost**: N/A
- **Nodes Explored**: 6
- **Observation**: A* quickly determines no path exists after exploring only the reachable region. The wall completely separates start from goal. This demonstrates A*'s completeness - it terminates correctly when no solution exists.

### Test Case 6: Complex Environment
```
S . # . . . . .
. . # . # . . .
. . . . # . # .
. # # . . . # .
. . # . # . . .
. . . . # . . G
```
- **Path Found**: YES
- **Path Cost**: 12
- **Nodes Explored**: 21
- **Observation**: The complex obstacle arrangement requires the most exploration. The algorithm must navigate around multiple obstacles, causing the search frontier to expand in several directions before converging on the optimal path.

---

## 6. Summary Table

| Test Case | Name | Path Found | Cost | Nodes Explored | Time (s) |
|-----------|------|------------|------|----------------|----------|
| 1 | Simple Path | YES | 12 | 13 | 0.000061 |
| 2 | Multiple Paths | YES | 12 | 13 | 0.000062 |
| 3 | Narrow Passage | YES | 12 | 19 | 0.000082 |
| 4 | Maze-like | YES | 12 | 13 | 0.000043 |
| 5 | No Path | NO | N/A | 6 | 0.000042 |
| 6 | Complex | YES | 12 | 21 | 0.000088 |

---

## 7. Assumptions and Limitations

### Assumptions
1. The grid is rectangular (all rows have equal length)
2. Exactly one 'S' (start) and one 'G' (goal) exist in each grid
3. Movement is restricted to 4 directions (no diagonals)
4. Uniform movement cost: every valid move costs exactly 1
5. Obstacles are static (non-moving)

### Limitations
1. **Memory Usage**: For very large grids, the open and closed lists can consume significant memory
2. **Grid Resolution**: Real-world UAVs operate in continuous space; grid discretization is a simplification
3. **Dynamic Obstacles**: The current implementation does not handle moving obstacles
4. **No Path Smoothing**: The output path consists of grid-aligned segments; real UAVs may need smoothed trajectories
5. **Single Agent**: Does not handle multi-UAV collision avoidance

---

## 8. Conclusion

The A* algorithm successfully solves the UAV path planning problem across diverse test environments. The Manhattan distance heuristic proves to be both admissible and consistent for 4-directional grid movement, guaranteeing optimal paths while minimizing unnecessary exploration. The implementation correctly handles cases with simple paths, multiple paths, narrow passages, complex mazes, and impossible scenarios.

Key takeaways:
- **A* is optimal** when using an admissible heuristic
- **Manhattan distance is the ideal heuristic** for 4-directional uniform-cost grids
- **The algorithm is complete** - it always finds a path if one exists, or correctly reports failure
- **Visualization is crucial** for understanding search behavior and validating correctness

---

*End of Document*
