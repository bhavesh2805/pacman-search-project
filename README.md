# pacman-search-project
Implemented classical AI search algorithms and heuristics using the Pacman framework.
# AI Search Algorithms – Pacman Project

This project demonstrates the implementation of classical AI search algorithms
and heuristic design using the Pacman framework.

## Features
- Depth-First Search (DFS)
- Breadth-First Search (BFS)
- Uniform Cost Search (UCS)
- A* Search with admissible and consistent heuristics
- Corner visiting problem (multi-goal search)
- Food collection problem
- Greedy closest-dot search agent (suboptimal but fast)

## Key Concepts
- State-space search
- Admissible and consistent heuristics
- Graph search vs tree search
- Performance optimization using heuristics

## Technologies
- Python
- AI Search Algorithms
- Heuristic Optimization

## How to Run
```bash
python pacman.py -l mediumCorners -p AStarCornersAgent
python pacman.py -l bigSearch -p ClosestDotSearchAgent
