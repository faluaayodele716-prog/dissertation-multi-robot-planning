# Based on the PythonRobotics A* implementation by Atsushi Sakai.
# Adapted for evaluation on a 20x20 warehouse occupancy grid.
import time
import math

class AStarPlanner:
    def __init__(self, grid):
        self.grid = grid
        self.height = len(grid)
        self.width = len(grid[0])

    def plan(self, start, goal):
        t0 = time.perf_counter()
        open_set = {start}
        came_from = {}
        g_score = {start: 0.0}
        f_score = {start: self._heuristic(start, goal)}
        nodes_explored = 0

        while open_set:
            current = min(open_set, key=lambda node: f_score.get(node, float('inf')))
            nodes_explored += 1

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                elapsed = (time.perf_counter() - t0) * 1000.0
                return path, elapsed, True, nodes_explored

            open_set.remove(current)

            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height:
                    if self.grid[neighbor[1]][neighbor[0]] == 1:
                        continue
                    
                    step_cost = math.sqrt(dx*dx + dy*dy)
                    tentative_g = g_score[current] + step_cost

                    if tentative_g < g_score.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                        open_set.add(neighbor)

        elapsed = (time.perf_counter() - t0) * 1000.0
        return [], elapsed, False, nodes_explored

    def _heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])