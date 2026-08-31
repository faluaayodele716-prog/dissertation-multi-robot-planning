# Based on the PathPlanning RRT implementation by zhm-real.
# Adapted for discrete space-time grid constraints.
import time
import math
import random

class RRTPlanner:
    def __init__(self, grid, max_iter=1000, step_size=1.0):
        self.grid = grid
        self.height = len(grid)
        self.width = len(grid[0])
        self.max_iter = max_iter
        self.step_size = step_size

    def plan(self, start, goal):
        t0 = time.perf_counter()
        tree = {start: None}

        for _ in range(self.max_iter):
            rnd = goal if random.random() < 0.2 else (
                random.randint(0, self.width - 1),
                random.randint(0, self.height - 1)
            )

            nearest = min(tree.keys(), key=lambda n: (n[0]-rnd[0])**2 + (n[1]-rnd[1])**2)
            dx, dy = rnd[0] - nearest[0], rnd[1] - nearest[1]
            dist = math.hypot(dx, dy)
            if dist == 0:
                continue

            nx = int(round(nearest[0] + (dx / dist) * self.step_size))
            ny = int(round(nearest[1] + (dy / dist) * self.step_size))

            if 0 <= nx < self.width and 0 <= ny < self.height and self.grid[ny][nx] == 0:
                if (nx, ny) not in tree:
                    tree[(nx, ny)] = nearest
                    if (nx, ny) == goal:
                        path, curr = [], goal
                        while curr is not None:
                            path.append(curr)
                            curr = tree[curr]
                        path.reverse()
                        elapsed = (time.perf_counter() - t0) * 1000.0
                        return path, elapsed, True, len(tree)

        elapsed = (time.perf_counter() - t0) * 1000.0
        return [], elapsed, False, len(tree)