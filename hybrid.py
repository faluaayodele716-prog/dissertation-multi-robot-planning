# RRT implementation with line-of-sight path shortcutting.
import time
from planners.rrt import RRTPlanner

class RRTShortcutPlanner:
    def __init__(self, grid, max_iter=1000):
        self.rrt = RRTPlanner(grid, max_iter=max_iter)
        self.grid = grid

    def plan(self, start, goal):
        path, elapsed_rrt, success, nodes = self.rrt.plan(start, goal)
        if not success or len(path) <= 2:
            return path, elapsed_rrt, success, nodes

        t0 = time.perf_counter()
        smoothed = [path[0]]
        curr = 0

        while curr < len(path) - 1:
            next_idx = len(path) - 1
            while next_idx > curr + 1:
                if self._line_of_sight(path[curr], path[next_idx]):
                    break
                next_idx -= 1
            smoothed.append(path[next_idx])
            curr = next_idx

        total_elapsed = elapsed_rrt + ((time.perf_counter() - t0) * 1000.0)
        return smoothed, total_elapsed, True, nodes

    def _line_of_sight(self, p1, p2):
        x0, y0 = p1
        x1, y1 = p2
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        x, y = x0, y0
        n = 1 + dx + dy
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1
        error = dx - dy
        dx *= 2
        dy *= 2

        for _ in range(n):
            if not (0 <= x < len(self.grid[0]) and 0 <= y < len(self.grid)) or self.grid[y][x] == 1:
                return False
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx
        return True