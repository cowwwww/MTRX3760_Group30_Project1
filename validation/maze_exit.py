"""Geometric exit detector for default_maze; independent of ROS and Gazebo."""
import math


class EastExitDetector:
    # East wall outer face, gap between its top and the north wall inner face.
    PORTAL_X = 2.425
    GAP_MIN_Y = 0.825
    GAP_MAX_Y = 1.575
    OUTSIDE_X = 2.60

    def __init__(self):
        self.previous = None
        self.crossed_portal = False

    def update(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return False
        if self.previous is not None:
            px, py = self.previous
            if px <= self.PORTAL_X < x:
                crossing_y = py + (y-py) * (self.PORTAL_X-px) / (x-px)
                if self.GAP_MIN_Y < crossing_y < self.GAP_MAX_Y:
                    self.crossed_portal = True
        self.previous = (x, y)
        return self.crossed_portal and x > self.OUTSIDE_X
