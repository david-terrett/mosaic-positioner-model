# -*- coding utf-8 -*-

from random import random

from geometry import point
from positioner import positioner
from target import target

class focal_plane(object):
    """
    Model of the MOSAIC focal plane
    """

    def __init__(self):
        self.dx = 95.0
        self.dy = 82.3
        self.x_max = 2.0 * self.dx
        self.x_min = -self.x_max;
        self.y_max = 2.0 * self.dy
        self.y_min = -self.y_max;

    # Build the list of positioners
        self.positioners = []
        self._add_positioner(0, 0)
        self._add_positioner(0, 1)
        self._add_positioner(0, -1)
        self._add_positioner(1, 0.5)
        self._add_positioner(1, -0.5)
        self._add_positioner(-1, 0.5)
        self._add_positioner(-1, -0.5)

        self.targets = []


    def add_targets_to_positioners(self, targets):
        for p in self.positioners:
            for t in self.targets:
                p.add_target(t)


    def create_random_targets(self, n):
        for i in range(0, n):
            x = self.x_min + random() * (self.x_max - self.x_min)
            y = self.y_min + random() * (self.y_max - self.y_min)
            self.targets.append(target(x, y))
        self.add_targets_to_positioners(self.targets)

    def plot(self, plt):
        plt.gca().set_aspect('equal')
        for p in self.positioners:
            p.plot(plt)
        for t in self.targets:
            plt.plot(t.position.x(), t.position.y(), 'o', color='black')
        return plt

    def _add_positioner(self, i, j):
        self.positioners.append(positioner(point(i * self.dx, j * self.dy)))
