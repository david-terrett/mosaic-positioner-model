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

    # empty list of targets
        self.targets = []


    def add_targets_to_positioners(self, targets):
        """
        Add targets to the positioners

        Parameters
        ----------
        targets : [target]
            List of targets
        """
        for p in self.positioners:
            for t in self.targets:
                p.add_target(t)


    def clear_targets(self):
        """
        Remove targets from all positioners
        """
        for pos in self.positioners:
            pos.clear_targets()


    def clear_target_assignments(self):
        """
        Clear the target assignments for all positioners
        """
        for pos in self.positioners:
            pos.assign_target(None)


    def create_random_targets(self, n):
        """
        Create random targets and add them to the positioners

        Parameters
        ---------
        n : int
            Number of targets to create
        """
        self.targets = []
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


    def simple_allocator(self):
        """
        Simple target to positioner assignment alogorithm
        """
        self.clear_target_assignments()

        # Until there are no unallocated positioners left...
        unalloc = self.unallocated()
        while unalloc > 0:
            for pos in self.positioners:
                for t in [*pos.targets]:
                    if not t.positioner:
                        if self._assign_target_to_positioner(pos, t):
                            break

            # If that pass didn't find any more, give up.
            if unalloc == self.unallocated():
                break
            unalloc = self.unallocated()

    def unallocated(self):
        """
        Returns
        -------
        int
            number of positioners without a target
        """
        n = 0
        for pos in self.positioners:
            if not pos.target:
                n += 1
        return n


    def _add_positioner(self, i, j):
        self.positioners.append(positioner(point(i * self.dx, j * self.dy)))


    def _assign_target_to_positioner(self, pos, t):
        """
        Try assigning a target to positioner. If we can't find a target that
        doesn't cause a collision with another positioner, put the positioner
        back to where it was.
        """
        theta_1 = pos.theta_1
        theta_2 = pos.theta_2
        pos.assign_target(t, False)
        if self._has_collision(pos):
            pos.assign_target(t, True)
            if self._has_collision(pos):
                pos.assign_target(None)
                pos.pose([theta_1, theta_2])
                return False
        return True


    def _has_collision(self, pos):
        for p in self.positioners:
            if p.collides_with(pos):
                return True
        return False
