# -*- coding utf-8 -*-

from math import floor
from random import random

from geometry import point
from positioner import positioner
from target import target

class focal_plane(object):
    """
    Model of the MOSAIC focal plane

    Attributes
    ----------
        positioners : [positioner]
            List of the positioners in the focal plane

        targets : [targets]
            List of targets
    """

    def __init__(self):
        self._dx = 95.0
        self._dy = 82.3
        self._x_max = 13.0 * self._dx
        self._x_min = -self._x_max;
        self._y_max = 12.0 * self._dy
        self._y_min = -self._y_max;

    # Build the list of positioners
        self.positioners = []
        self._add_column(0, 21)
        self._add_column(1, 20)
        self._add_column(-1, 20)
        self._add_column(2, 21)
        self._add_column(-2, 21)
        self._add_column(3, 20)
        self._add_column(-3, 20)
        self._add_column(4, 19)
        self._add_column(-4, 19)
        self._add_column(5, 8)
        self._add_column(-5, 8)
        self._add_column(6, 7)
        self._add_column(-6, 7)
        self._add_column(7, 6)
        self._add_column(-7, 6)
        self._add_column(8, 7)
        self._add_column(-8, 7)
        self._add_column(9, 8)
        self._add_column(-9, 8)
        self._add_column(10, 9)
        self._add_column(-10, 9)
        self._add_column(11, 8)
        self._add_column(-11, 8)

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
            x = self._x_min + random() * (self._x_max - self._x_min)
            y = self._y_min + random() * (self._y_max - self._y_min)
            self.targets.append(target(x, y))
        self.add_targets_to_positioners(self.targets)


    def plot(self, plt):
        plt.gca().set_aspect('equal')
        for p in self.positioners:
            p.plot(plt)
        for t in self.targets:
            plt.plot(t.position.x(), t.position.y(), '.', color='black',
                     markersize=1.0)
        return plt


    def simple_allocator(self):
        """
        Simple target to positioner assignment alogorithm
        """
        self.clear_target_assignments()

        # sort the list of positioners so that we allocate the ones
        # with the fewest targets first
        positioners = sorted(self.positioners, key=self._num_targets)

        # Until there are no unallocated positioners left...
        unalloc = self.unallocated()
        while unalloc > 0:
            for pos in positioners:
                if not pos.target:
                    for t in [*pos.targets]:
                        if not t.positioner:
                            if self._assign_target_to_positioner(pos, t):
                                break

            # If that pass didn't find any more, give up.
            if unalloc == self.unallocated():
                break
            unalloc = self.unallocated()

    def swapper(self):
        """
        Look for targets for free positioners by looking for reachable targets
        that are allocated to another positioner and seeing if that positioner
        can be allocated a different target
        """
        for pos in self.positioners:
            if not pos.target:
                self._try_swap(pos)


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


    def _add_column(self, x, n):
        if n % 2:
            self._add_positioner(x, 0)
            for y in range(1, floor(n/2.0) + 1):
                self._add_positioner(x, y)
                self._add_positioner(x, -y)
        else:
            for y in range(0, floor(n/2.0)):
                self._add_positioner(x, y + 0.5)
                self._add_positioner(x, -y - 0.5)


    def _add_positioner(self, i, j):
        self.positioners.append(positioner(point(i * self._dx, j * self._dy)))


    def _assign_target_to_positioner(self, pos, t):
        """
        Try assigning a target to positioner. If we can't find a target that
        doesn't cause a collision with another positioner, put the positioner
        back to where it was.
        """
        theta_1 = pos.theta_1
        theta_2 = pos.theta_2
        target = pos.target
        pos.assign_target(t, False)
        if self._has_collision(pos):
            pos.assign_target(t, True)
            if self._has_collision(pos):
                pos.assign_target(target)
                pos.pose([theta_1, theta_2])
                return False
        return True


    def _has_collision(self, pos):
        for p in self.positioners:
            if p.collides_with(pos):
                return True
        return False


    def _num_targets(self, p):
        return len(p.targets)


    def _try_swap(self, this_pos):
        print("trying", this_pos)

        # Save the current pos of the positioner so that we can put it back
        # to where it was
        theta1 = this_pos.theta_1
        theta2 = this_pos.theta_2

        # Go through all the targets this positioner can reach
        for t in [*this_pos.targets]:
            other_pos = t.positioner
            if other_pos:

                # This target is reachable but assigned to another positioner
                # so put this positioner on the targets and check for
                # collisions - but skipping the other positioner.
                for p in self.positioners:
                    if p is not other_pos:
                        if p.collides_with(this_pos):
                            break

                # We have found a target for this positioner so now look
                # for one for the positioner we are swapping with
                for t in other_pos.targets:
                    other_pos.assign_target(t)
                    if self._assign_target_to_positioner(other_pos, t):
                        return

        # Failure - put this positioner back to where it was
        this_pos.pose([theta1, theta2])
