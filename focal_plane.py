# -*- coding utf-8 -*-

import csv
from math import floor
from math import inf
from random import random
import matplotlib.pyplot as plt

from geometry import point
from positioner import positioner
from target import target

class focal_plane(object):
    """
    Model of the MOSAIC focal plane

    Attributes
    ----------
        allocated : int
            Number of positioners with targets allocated
        positioners : [positioner]
            List of the positioners in the focal plane
        targets : [targets]
            List of targets
    """

    def __init__(self):
        self._dx = 95.0
        self._dy = 82.3
        self._x_max = 13.0 * self._dx
        self._x_min = -self._x_max
        self._y_max = 12.0 * self._dy
        self._y_min = -self._y_max

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

    # No targets allocated
        self.allocated = 0

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
            for t in targets:
                p.add_target(t)

        # Remove any targets that can't be reached by any positioner
        targets = []
        for t in self.targets:
            if len(t.reachable):
                targets.append(t)
        self.targets = targets

    def check(self):
        """
        Check that the configuration is valid

        Returns
        -------
        : boolean
            True if there are no collisions
        """
        for p in self.positioners:
            if self._has_collision(p):
                return False
        return True


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
        self.allocated = 0


    def create_random_targets(self, density):
        """
        Create random targets and add them to the positioners

        Parameters
        ---------
        density : float
            Density of targets to create (number per arcmin^2)
        """
        self.targets = []

        # Plate scale (mm/arcsec)
        plate_scale = 3.316

        # Area to cover (arcsec^2)
        area = ((self._x_max - self._x_min) * (self._y_max - self._y_min) /
                (plate_scale * plate_scale))

        # Number of targets to create
        n = int(density * area / 3600.0)

        for i in range(0, n):
            x = self._x_min + random() * (self._x_max - self._x_min)
            y = self._y_min + random() * (self._y_max - self._y_min)
            self.targets.append(target(x, y))
        self.add_targets_to_positioners(self.targets)


    def load_targets(self, csvfile):
        """
        Load targets from a CVS file

        Parameters
        ----------
        csvfile : file
            File to load from
        """
        reader = csv.DictReader(csvfile, quoting=csv.QUOTE_NONNUMERIC)
        for row in reader:
            self.targets.append(target(row['X'], row['Y']))
        self.add_targets_to_positioners(self.targets)


    def plot(self):
        """
        Plot positioners
        """
        plt.gca().set_aspect('equal')
        for p in self.positioners:
            p.plot(plt)
        for t in self.targets:
            plt.plot(t.position.x(), t.position.y(), '.', color='black',
                     markersize=1.0)


    def positioner_at(self, x, y):
        """
        Find the positioner closest to a given point

        Parameters
        ----------
        x : float
            x position
        y : float
            y position

        Returns
        -------
        : positioner
            The positioner closest to x,y
        """
        min_d2 = inf
        pos = None
        for p in self.positioners:
            d2 = ((p.position.x() - x) * (p.position.x() - x) +
               (p.position.y() - y) * (p.position.y() - y))
            if d2 < min_d2:
                min_d2 = d2
                pos = p
        return pos


    def report(self):
        """
        Print stuff about the state of the positioner
        """
        print(f"there are {len(self.targets)} targets in the field")
        print("{0} positioners out of {1} don't have a target allocated".
              format(len(self.positioners) - self.allocated,
              len(self.positioners)))
        unreachable = 0
        one_target = 0
        for p in self.positioners:
            if len(p.targets) == 0:
                unreachable += 1
            elif len(p.targets) == 1:
                one_target += 1
        if unreachable > 0:
            print(f"of these {unreachable} can't reach any targets")
        else:
            print("all positioners can reach at least one target")
        print(f"and {one_target} can reach only one target")


    def save_targets(self, csvfile):
        """
        Save targets to a CSV file

        Parameters
        ----------
        csvfile : file
            File to save to
        """
        writer = csv.DictWriter(csvfile, fieldnames=['X', 'Y'],
                                quoting=csv.QUOTE_NONNUMERIC)
        writer.writeheader()
        for t in self.targets:
            writer.writerow({'X': t.position.x(), 'Y': t.position.y()})


    def simple_allocator(self):
        """
        Simple target to positioner assignment alogorithm

        Paramters
        ---------
            swapper : boolean
                Run the swapper after the initial allocation
        """
        self.clear_target_assignments()

        # sort the list of positioners so that we allocate the ones
        # with the fewest targets first
        positioners = sorted(self.positioners, key=lambda p: len(p.targets))

        # Until there are no unallocated positioners left...
        alloc = self.allocated
        while self.allocated < len(self.positioners):
            for pos in positioners:

                # If this positioner doesn't have a target
                if not pos.target:

                    # Sort the list of potential targets so that we try the
                    # ones with the fewest positioners that can reach it first
                    targets = sorted([*pos.targets],
                                     key=lambda t: len(t.reachable))
                    for t in targets:
                        if not t.positioner:
                            if self._assign_target_to_positioner(pos, t):
                                break

            # If that pass didn't find any more, give up.
            if alloc == self.allocated:
                break
            alloc = self.allocated


    def swapper(self):
        """
        Look for targets for free positioners by looking for reachable targets
        that are allocated to another positioner and seeing if that positioner
        can be allocated a different target
        """
        for pos in self.positioners:
            if not pos.target:
                if not self._try_swap(pos, False):
                    self._try_swap(pos, True)


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
        self.positioners.append(positioner(point(i * self._dx, j * self._dy),
                                           len(self.positioners)))


    def _assign_target_to_positioner(self, pos, t):
        """
        Try assigning a target to positioner. If we can't find a target that
        doesn't cause a collision with another positioner, put the positioner
        back to where it was.
        """
        current_theta_1 = pos.theta_1
        current_theta_2 = pos.theta_2
        current_target = pos.target
        pos.assign_target(t, False)
        if self._has_collision(pos):
            pos.assign_target(t, True)
            if self._has_collision(pos):
                pos.assign_target(current_target)
                pos.pose([current_theta_1, current_theta_2])
                return False
        self.allocated += 1
        return True


    def _has_collision(self, pos):
        for p in self.positioners:
            if p.collides_with(pos):
                return True
        return False


    def _try_swap(self, this_pos, alt):

        # Save the current pos of the positioner so that we can put it back
        # to where it was
        theta1 = this_pos.theta_1
        theta2 = this_pos.theta_2

        # Go through all the targets this positioner can reach
        for t1 in [*this_pos.targets]:
            other_pos = t1.positioner
            if other_pos:

                # This target is reachable but assigned to another positioner
                # so put this positioner on the target and check for
                # collisions - but skipping the other positioner as we are
                # going to try moving it somewhere else.
                if alt:
                    this_pos.pose(this_pos.targets[t1][1])
                else:
                    this_pos.pose(this_pos.targets[t1][0])
                for p in self.positioners:
                    if p is not other_pos:
                        if p.collides_with(this_pos):
                            break

                # If we found a collision try the next target
                if p:
                    continue

                # We have found a target for this positioner so now look
                # for one for the positioner we are trying swapping with
                for t2 in other_pos.targets:
                    if self._assign_target_to_positioner(other_pos, t2):
                        this_pos.assign_target(t1)
                        return True

        # Failure - put this positioner back to where it was
        this_pos.pose([theta1, theta2])
        return False
