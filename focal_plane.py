# -*- coding utf-8 -*-

import csv
from math import floor
from math import inf
from math import pi
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
        self._max_sep2 = self._dx*self._dx*4
        self._x_max = 13.0 * self._dx
        self._x_min = -self._x_max
        self._y_max = 12.0 * self._dy
        self._y_min = -self._y_max
        self._types = [3,5,2,3,5,2,3,1,0,1,2,5,3,2,5,3,2,1,1,   
                       2,3,5,2,3,5,2,3,1,5,3,2,5,3,2,5,3,0,
                       2,3,5,2,3,5,2,3,1,5,3,2,5,3,2,5,3,0,
                       3,5,2,3,5,2,3,1,2,1,2,5,3,2,5,3,2,1,1,
                       3,5,2,3,5,2,3,1,2,1,2,5,3,2,5,3,2,1,1,
                       2,3,5,2,3,5,2,3,1,5,3,2,5,3,2,5,3,0,
                       2,3,5,2,3,5,2,3,1,5,3,2,5,3,2,5,3,0,
                       3,5,2,3,5,2,3,1,0,2,5,3,2,5,3,2,1,
                       3,5,2,3,5,2,3,1,0,2,5,3,2,5,3,2,1,
                       2,3,5,2,3,5,8,1,5,3,2,5,3,2,5,1,
                       2,3,5,2,3,5,0,1,5,3,2,5,3,0,5,1,
                       3,5,2,3,0,0,0,0,0,2,5,3,8,0,0,0,8,
                       3,5,2,3,1,0,0,0,8,2,5,3,0,0,0,0,8,
                       2,3,5,8,5,3,2,0,
                       2,3,5,8,5,3,2,8,
                       3,5,2,1,2,5,1,
                       3,5,2,1,2,5,1,
                       2,3,5,5,3,0,
                       2,3,1,1,3,2,
                       1,1,0,1,0,1,1,
                       1,1,0,1,0,1,1]
                       
                       
                       
    # Build the list of positioners
        self.positioners = []
        self._add_column(0, 19)
        self._add_column(1, 18)
        self._add_column(-1, 18)
        self._add_column(2, 19)
        self._add_column(-2, 19)
        self._add_column(3, 18)
        self._add_column(-3, 18)
        self._add_column(4, 17)
        self._add_column(-4, 17)
        self._add_column(5, 16)
        self._add_column(-5, 16)
        self._add_column(6, 17)
        self._add_column(-6, 17)
        self._add_column(7, 8)
        self._add_column(-7, 8)
        self._add_column(8, 7)
        self._add_column(-8, 7)
        self._add_column(9, 6)
        self._add_column(-9, 6)
        self._add_column(10, 7)
        self._add_column(-10, 7)

        # Build list of neighbours for each positioner
        for p in self.positioners:
            p.type = self._types[p.id]
            xp=p.position.x()
            yp=p.position.y()
            p.neighbours = []
            for q in self.positioners:
                if not p == q:
                    dx= xp-q.position.x()
                    dy= yp-q.position.y()
                    sep2 = dx*dx+dy*dy
                    if sep2 <= self._max_sep2:
                        p.neighbours.append(q)

        # No targets allocated
        self.allocated = 0
        self.positioned = 0

        # empty list of targets
        self.targets = []

        # No plot yet
        self.figure = None
        self.axes = None
        self._target_markers = []
        self.live_view = False


    def add_targets(self, targets):
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
        for t in targets:
            if len(t.reachable):
                self.targets.append(t)

        # Plot the new targets
        if self.figure:
            self._plot_targets()


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
        Remove all targets
        """
        for pos in self.positioners:
            pos.clear_targets()
        for p in self._target_markers:
            p[0].remove()
        self._target_markers = []
        self.targets = []
        self.allocated = 0
        self.positioned = 0


    def clear_target_assignments(self):
        """
        Clear the target assignments for all positioners
        """
        for pos in self.positioners:
            pos.assign_target(None)
        self.allocated = 0
        self.positioned = 0


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

        targets = []
        for i in range(0, n):
            x = self._x_min + random() * (self._x_max - self._x_min)
            y = self._y_min + random() * (self._y_max - self._y_min)
            targets.append(target(x, y))
        self.add_targets(targets)


    def load_targets(self, csvfile):
        """
        Load targets from a CVS file

        Parameters
        ----------
        csvfile : file
            File to load from
        """
        reader = csv.DictReader(csvfile, quoting=csv.QUOTE_NONNUMERIC)
        targets = []
        for row in reader:
            targets.append(target(row['X'], row['Y']))
        self.add_targets(targets)


    def plot(self):
        """
        Plot positioners
        """
        if not self.figure:
            self.figure = plt.figure()
            self.axes = self.figure.subplots()
        self.figure.gca().set_aspect('equal')
        self.figure.gca().axis([-1300.0, 1300.0, -1000.0, 1000.0])
        for p in self.positioners:
            p.plot(self.axes)

        # Plot the targets
        if self.figure:
            self._plot_targets()


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
        print(f"{self.positioned} positioners can be moved to their target positions without collisions")


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


    def _add_column(self, x, n):
        if n % 2:
            self._add_positioner(x, 0, 1)
            for y in range(1, floor(n/2.0) + 1):
                self._add_positioner(x, y, 1)
            for y in range(1, floor(n/2.0) + 1):
                self._add_positioner(x, -y, 2)
        else:
            for y in range(0, floor(n/2.0)):
                self._add_positioner(x, y + 0.5, 3)
            for y in range(0, floor(n/2.0)):
                self._add_positioner(x, -y - 0.5, 5)


    def _add_positioner(self, i, j, t):
        self.positioners.append(positioner(point(i * self._dx, j * self._dy),
                                           len(self.positioners), t))


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


    def simple_allocator(self, live_view=False):
        """
        Simple target to positioner assignment algorithm
        """
        self.live_view = live_view
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

        # Turn off live view at the end anyway, to keep everything tidy
        self.live_view = False


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
        pos.in_position = False # Assigned a target, but haven't got there yet
        if True: # (self.live_view):
            # set up the move sequence
            _tdest = [pos.theta_1,pos.theta_2]
            _tpos = pos.fiber
            pos.pose([current_theta_1,current_theta_2]) # set it back to park for a second
            pos.trajectory_from_here_simultaneous(_tdest) # calculate the movement
            pos.pose(_tdest) # set it in position so we can continue the allocations
        return True

    def _park_all(self):
        for p in self.positioners:
            p.park()
            p.in_position = False
        return

    def _move_to_position(self, pos):
        if pos.in_position:
            return True
        current_theta_1 = pos.theta_1
        current_theta_2 = pos.theta_2
        if self.figure and pos.target:
            self._zoom_to(pos)
            _safe = True
            once = True
            while once:
                once=False
                for next_pose in pos.poses:
                    if _safe:
                        pos.pose(next_pose)
                        if self._has_collision(pos):  # Did we hit anything along the way
                            _safe = False
                            pos.pose([current_theta_1,current_theta_2])
                        pos.plot(self.axes)
                        plt.draw()
                        plt.pause(0.02)
                print('End of first loop... Safe = ',_safe)
                if _safe:
                    print (pos.id,' Moved to final position ')
                    pos.in_position = True
                else:
                    block = pos.collision_list[0]
                    print(pos.id," Collided with something along the way... deferring until next pass", block.id)
                    if not block.target:
                        print(block.id,' Has no target... trying to move 90 degrees')
                        if self._move_to_pose(block,[pi/2,3*pi/2]):
                            print('moved ',block.id)
                    else:
                        if block.in_position:
                            print(block.id,' Has a target, but is already in position... trying to park')
                            if self._reverse_last_move(block):
                                print('Parked ',block.id, block.in_position)
                                block.in_position = False
                                print('Parked ',block.id, block.in_position)
                        else:
                            print(block.id, ' has not yet been moved... trying to deploy')
                            if self._move_to_position(block):
                                print ('Successfully deployed ',block.id,block.in_position)
                                block.in_position = True
                                print ('Successfully deployed ',block.id,block.in_position)


                    plt.pause(0.5)
                print(pos.in_position)
        else:
            pos.plot(self.axes)
            plt.draw()
            plt.pause(0.001)
        return pos.in_position


    def _move_to_pose(self, pos, pose=None):
        """
        Try to move this positioner to a specified new pose
        leave it back one step if we hit anything
        """
        if pose is None:
            pose = [0.0, pi]
        current_theta_1 = pos.theta_1
        current_theta_2 = pos.theta_2
        current_target_path = pos.poses.copy()
        _safe = True
        if self.figure:

            # calculate a new set of poses to get to the new destination
            pos.trajectory_from_here_simultaneous(pose)
            for next_pose in pos.poses:
                if _safe:
                    pos.pose(next_pose)
                    if self._has_collision(pos):  # Did we hit anything along the way
                        _safe = False
                        pos.pose([current_theta_1,current_theta_2])
                    pos.plot(self.axes)
                    plt.draw()
                    plt.pause(0.02)
            if _safe:

                # New path from here to final destination
                pos.trajectory_from_here_simultaneous(current_target_path[-1])
            else:
                pos.in_position = False
                block = pos.collision_list[0]
                print (pos.id,
                       " Collided with something along the way... deferring until next pass",
                       block.id)
            plt.pause(0.5)
        else:
            pos.plot(self.axes)
            plt.draw()
            plt.pause(0.001)
        return _safe


    def _reverse_last_move(self, pos):
        """
        Try to reverse the moves for this positioner (n.b. this may not be to park)
        """
        current_theta_1 = pos.theta_1
        current_theta_2 = pos.theta_2
        _safe = True
        if self.figure:
            for i in range (0,len(pos.poses)):
                if _safe:
                    pos.pose(pos.poses[-(i+1)])
                    if self._has_collision(pos):
                        _safe = False
                        pos.pose([current_theta_1,current_theta_2])
                    pos.plot(self.axes)
                    plt.draw()
                    plt.pause(0.02)
            if _safe:
                pos.in_position = False
            else:
                print (pos.id,"Collided with something try to return to start position",pos.collision_list[0].id)
            plt.pause(0.5)
        else:
            pos.plot(self.axes)
            plt.draw()
            plt.pause(0.001)
        return _safe


    def _zoom_to(self,pos):
        xmin = pos._axis_1_base.x()-120
        xmax = pos._axis_1_base.x()+120
        ymin = pos._axis_1_base.y()-120
        ymax = pos._axis_1_base.y()+120
        if self.figure:
            self.figure.gca().axis([xmin,xmax,ymin,ymax])
            plt.draw()
            plt.pause(0.002)
        return


    def _has_collision(self, pos):
        pos.collision_list = []
        for p in pos.neighbours:
            if p.collides_with(pos):
                pos.collision_list.append(p)
                return True
        return False


    def _plot_targets(self):
        for m in self._target_markers:
            try:
                self._target_markers.remove(m)
            except ValueError:
                pass
        self._target_markers = []
        for t in self.targets:
            self._target_markers.append(self.axes.plot(t.position.x(),
                                        t.position.y(), '.', color='black',
                                        markersize=1.0))


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
