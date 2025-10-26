# -*- coding utf-8 -*-

from ast import literal_eval
import csv
from math import cos
from math import floor
from math import inf
from math import radians
from math import sin
from math import sqrt
import matplotlib.pyplot as plt
import numpy as np

from .beam_steering_mirror import beam_steering_mirror
from .geometry import point
from .positioner import positioner
from .target import target

class focal_plane(object):
    """
    Model of the MOSAIC focal plane. It contains positioners and IFU
    beam steering mirrors.

    Attributes
    ----------
        beam_steering_mirrors : [beam_steering_mirror]
            List of beam steering mirrors.
        positioners : [positioner]
            List of the positioners in the focal plane
        targets : [targets]
            List of targets in the field
        x_max : double
            Maximum extend of field in x
        x_min : double
            Minimum extend of field in x
        y_max : double
            Maximum extend of field in y
        y_min : double
            Minimum extend of field in y
    """

    def __init__(self):
        self._dx = 95.0 * 3.0 / 4.0
        self._dy = 95.0 * sqrt(3.0) / 2.0
        self._max_sep2 = 30000.
        self.x_max = 13.0 * self._dx
        self.x_min = -self.x_max
        self.y_max = 12.0 * self._dy
        self.y_min = -self.y_max

    # Build the list of positioners
        self.positioners = []
        self._add_column(0, 13)
        self._add_column(1, 12)
        self._add_column(-1, 12)
        self._add_column(2, 13)
        self._add_column(-2, 13)
        self._add_column(3, 16)
        self._add_column(-3, 16)
        self._add_column(4, 19)
        self._add_column(-4, 19)
        self._add_column(5, 18)
        self._add_column(-5, 18)
        self._add_column(6, 17)
        self._add_column(-6, 17)
        self._add_column(7, 16)
        self._add_column(-7, 16)
        self._add_column(8, 13)
        self._add_column(-8, 13)
        self._add_column(9, 12)
        self._add_column(-9, 12)
        self._add_column(10, 9)
        self._add_column(-10, 9)

        # Set the positioner types
        types = [3,1,6,3,1,6,3,6,1,3,6,1,3,              # 13
                 6,3,1,6,3,1,1,3,6,1,3,6,                # 12
                 6,3,1,6,3,1,1,3,6,1,3,6,                # 12
                 3,1,6,3,1,6,1,6,1,3,6,1,3,              # 13
                 3,1,6,3,1,6,1,6,1,3,6,1,3,              # 13
                 6,3,1,6,3,1,6,1,1,3,6,1,3,6,1,1,        # 16
                 6,3,1,6,3,1,6,1,1,3,6,1,3,6,1,1,        # 16
                 3,1,6,3,1,6,3,1,6,1,6,1,3,6,1,3,6,1,1,  # 19
                 3,1,6,3,1,6,3,1,6,1,6,1,3,6,1,3,6,1,1,  # 19
                 6,3,1,6,3,1,6,3,1,1,3,6,1,3,6,1,3,0,    # 18
                 6,3,1,6,3,1,6,3,1,1,3,6,1,3,6,1,3,0,    # 18
                 3,1,6,3,1,6,3,1,0,6,1,3,6,1,3,6,1,      # 17
                 3,1,6,3,1,6,3,1,0,6,1,3,6,1,3,6,1,      # 17
                 6,1,3,6,1,3,2,1,1,3,6,1,3,2,1,1,        # 16
                 6,1,3,6,1,3,2,1,1,3,6,1,3,2,1,1,        # 16
                 0,0,0,1,1,2,1,0,0,1,2,1,1,              # 13
                 0,0,0,1,1,2,1,0,0,1,2,1,1,              # 13
                 0,0,0,0,1,1,0,0,0,1,1,0,                # 12
                 0,0,0,0,1,1,0,0,0,1,1,0,                # 12
                 0,0,0,1,1,0,0,1,0,                      # 9
                 0,0,0,1,1,0,0,1,0 ]                     # 9
        for pos in self.positioners:
            pos.type = types[pos.id]

        # Build list of neighbours for each positioner ignoring ones that
        # are absent
        for p in self.positioners:
            xp = p.position.x()
            yp = p.position.y()
            p.neighbours = []
            if p.exists():
                for q in self.positioners:
                    if p is not q and q.exists():
                        dx = xp - q.position.x()
                        dy = yp - q.position.y()
                        sep2 = dx*dx+dy*dy
                        if sep2 <= self._max_sep2:
                            p.neighbours.append(q)

        # Build the list of beam steering mirrors
        self.beam_steering_mirrors = []
        r = 1300.0
        self._add_bsm(r, 45.0 - 8.5)
        self._add_bsm(r, 45.0 + 8.5)
        self._add_bsm(r, -45.0 - 8.5)
        self._add_bsm(r, -45.0 + 8.5)
        self._add_bsm(r, 135.0 - 8.5)
        self._add_bsm(r, 135.0 + 8.5)
        self._add_bsm(r, -135.0 - 8.5)
        self._add_bsm(r, -135.0 + 8.5)

        # Initially, no targets allocated
        self.ir_allocated = 0
        self.vis_allocated = 0
        self.ifu_allocated = 0
        self.positioned = 0

        # List of targets is empty
        self.targets = []

        # No plot yet
        self.figure = None
        self.axes = None
        self._target_markers = []
        self.live_view = False

        # No collision matrix either
        self.collision_matrix = None

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

        # The collision matris is now invalid
        self.collision_matrix = None


    def build_collision_matrix(self):
        """
        Build the collision matrix
        """

    # We need to know the maximum number of neighbours that a positioner
    # can have
        max_neighbours = 0
        for p in self.positioners:
            if len(p.neighbours) > max_neighbours:
                max_neighbours = len(p.neighbours)

    # And the maximum number of targets that any positioner can reach
        max_targets = 0
        for p in self.positioners:
            if len(p.targets) > max_targets:
                max_targets = len(p.targets)

        # Give every target an id
        id = 0
        for t in self.targets:
            t.id = id
            id += 1

        try:

            # Create the collision matrix. The collision matrix is indexed by:
            #
            #  positioner id
            #  target index in the positioner's reachable targets array
            #  pose of the positioner (0 or 1)
            #  index of the other positioner in the neighbours array of the positioner
            #  target index in the other positioner's reachable targets array
            #  pose of the other positioner (0 or 1)
            #
            # and contains whether or not the positioners will collide
            self.collision_matrix = np.full((len(self.positioners), max_targets,
                                             2, max_neighbours, max_targets, 2),
                                            False)

            # Create the array that lists a positioners neighbours. Indexed by positioner id
            # and containing positioner id's
            self.neighbours_array = np.full((len(self.positioners), max_neighbours),
                                             -1)

            # Create the targets array that stores which target is allocated to each
            # positioner and which pose
            self.pos_to_targ_array = np.full((len(self.positioners), 2), -1)

            # Create the array that stores which positioner each target is assigned to
            self.targ_to_pos_array = np.full((len(self.targets)), -1)

            # Create the reachable targets array. This is indexed by positioner id and target
            # order in the positioner's reachable targets array. It contains target id's
            self.reachable_targets = np.full((len(self.positioners), max_targets), -1)

            # Populate the collision matrix
            for p in self.positioners:
                p.build_collision_matrix(self.collision_matrix)

            # Fill out the neighbours array
            for p in self.positioners:
                ni = 0
                for n in p.neighbours:
                    self.neighbours_array[p.id, ni] = n.id
                    ni += 1

            # Set the reachable targets array
            for p in self.positioners:
                n = 0
                for t in p.targets:
                    self.reachable_targets[p.id, n] = t.id
                    n +=1

        except BaseException as e:
            self.collision_matrix = None
            self.neighbours_array = None
            self.targets_array = None
            self.reachable_targets = None
            raise e

    def check(self):
        """
        Check that the configuration is valid

        Returns
        -------
        : list of list of positioners
            A list of pairs of colliding positioners
            are no collisions
        """
        collisions = []
        for p1 in self.positioners:
                for p2 in p1.neighbours:
                    if p1.collides_with(p2):
                        collisions.append([p1, p2])
        return collisions


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
        self.collision_matrix = None


    def clear_all_target_assignments(self):
        """
        Clear the target assignments for all positioners
        """
        for pos in self.positioners:
            pos.assign_target(None)


    def load_targets(self, csvfile):
        """
        Load targets from a CVS file

        Parameters
        ----------
        csvfile : file
            File to load from
        """
        reader = csv.DictReader(csvfile)
        targets = []
        for row in reader:
            targets.append(target(float(row['X']), float(row['Y']),
                                  ir=literal_eval(row['IR']),
                                  vis_lr=literal_eval(row['VIS_LR']),
                                  vis_hr=literal_eval(row['VIS_HR'])))
        self.add_targets(targets)

        # The collision matris is now invalid
        self.collision_matrix = None


    def park_all(self):
        """
        Set all positioners to their "park" positions
        """
        for pos in self.positioners:
            pos.alpha_motor.set(0.0)
            pos.beta_motor.set(0.0)
            pos.set_pose_from_motors()
            pos.on_target = False
        return


    def plot(self):
        """
        Plot positioners and beam steering mirrors
        """
        if not self.figure:
            self.figure = plt.figure()
            self.axes = self.figure.subplots()
        self.figure.gca().set_aspect('equal')
        self.figure.gca().axis([-1300.0, 1300.0, -1300.0, 1300.0])
        for p in self.positioners:
            p.plot(self.axes)

        # Plot the beam steering mirrors
        for bsm in self.beam_steering_mirrors:
            bsm.plot(self.axes)


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
        print(f"{self.ir_allocated} IR targets assigned to a positioner")
        print(f"{self.vis_allocated} VIS targets assigned to a positioner")
        print(f"{self.ifu_allocated} IFU targets assigned to a positioner")
        unalloc = (len(self.positioners) - self.ir_allocated -
                   self.vis_allocated - self.ifu_allocated)
        print(f"{unalloc} positioners out of {len(self.positioners)} don't have a target allocated")
        unreachable = 0
        one_target = 0
        on_target = 0
        positioners = 0
        ir_allocated = 0
        vis_allocated = 0
        for pos in self.positioners:
            if pos.exists():
                positioners += 1
                if len(pos.targets) == 0:
                    unreachable += 1
                elif len(pos.targets) == 1:
                    one_target += 1
                if pos.target:
                    if pos.target.ir:
                        ir_allocated += 1
                    elif pos.target.vis_lr or pos.target.vis_hr:
                        vis_allocated += 1
                if pos.on_target:
                    on_target += 1
        print(f"{ir_allocated} IR targets assigned to a positioner")
        print(f"{vis_allocated} VIS targets assigned to a positioner")
        unalloc = positioners - ir_allocated - vis_allocated
        print(f"{unalloc} positioners out of {positioners} don't have a target allocated")
        if unreachable > 0:
            print(f"of these {unreachable} can't reach any targets")
        else:
            print("all positioners can reach at least one target")
        print(f"and {one_target} can reach only one target")
        print(f"{on_target} positioners have been moved to their target positions")


    def will_collide(self, p1, t1, alt1):
        """
        Use the collision matrix to determine if assigning the target
        to the positioner will cause a collision.

        Arguments
        ---------
            p1 : positioner
                Positioner
            t1 : target
                Target
            alt1 : bool
                Use alternate pose
        """
        for p2 in range(len(self.neighbours_array[p1])):
            t2 = self.pos_to_targ_array[self.neighbours_array[p1, p2]][0]
            alt2 = self.pos_to_targ_array[self.neighbours_array[p1, p2]][1]
            if t2 != -1:
                if self.collision_matrix[p1, t1, alt1, p2, t2, alt2]:
                    return True
        return False


    def save_targets(self, csvfile):
        """
        Save targets to a CSV file

        Parameters
        ----------
        csvfile : file
            File to save to
        """
        writer = csv.DictWriter(csvfile,
                                fieldnames=['X', 'Y', 'IR', 'VIS_LR', 'VIS_HR'],
                                quoting=csv.QUOTE_MINIMAL)
        writer.writeheader()
        for t in self.targets:
            writer.writerow({'X': t.position.x(), 'Y': t.position.y(),
                             'IR': t.ir, 'VIS_LR': t.vis_lr,
                             'VIS_HR': t.vis_hr})


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
            for y in range(1, floor(n/2.0) + 1):
                self._add_positioner(x, -y)
        else:
            for y in range(0, floor(n/2.0)):
                self._add_positioner(x, y + 0.5)
            for y in range(0, floor(n/2.0)):
                self._add_positioner(x, -y - 0.5)

    def _add_bsm(self, r, a):
        self.beam_steering_mirrors.append(beam_steering_mirror(
            point(r * cos(radians(a)), r * sin(radians(a)))))


    def _add_positioner(self, i, j):
        """
        Add a positioner to the focal plane
        """
        self.positioners.append(positioner(point(i * self._dx, j * self._dy),
                                           len(self.positioners)))


    def _plot_targets(self):
        """
        Plot all the targets
        """
        for m in self._target_markers:
            try:
                self._target_markers.remove(m)
            except ValueError:
                pass
        self._target_markers = []
        color = 'white'
        for t in self.targets:
            if t.vis_lr or t.vis_hr and not t.ir:
                color = 'green'
            elif t.ir and not t.vis_lr or t.vis_hr:
                color = 'red'
            elif t.ir and t.vis_lr or t.vis_hr:
                color = 'brown'
            elif t.ifu:
                color = 'black'
            self._target_markers.append(self.axes.plot(t.position.x(),
                                        t.position.y(), '.', color=color,
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
                    this_pos.set_pose(this_pos.targets[t1][1])
                else:
                    this_pos.set_pose(this_pos.targets[t1][0])
                p = None
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
                    if other_pos.assign_target(t2, False):
                        this_pos.assign_target(t1)
                        return True
                    if other_pos.assign_target(t2, True):
                        this_pos.assign_target(t1)
                        return True

        # Failure - put this positioner back to where it was
        this_pos.set_pose([theta1, theta2])
        return False

def test_setup():
    fp = focal_plane()
    add_random_targets(fp, density=6, ir=True)
    add_random_targets(fp, density=5, vis_lr=True)
    add_random_targets(fp, density=5, vis_hr=True)
    simple_allocator(fp)
    fp.park_all()
    return fp
