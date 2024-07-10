# -*- coding utf-8 -*-
from math import acos
from math import atan2
from math import cos
from math import fabs
from math import pi
from math import sin
import numpy as np
from matplotlib import pyplot as plt

from geometry import distance
from geometry import intersects
from geometry import move_point
from geometry import move_polygon
from geometry import point
from geometry import polygon
from geometry import rotate_point
from geometry import rotate_polygon

class positioner(object):
    """
    Model of a MOSAIC fiber positioner

    Attributes
    ----------
    arm_1 : polygon
        outline of the lower arm
    arm_2 : polygon
        outline of the upper arm
    axis_2 : point
        position of second axis in focal plane
    fiber : point
        location of fiber in focal plane
    max_r : float
        The maximum reach of the positioner
    min_r : float
        The minimum reach of the positioner
    targets : dict
        A dictionary of reachable targets indexed by target
    theta_1 : float
        Angle of lower arm relative to x axis (radians)
    theta_2 : float
        Angle of lower arm relative to x axis (radians)
    """


    def __init__(self, position, ident, type):
        """
        Create positioner

        Parameters
        ----------
        position : point
            position of positioner axis 1 in focal plane
        """
        self.position = position
        self.id = ident
        self.targets = {}
        self.target = None
        self.neighbours = []
        self.d = []

        # type can be 0: Not present, 1: NIR-only, 2: VIS-ONLY, 3; NIR+VIS, 5: NIR+VIS-HR, 8: Camera (i.e. a bit mask)
        self.colours={0:'w',1:'r',2:'b',3:'m',5:'g',8:'c'}
        self.type = type


        # Define the rotation axis of arm 1 for a positioner at placed at 0,0
        axis_1 = point(0.0, 0.0)

        # Construct outline of lower positioner arm in the initial position
        # (angle 90, so parallel to the Y axis)
        arm_1 = polygon()

        #        w = 24.7 / 2.0
        #        l1 = 39.8
        #        l2 = w/2.0
        #        arm_1.append(point(w, -l2))
        #        arm_1.append(point(w, l1))
        #        arm_1.append(point(-w, l1))
        #        arm_1.append(point(-w, -l2))
        #        arm_1.append(point(w, -l2))

        w = 25 / 2.0
        l1 = 28.5
        l2 = w
        semicirc_points = 6
        arm_1.append(point(w, 0))
        arm_1.append(point(w, l1))
        for i in range(0, semicirc_points+1):
            t=pi * float(i) / semicirc_points
            xx=l2 * cos(t)
            yy=l1 + l2 * sin(t)
            arm_1.append(point(xx, yy))
        arm_1.append(point(-w, l1))
        arm_1.append(point(-w, 0))
        for i in range(1, semicirc_points + 1):
            t = pi * float(i) / semicirc_points
            xx = -l2 * cos(t)
            yy = -l2 * sin(t)
            arm_1.append(point(xx, yy))
        arm_1.append(point(w, 0))

        # Position of arm2 rotation axis when arm 1 is parked
        axis_2 = point(0.0, 28.5)

        # Outline of arm 2 with axis at 0.0 and angle -90 (so folded
        # back on top of the lower arm).
        arm_2 = polygon()
        l1 = 57.0
        l2 = 12.5

        #        arm_2.append(point(w, l2))
        #        arm_2.append(point(-w, l2))
        #        arm_2.append(point(-w, -l1 - l2 + w * 2.0))
        #        arm_2.append(point(-w * 3.0, -l1 - l2 + w * 2.0))
        #        arm_2.append(point(-w * 3.00, -l1 - l2))
        #        arm_2.append(point(w, -l1 - l2))
        #        arm_2.append(point(w, l2))

        for i in range(0,semicirc_points+1):
            t = pi*float(i)/semicirc_points
            xx = l2*cos(t)
            yy = l2*sin(t)
            arm_2.append(point(xx,yy))
        arm_2.append(point(-w, -l1))
        for i in range(1,semicirc_points+1):
            t = pi*float(i)/semicirc_points
            xx = -l2*cos(t)
            yy = -l1-l2*sin(t)
            arm_2.append(point(xx,yy))
        arm_2.append(point(w, -l1))
        arm_2.append(point(w, 0))

        # Fibre position
        fiber = point(0.0, -57.0)

        # Move arm 2 onto its axis position
        arm_2 = move_polygon(arm_2, axis_2.x(), axis_2.y())
        fiber = move_point(fiber, axis_2.x(), axis_2.y())

        # Move everything to the positioner's position
        self._axis_1_base = move_point(axis_1, position.x(), position.y())
        self._arm_1_base = move_polygon(arm_1, position.x(), position.y())
        self._axis_2_base = move_point(axis_2, position.x(), position.y())
        self._arm_2_base = move_polygon(arm_2, position.x(), position.y())
        self._fiber_base = move_point(fiber, position.x(), position.y())

        # Set the axes to the angles we used when defining the geometry
        self._theta_1_base = pi/2.0
        self._theta_2_base = -pi/2.0

        # Calculate the distances from axis 1 to axis 2 and axis 2 to the
        # fiber (need for calculating the arm angles to reach a point).
        self._axis_1_to_axis_2 = distance(axis_1, axis_2)
        self._axis_2_to_fiber = distance(axis_2, fiber)

        # Define the maximum and minimum radius the fibre can reach from the
        # arm 1 axis
        self.max_r = self._axis_1_to_axis_2 + self._axis_2_to_fiber
        self.min_r = fabs(self._axis_2_to_fiber - self._axis_1_to_axis_2)

        # Define the angle offsets between the arm 1 and axis 2 and arm 2
        # and the fiber.
        self._arm_1_angle_offset = atan2(axis_2.y(), axis_2.x()) - pi / 2.0
        self._arm_2_angle_offset = atan2(fiber.y() - axis_2.y(),
                                        fiber.x() - axis_2.x()) + pi / 2.0

        # Park the positioner
        self.tpose = [0., pi]
        self.park()



    def add_target(self, t):
        """
        Add a target to the list of potential targets for this positioner
        if it can reach

        Parameters
        ----------
        t : target
            Target to add

        Returns
        -------
        : boolean
            True if the target is reachable
        """
        if self.can_reach(t.position):
            self.targets[t] = self._arm_angles(t.position)
            t.reachable.append(self)
        else:
            return False
        return True


    def assign_target(self, t, alt=False):
        """
        Assign a target to the positioner

        Parameters
        ----------
        t : target
            target to assign
        alt : boolean
            Use alternate arm position
        """
        if self.target:
            self.target.positioner = None
        if t:
            if alt:
                self.tpose = self.targets[t][0]
            else:
                self.tpose = self.targets[t][1]
            self.pose(self.tpose)
            t.positioner = self
        self.target = t


    def can_reach(self, p):
        """
        Fiber can reach point

        Parameters
        ----------
        p : point
            position to test

        Returns
        -------
        : boolean
            True if the fiber can be positioned at the specified point
        """
        r2 = ((p.x() - self._axis_1_base.x()) * (p.x() - self._axis_1_base.x()) +
              (p.y() - self._axis_1_base.y()) * (p.y() - self._axis_1_base.y()))
        return r2 < self.max_r * self.max_r and r2 > self.min_r * self.min_r


    def clear_targets(self):
        """
        Clear the list of reachable targets
        """
        self.assign_target(None)
        self.targets = {}


    def collides_with(self, other):
        """
        Positioner collides with another

        Parameters
        ----------
        other : positioner

        Returns
        -------
        : boolean
            True if the positioners collide
        """
        if other is self:
            return False
        return (intersects(self.arm_1, other.arm_1) or
                intersects(self.arm_2, other.arm_2))

    def park(self):
        """
        Park the positioner
        """
        self.pose([0, pi])
        self.poses = [[0, pi]]
        self.pose_index = 0
        self.in_position = True


    def plot(self, ax):
        """
        Plot the positioner outline

        Parameters
        ----------
            ax : matplotlib.Axes
                plot axes
        """
        # Delete existing drawing
        for d in self.d:
            d[0].remove()
        self.d = []

        # Draw the arms
        self.d.append(ax.plot(self.arm_1.x(), self.arm_1.y(), color='gray'))
        self.d.append(ax.plot(self.arm_2.x(), self.arm_2.y(), color=self.colours[self.type]))

        # Draw the axes
        self.d.append(ax.plot(self._axis_1_base.x(), self._axis_1_base.y(),
                              '+', color='black', markersize=4.0))
        self.d.append(ax.plot(self.axis_2.x(), self.axis_2.y(), '+',
                              color='black', markersize=4.0))
        if self.target:
            if (self.in_position):
                c = 'blue'
            else:
                c = 'green'
        else:
            c = 'red'

        # Draw the fiber
        self.d.append(ax.plot(self.fiber.x(), self.fiber.y(), 'o', color='red',
                      markersize=4.0))


    def directions(self, t_end):
        t1_0 = self.theta_1
        t1_1 = t_end[0]
        # Assume all angles in the range -pi < t < pi
        dt1 = t1_1 - t1_0
        t2_0 = self.theta_2+dt1
        t2_1 = t_end[1]
        dt2 = t2_1 - t2_0
        print(dt1, dt2)
        return dt1, dt2

    def pose_to_arm_angles(self,theta):
        # need to wrap angles here
        t = self.wrap_angle_pmpi(theta)
        arm_angles=t.copy()
        arm_angles[1] = self.wrap_angle_pmpi(self.wrap_angle_pmpi(t[1] - t[0]) - pi)
        return arm_angles

    def arm_angles_to_pose(self,theta):
        # need to wrap angles here
        pose = theta.copy()
        pose[1] = self.wrap_angle_pmpi(theta[1]+theta[0]+pi)
        return pose

    def wrap_angle_pmpi(self, theta):
        # wrap an angle into -pi < theta < pi
        return np.arctan2(np.sin(theta), np.cos(theta))

    def wrap_angle_ztpi(self, theta):
        # wrap an angle into 0 < theta < 2*pi
        angle = self.wrap_angle_pmpi(theta)
        if (angle < 0):
            angle = abs(angle) + 2 * (np.pi - abs(angle))
        return angle

    def zoom_to(self, figure, winsize=240):
        xmin = self._axis_1_base.x() - winsize/2
        xmax = self._axis_1_base.x() + winsize/2
        ymin = self._axis_1_base.y() - winsize/2
        ymax = self._axis_1_base.y() + winsize/2
        if (figure):
            figure.gca().axis([xmin,xmax,ymin,ymax])
            plt.draw()
            plt.pause(0.002)
        return

    def _has_collision(self):
        self.collision_list = []
        for p in self.neighbours:
            if self.collides_with(p):
                self.collision_list.append(p)
                return True
        return False

    def step_one_pose(self,figure,axes,forward=True):
        '''
        Probably need to return more than one flag here
        '''
        if not self.poses:
            return True  # no moves defined, so we are 'in position'
        if forward and self.in_position:
            return True # nowhere to go
        if (not forward and self.pos_index == 0):
            return True # nowhere to go
        _spi = self.pose_index
        if forward:
            self.pose_index = self.pose_index + 1
        else:
            self.pose_index = self.pose_index - 1
        _safe = True
        self.pose(self.poses[self.pose_index])
        _safe = not self._has_collision() # did we hit anything?
        if figure:
            self.zoom_to(figure)
            self.plot(axes)
            plt.draw()
            plt.pause(0.001)
        if not _safe:
            print(self.id, ': Blocked by ', self.collision_list[0].id)
            self.pose_index = _spi


    def move_to_position(self, figure, axes):
        """
        move through the calculated poses to get to destination, check for
        collisions at each step and report collisions.
        """
        if self.in_position:
            return True
        if not self.target:
            return True
        start_pose = [self.theta_1,self.theta_2]
        if figure:
            self.zoom_to(figure)
            _safe = True
            for next_pose in self.poses:
                if _safe:
                    self.pose(next_pose)
                    _safe = not self._has_collision() # did we hit anything?
                    if (not _safe):
                        self.pose(start_pose)
                    self.plot(axes)
                    plt.draw()
                    plt.pause(0.02)
            if _safe:
                print(self.id, ': Moved to final position')
                self.in_position = True
            else:
                print(self.id, ': Blocked by ', self.collision_list[0].id)
        else:
            self.plot(axes)
            plt.draw()
            plt.pause(0.001)
        return self.in_position

    def move_to_pose(self, figure, axes, pose=None):
        """
        move the positioner to a newly specified pose, plotting and checking for collisions along the way
        """
        if pose is None:
            pose = [0.0, pi]
        start_pose = [self.theta_1,self.theta_2]
        current_target_path = self.poses.copy()
        _old_in_position = self.in_position
        _safe = True

        # calculate a new set of poses to get to the new destination
        self.trajectory_from_here_simultaneous(pose)
        if figure:
            self.zoom_to(figure)
            for next_pose in self.poses:
                if _safe:
                    self.pose(next_pose)
                    if self._has_collision():  # did we hit anything?
                        _safe = False
                        self.pose(start_pose)
                    self.plot(axes)
                    plt.draw()
                    plt.pause(0.02)
            if _safe:
                print (self.id,': Moved to requested position')

                # calculate new path from here to target pose.
                self.trajectory_from_here_simultaneous(self.tpose)
                if (self.in_position):
                    self.in_position = False
                if ((next_pose[0] == self.tpose[0]) and (next_pose[1] == self.tpose[1])):
                    self.in_position = True
            else:
                self.in_position = _old_in_position
                block = self.collision_list[0]
                print(self.id, ': Collided with ', block.id)
            plt.pause(0.5)
        else:
            self.plot(axes)
            plt.draw()
            plt.pause(0.001)
        return _safe

    def reverse_last_move(self, figure, axes):
        """
        Try to reverse the last move we made, assuming the poses are
        unchanged. Check along the way.
        """
        start_pose = [self.theta_1,self.theta_2]
        current_target_path = self.poses.copy()
        _safe = True
        if figure:
            for i in range(0,len(self.poses)):
                if _safe:
                    self.pose(self.poses[-(i+1)])
                    if self._has_collision(): # did we hit anything
                        _safe = False
                        self.pose(start_pose)
                    self.plot(axes)
                    plt.draw()
                    plt.pause(0.02)
            if _safe:
                if (self.in_position):
                    self.in_position = False
                if ((self.poses[-(i + 1)][0] == self.tpose[0]) and (
                     self.poses[-(i + 1)][1] == self.tpose[1])):
                    self.in_position = True
            else:
                print(self.id,
                     ': Collided with something trying to return to last start position',
                      self.collision_list[0].id)
            plt.pause(0.5)
        else:
            self.plot(axes)
            plt.draw()
            plt.pause(0.001)
        return _safe



    def trajectory_from_here_simultaneous(self, theta):
        """
        Move both arms in 50 steps together
        """
        abend = self.pose_to_arm_angles(theta) # final alpha beta in -pi < angle < pi
        pstart = abend.copy()
        pend = theta
        pstart[0] = self.theta_1
        pstart[1] = self.theta_2
        abstart = self.pose_to_arm_angles(pstart) # initial alpha beta in -pi < angle < pi
        self.poses = []
        self.pos_index = 0
#        print ('Pose Start: ',np.asarray(pstart)*180/np.pi)
#        print ('Pose End: ',np.asarray(pend)*180/np.pi)
#        print ('AB Start: ',np.asarray(abstart)*180/np.pi)
#        print ('AB End: ',np.asarray(abend)*180/np.pi)
        d0 = abend[0] - abstart[0]
        d1 = abend[1] - abstart[1]
        abnew = abstart.copy()
        for i in range(0,51):
            abnew[0] = abstart[0] + d0 * i / 50
            abnew[1] = abstart[1] + d1 * i / 50
            pnew = self.arm_angles_to_pose(abnew)
            self.poses.append(pnew)
        return


    def set_next_pose(self, i):
        self.pose(self.poses[i])
        return


    def pose(self, theta):
        """
        Set the axis positions

        Parameters
        ----------
        theta : list[float]
            Axis angles (radians)
        """
        self.theta_1 = theta[0]
        self.theta_2 = theta[1]

        # Rotate arm 1
        c = cos(theta[0] - self._theta_1_base)
        s = sin(theta[0] - self._theta_1_base)
        self.arm_1 = rotate_polygon(self._arm_1_base, self._axis_1_base, c, s)
        self.axis_2 = rotate_point(self._axis_2_base, self._axis_1_base, c, s)

        # Move arm 2 to the new axis 2 position.
        arm_2 = move_polygon(self._arm_2_base, self.axis_2.x() - self._axis_2_base.x(),
                             self.axis_2.y() - self._axis_2_base.y())
        fiber = move_point(self._fiber_base, self.axis_2.x() - self._axis_2_base.x(),
                             self.axis_2.y() - self._axis_2_base.y())

        # Rotate arm 2
        c = cos(theta[1] - self._theta_2_base)
        s = sin(theta[1] - self._theta_2_base)
        self.arm_2 = rotate_polygon(arm_2, self.axis_2, c, s)
        self.fiber = rotate_point(fiber, self.axis_2, c, s)


    def _arm_angles(self, p):

        # Bearing of target
        t = atan2(p.y() - self._axis_1_base.y(), p.x() - self._axis_1_base.x())

        # Solve for the angles of a triangle formed by axis 1 (A), axis 2 (B)
        # and the fiber (C)
        a = self._axis_2_to_fiber
        b = distance(self._axis_1_base, p)
        c = self._axis_1_to_axis_2
        A = acos((b * b + c * c - a * a)/(2.0 * b * c))
        B = acos((a * a + c * c - b * b)/(2.0 * a * c))

        # Arm angles
        arm_1_1 = t - A
        arm_2_1 = arm_1_1 + pi - B

        # Alternate arm angles
        arm_1_2 = t + A
        arm_2_2 = arm_1_2 - pi + B

        return ([arm_1_1 - self._arm_1_angle_offset, arm_2_1 - self._arm_2_angle_offset],
                [arm_1_2 - self._arm_1_angle_offset, arm_2_2 - self._arm_2_angle_offset])
