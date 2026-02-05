# -*- coding utf-8 -*-
from enum import Enum
from math import acos
from math import atan2
from math import cos
from math import degrees
from math import fabs
from math import radians
from math import pi
from math import sin
from typing import NamedTuple

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Ellipse

from .geometry import distance
from .geometry import intersects
from .geometry import point
from .geometry import polygon
from .geometry_utilities import move_point
from .geometry_utilities import move_polygon
from .geometry_utilities import rotate_point
from .geometry_utilities import rotate_polygon

from .motor import motor
from .motor import step_all


class motor_positions(NamedTuple):
    alpha : float
    alpha_alt : float
    beta : float
    beta_alt : float


class target_type(Enum):
    IR = 0
    VIS_LR = 1
    VIS_HR = 2
    IFU = 3


class positioner(object):
    """
    Model of a MOSAIC fiber positioner.

    The orientation of the two arms is called the "pose" and the pose can
    be controlled, either by changing the positions of the axes motors
    or by setting it directly. The user is responsible for keeping the motor
    positions and the pose syncronised except when moving the motors along
    a path with the move method.

    Attributes
    ----------
    alpha_motor : motor
        alpha axis motor
    arm_1 : polygon
        outline of the lower arm
    arm_2_top : polygon
        outline of the upper arm top surface
    arm_2_bot : polygon
        outline of the upper arm bottom surface
    axis_2 : point
        position of second axis in focal plane
    beta_motor : motor
        beta axis motor
    blocker : positioner
        positioner the blocked the last attempted move (or None)
    id : any
        an identifier for the positioner
    ifu : point
        location of IFU pickoff mirror in focal plane
    ifu_max_r : float
        The maximum reach of the positioner with the IFU
    ifu_min_r : float
        The minimum reach of the positioner with the IFU
    ir_fiber : point
        location of IR fiber in focal plane
    ir_max_r : float
        The maximum reach of the positioner with the IR fibre
    ir_min_r : float
        The minimum reach of the positioner with the IR fibre
    neighbours : list
        A list of neighbouring positioners
    on_target : bool
        True if the positioner's fiber is positioned on the target
    position : point
        Position of the lower arm axis in the focal plane
    target : target
        Target assigned to the positioner
    target_pose : pose
        Pose that positions the fiber on the assigned target
    targets : dict of targets
        A dictionary of reachable targets indexed by target
    type : int
        type can be 0: Not present, 1: NIR-only, 2: VIS-ONLY, 3; NIR+VIS,
        6: VIS+VIS-HR, 8: Camera (i.e. a bit mask)
    theta_1 : float
        Angle of upper arm relative to x axis (radians)
    theta_2 : float
        Angle of lower arm relative to x axis (radians)
    vis_fiber : point
        location of VIS fiber in focal plane
    vis_max_r : float
        The maximum reach of the positioner with the VIS fibre
    vis_min_r : float
        The minimum reach of the positioner with the VIS fibre
    """


    def __init__(self, position, ident):
        """
        Create positioner

        Parameters
        ----------
        position : point
            Position of positioner axis 1 in focal plane
        ident : any
            Identifier for the positioner
        """
        self.position = position
        self.id = ident

        self.blocker = None
        self.alpha_motor = motor()
        self.beta_motor = motor()
        self.gamma_motor = motor()
        self.neighbours = []
        self.on_target = False
        self.target = None
        self.target_pose = None
        self.targets = {}
        self.type = 0

        self._d = []
        self._patches = []
        self._colours={0:'w',1:'r',2:'b',3:'m',6:'g',8:'c'}

        # Define the rotation axis of arm 1 for a positioner placed at 0,0
        axis_1 = point(0.0, 0.0)

        # Construct outline of lower positioner arm in the initial position
        # (angle 90, so parallel to the Y axis)
        arm_1 = polygon()
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

        # Position of arm 2 rotation axis when arm 1 is parked
        axis_2 = point(0.0, 28.5)

        # Outline of arm 2 top surface with axis at 0.0 and angle -90 (so
        # folded back on top of the lower arm).
        arm_2_top = polygon()
        l1 = 57.0
        l2 = 12.5
        #for i in range(0,semicirc_points+1):
        #    t = pi*float(i)/semicirc_points
        #    xx = l2*cos(t)
        #    yy = l2*sin(t)
        #    arm_2_top.append(point(xx,yy))
        arm_2_top.append(point(-w, -l1/2.0 + 10.0))
        arm_2_top.append(point(-w, -l1))
        for i in range(1,semicirc_points+1):
            t = pi*float(i)/semicirc_points
            xx = -l2*cos(t)
            yy = -l1-l2*sin(t)
            arm_2_top.append(point(xx,yy))
        arm_2_top.append(point(w, -l1))
        arm_2_top.append(point(w, -l1/2.0 - 10.0))
        for i in range(0,semicirc_points+1):
            t = pi * float(i)/semicirc_points
            xx = w + 8.0 + 10.0 * sin(pi - t)
            yy = - l1/2.0 + 10.0 * cos(pi - t)
            arm_2_top.append(point(xx,yy))
        arm_2_top.append(point(-w, -l1/2.0 + 10.0))
        arm_2_top.append(point(w, -l1/2.0 + 10.0))
        #arm_2_top.append(point(w, 0))

        # Outline of arm 2 with axis at 0.0 and angle -90 (so folded
        # back on top of the lower arm).
        arm_2_bot = polygon()
        l1 = 57.0
        l2 = 12.5
        for i in range(0,semicirc_points+1):
            t = pi*float(i)/semicirc_points
            xx = l2*cos(t)
            yy = l2*sin(t)
            arm_2_bot.append(point(xx,yy))
        arm_2_bot.append(point(-w, -l1/2.0 -10.0))
        arm_2_bot.append(point(w, -l1/2.0 - 10.0))
        for i in range(0,semicirc_points+1):
            t = pi * float(i)/semicirc_points
            xx = w + 8.0 + 10.0 * sin(pi - t)
            yy = - l1/2.0 + 10.0 * cos(pi - t)
            arm_2_bot.append(point(xx,yy))
        arm_2_bot.append(point(w, -l1/2.0 + 10.0))
        arm_2_bot.append(point(w, 0))

        # Fibre positions
        ir_fiber = point(4.0, -57.0)
        vis_fiber = point(-4.0, -57.0)
        ifu = point(0.0, -l1/2.0)

        # Move arm 2 onto its axis position
        arm_2_top = move_polygon(arm_2_top, axis_2.x(), axis_2.y())
        arm_2_bot = move_polygon(arm_2_bot, axis_2.x(), axis_2.y())
        ir_fiber = move_point(ir_fiber, axis_2.x(), axis_2.y())
        vis_fiber = move_point(vis_fiber, axis_2.x(), axis_2.y())
        ifu = move_point(ifu, axis_2.x(), axis_2.y())

        # Move everything to the positioner's position
        self._axis_1_base = move_point(axis_1, self.position.x(),
                                       self.position.y())
        self._arm_1_base = move_polygon(arm_1, self.position.x(),
                                        self.position.y())
        self._axis_2_base = move_point(axis_2, self.position.x(),
                                       self.position.y())
        self._arm_2_top_base = move_polygon(arm_2_top, self.position.x(),
                                        self.position.y())
        self._arm_2_bot_base = move_polygon(arm_2_bot, self.position.x(),
                                        self.position.y())
        self._ir_fiber_base = move_point(ir_fiber, self.position.x(),
                                         self.position.y())
        self._vis_fiber_base = move_point(vis_fiber, self.position.x(),
                                          self.position.y())
        self._ifu_base = move_point(ifu, self.position.x(), self.position.y())

        # Set the axes to the angles we used when defining the geometry
        self._theta_1_base = pi/2.0
        self._theta_2_base = -pi/2.0

        # Calculate the distances from axis 1 to axis 2 and axis 2 to the
        # fiber (need for calculating the arm angles to reach a point).
        self._axis_1_to_axis_2 = distance(axis_1, axis_2)
        self._axis_2_to_ir_fiber = distance(axis_2, ir_fiber)
        self._axis_2_to_vis_fiber = distance(axis_2, vis_fiber)
        self._axis_2_to_ifu = distance(axis_2, ifu)

        # Define the maximum and minimum radius the fibre can reach from the
        # arm 1 axis
        self.ir_max_r = self._axis_1_to_axis_2 + self._axis_2_to_ir_fiber
        self.ir_min_r = fabs(self._axis_2_to_ir_fiber - self._axis_1_to_axis_2)
        self.vis_max_r = self._axis_1_to_axis_2 + self._axis_2_to_vis_fiber
        self.vis_min_r = fabs(self._axis_2_to_vis_fiber - self._axis_1_to_axis_2)
        self.ifu_max_r = self._axis_1_to_axis_2 + self._axis_2_to_ifu
        self.ifu_min_r = fabs(self._axis_2_to_ifu - self._axis_1_to_axis_2)

        # Define the angle offsets between the arm 1 and axis 2 and arm 2
        # and the fiber.
        self._arm_1_angle_offset = atan2(axis_2.y(), axis_2.x()) - pi / 2.0
        self._ir_arm_2_angle_offset = atan2(ir_fiber.y() - axis_2.y(),
                                            ir_fiber.x() - axis_2.x()) + pi / 2.0
        self._vis_arm_2_angle_offset = atan2(vis_fiber.y() - axis_2.y(),
                                             vis_fiber.x() - axis_2.x()) + pi / 2.0
        self._ifu_arm_2_angle_offset = atan2(ifu.y() - axis_2.y(),
                                             ifu.x() - axis_2.x()) + pi / 2.0

        # Set the initial position
        self.alpha_motor.set(0.0)
        self.beta_motor.set(0.0)
        self.gamma_motor.set(0.0)
        self.set_pose_from_motors()


    def add_target(self, t):
        """
        Add a target to the list of potential targets for this positioner
        if it can reach and the target and positioner types are compatible

        Parameters
        ----------
        t : target
            Target to add

        Returns
        -------
        : bool
            True if the target is reachable
        """
        reachable = False
        if t.ir and self.is_ir():
            if self.can_reach(t.position, target_type.IR):
                self.targets[t] = self._arm_angles(t.position, target_type.IR)
                reachable = True
        if t.vis_lr and self.is_vis_lr():
            if self.can_reach(t.position, target_type.VIS_LR):
                self.targets[t] = self._arm_angles(t.position,
                                                   target_type.VIS_LR)
                reachable = True
        if t.vis_hr and self.is_vis_hr():
            if self.can_reach(t.position, target_type.VIS_HR):
                self.targets[t] = self._arm_angles(t.position,
                                                   target_type.VIS_HR)
                reachable = True
        if t.ifu and self.is_ifu():
            if self.can_reach(t.position, target_type.IFU):
                self.targets[t] = self._arm_angles(t.position, target_type.IFU)
                reachable = True
        if reachable:
            t.reachable.append(self)
        return reachable


    def try_assigning_target(self, targ, alt, collision_check=True,
                             ignore_no_target=True):
        """
        Try assigning a target to positioner. If we can't find a target that
        doesn't cause a collision with another positioner that already has a
        target assigned, put the positioner back to where it was.

        Arguments
        ---------
        targ : target
            The target to assign
        alt : bool
            Use the alternative pose
        collision_check : bool
            Check for collisions with neighbouring positioners
        ignore_no_target : bool
            Ignore neighbouring positioners that don't have a target

        Returns
        -------
        : bool
            True if the target was successfully assigned
        """
        current_theta_1 = self.theta_1
        current_theta_2 = self.theta_2
        current_target = self.target
        current_on_target = self.on_target
        self.assign_target(targ, alt)
        self.set_pose_to_target()
        if collision_check and self.has_collision(ignore_no_target):
            self.assign_target(current_target, False)
            self.set_pose(pose(current_theta_1, current_theta_2))
            self.on_target = current_on_target
            return False
        self.on_target = False
        return True


    def assign_target(self, t, alt=False):
        """
        Assign a target to the positioner

        Parameters
        ----------
        t : target
            Target to assign
        alt : bool
            Use alternate arm position
        """

        # Deallocate existing target
        if self.target:
            self.target.positioner = None
        if t:
            if not alt:
                self.target_pose = self.targets[t][0]
            else:
                self.target_pose = self.targets[t][1]

            # Assign the positioner to the target
            t.positioner = self

        # Assign the target to the positioner
        self.target = t
        self.alt = alt
        self.on_target = False


    def can_reach(self, p, targ_type):
        """
        Fiber can reach point

        Parameters
        ----------
        p : point
            position to test
        targ_type : target_type
            type of target

        Returns
        -------
        : bool
            True if the fiber can be positioned at the specified point
        """
        r2 = ((p.x() - self._axis_1_base.x()) *
              (p.x() - self._axis_1_base.x()) +
              (p.y() - self._axis_1_base.y()) *
              (p.y() - self._axis_1_base.y()))
        if targ_type == target_type.IR:
            return (r2 < self.ir_max_r * self.ir_max_r and
                    r2 > self.ir_min_r * self.ir_min_r)
        elif (targ_type == target_type.VIS_LR or
              targ_type == target_type.VIS_HR):
            return (r2 < self.vis_max_r * self.vis_max_r and
                    r2 > self.vis_min_r * self.vis_min_r)
        elif targ_type == target_type.IFU:
            return (r2 < self.ifu_max_r * self.ifu_max_r and
                    r2 > self.ifu_min_r * self.ifu_min_r)


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
        : bool
            True if the positioners collide
        """
        if other is self:
            return False
        return (intersects(self.arm_1, other.arm_1) or
                intersects(self.arm_2_top, other.arm_2_top) or
                intersects(self.arm_2_bot, other.arm_2_bot))


    def build_collision_matrix(self, matrix):
        """
        Builds the collision matrix entry for this positioner
        """
        self._build_collision_array(matrix, 0, 0)
        self._build_collision_array(matrix, 1, 0)
        self._build_collision_array(matrix, 0, 1)
        self._build_collision_array(matrix, 1, 1)


    def exists(self):
        """
        Positioner exists

        Returns
        -------
        : bool
            False if this is one of the non-existent positioners
        """
        return self.type != 0


    def get_motors_from_pose(self, pose):
        """
        Get the motor positions that match the pose

        Parameters
        ----------
        pose : pose
            Axis angles (radians)

        Returns
        -------
        pos : motor_positions
            motor positions
        """
        alpha = degrees(atan2(sin(pose.a), cos(pose.a)))
        b = pose.b - pose.a - pi
        beta = degrees(atan2(sin(b), cos(b)))

        if alpha + 360.0 <= self.alpha_motor.high_limit:
            alpha_alt = alpha + 360.0
        elif alpha - 360.0 >= self.alpha_motor.low_limit:
            alpha_alt = alpha - 360.0
        else:
            alpha_alt = None
        if beta + 360.0 <= self.beta_motor.high_limit:
            beta_alt = beta + 360.0
        elif beta - 360.0 >= self.beta_motor.low_limit:
            beta_alt = beta - 360.0
        else:
            beta_alt = None
        return motor_positions(alpha, alpha_alt, beta, beta_alt)


    def has_collision(self, ignore_no_target=False, ignore_not_on_target=False):
        """
        Returns whether or not the positioner collides with one of its
        neighbours.

        Arguments
        ---------
        ignore_no_target : bool
            ignore neighbours with no target assigned
        ignore_not_on_target : bool
            ignore neighbours which are not on target

        Returns
        -------
        : positioner
            The positioner the collision is with or None if there is no collision
        """
        for pos in self.neighbours:
            if ignore_not_on_target and not pos.on_target:
                continue
            if ignore_no_target and not pos.target:
                continue
            if self.collides_with(pos):
                return pos
        return None


    def is_ifu(self):
        """
        Has an IFU pickoff mirror

            True if the positioner has an IFU pickoff mirror
        """
        return True


    def is_ir(self):
        """
        Has an IR fibre

        Returns
        -------
        : bool
            True if the positioner has an IR fibre
        """
        return (self.type & 1) == 1


    def is_vis_lr(self):
        """
        Has a low res visible fibre

        Returns
        -------
        : bool
            True if the positioner has a low res visible fibre
        """
        return (self.type & 2) == 2


    def is_vis_hr(self):
        """
        Has a high res visible fibre

        Returns
        -------
        : bool
            True if the positioner has a high res visible fibre
        """
        return (self.type & 4) == 4


    def move_to_target(self, axes=None):
        """
        Move to the assigned target and set the on_target attribute.

        Arguments
        ---------
        axes : matplotlib axes
            Axes to plot positioner on

        Returns
        -------
        : bool
            True if the move succeeded
        """

        # Already there so return true
        if self.on_target:
            return True

        # No target so return true
        if not self.target:
            return True

        # Move to the target pose
        self.set_path_to_target()
        self.on_target = self.move(axes)

        return self.on_target


    def move_to_pose(self, destination_pose, axes=None):
        """
        Move to the specified pose. If the pose is the target pose
        the caller is responsible for setting the on_target attribute.

        Arguments
        ---------
        destination_pose : pose
            Pose to move to
        axes : matplotlib axes
            Axes to plot positioner on

        Returns
        -------
        : bool
            True if the move succeeded
        """
        self.set_path_to_pose(destination_pose)
        return self.move(axes)


    def move(self, axes=None):
        """
        Move the position along the motor paths checking for
        a collision at each step. If the move puts the fiber on the
        target the caller is responsible for setting the on_target
        attribute.

        Arguments
        ---------
        axes : matplotlib axes
            Axes to plot positioner on

        Returns
        -------
        : bool
            True if the move succeeded
        """

        # Save the current motor positions
        start_alpha = self.alpha_motor.position
        start_beta = self.beta_motor.position

        blocker = None
        while not step_all([self.alpha_motor, self.beta_motor]):
            self.set_pose_from_motors()
            blocker = self.has_collision()
            if blocker is not None:
                self.alpha_motor.set(start_alpha)
                self.beta_motor.set(start_beta)
                self.set_pose_from_motors()
                break
            if axes:
                self.plot(axes)
                plt.draw()
                plt.pause(0.02)
        if axes:
            self.plot(axes)
            plt.draw()
            plt.pause(0.02)
        self.blocker = blocker
        self.on_target = blocker is None
        return self.on_target


    def plot(self, axes):
        """
        Plot the positioner outline

        Parameters
        ----------
            axes : matplotlib axes
                plot axes
        """
        # Delete existing drawing
        for d in self._d:
            d[0].remove()
        self._d = []
        for patch in self._patches:
            patch.remove()
        self._patches = []

        if self.exists():

            # Draw the arms
            self._d.append(axes.plot(self.arm_1.x(), self.arm_1.y(), color='gray'))
            self._d.append(axes.plot(self.arm_2_top.x(), self.arm_2_top.y(),
                                   color=self._colours[self.type]))
            self._d.append(axes.plot(self.arm_2_bot.x(), self.arm_2_bot.y(),
                                   color=self._colours[self.type],
                                   dashes=(5,5)))

            # Draw the axes
            self._d.append(axes.plot(self._axis_1_base.x(), self._axis_1_base.y(),
                                   '+', color='black', markersize=4.0))
            self._d.append(axes.plot(self.axis_2.x(), self.axis_2.y(), '+',
                                   color='black', markersize=4.0))
            # Draw the fibers
            self._patches.append(axes.add_patch(Ellipse(xy=(self.ir_fiber.x(),
                                                          self.ir_fiber.y()),
                                              width=5, height=5, angle=0,
                                              facecolor='none',
                                              edgecolor='red')))
            self._patches.append(axes.add_patch(Ellipse(xy=(self.vis_fiber.x(),
                                                          self.vis_fiber.y()),
                                              width=5, height=5, angle=0,
                                              facecolor='none',
                                              edgecolor='red')))

            # Draw the direction the IFU pickoff mirror is pointing
            a = self.theta_2 + radians(self.gamma_motor.position)
            x = self.ifu.x() + 10.0 * cos(a)
            y = self.ifu.y() + 10.0 * sin(a)
            self._d.append(axes.plot([self.ifu.x(), x], [self.ifu.y(), y],
                           color='black'))


    def point_ifu(self, obj):
        """
        Point the IFU pickoff mirror at an object (anything with a position
        method)

        Parameters
        ----------
        obj : any
            Object.
        """
        # Mirror angle relative to x axis
        a = atan2(obj.position.y() - self._ifu_base.y(),
                  obj.position.x() - self._ifu_base.x())

        # Angle relative to arm 2
        a = a - self.theta_2

        # Set motor
        self.gamma_motor.set(degrees(a))


    def reverse_last_move(self, axes=None):
        """
        Try to reverse the last move we made, assuming the motor paths are
        unchanged.

        Arguments
        ---------
        axes : matplotlib axes
            Axes to plot positioner on

        Returns
        -------
        : bool
            True if the move succeeded
        """
        # Save the current motor positions
        start_alpha = self.alpha_motor.position
        start_beta = self.beta_motor.position

        blocker = None
        while not step_all([self.alpha_motor, self.beta_motor]):
            self.set_pose_from_motors()
            blocker = self.has_collision() # did we hit anything?
            if blocker is not None:
                self.alpha_motor.set(start_alpha)
                self.beta_motor.set(start_beta)
                self.set_pose_from_motors()
                break
            if axes:
                self.plot(axes)
                plt.draw()
                plt.pause(0.02)
        if axes:
            self.plot(axes)
            plt.draw()
            plt.pause(0.001)
        self.blocker = blocker
        return blocker is None


    def set_pose(self, positioner_pose):
        """
        Set the pose

        Arguments
        ---------
        positioner_pose : pose
            The new pose
        """
        self.theta_1 = positioner_pose.a
        self.theta_2 = positioner_pose.b
        self._update_geometry()


    def set_pose_to_target(self):
        """
        Set the pose to the current target

        Does nothing if there is no target assigned
        """
        if self.target:
            self.set_pose(self.target_pose)


    def set_pose_from_motors(self):
        """
        Set the pose from the current motor positions
        """
        t0 = radians(self.alpha_motor.position)
        t1 = t0 + radians(180 + self.beta_motor.position)
        self.theta_1 = t0
        self.theta_2 = t1
        self._update_geometry()


    def set_path_to_pose(self, destination_pose):
        """
        Set the motor paths to move the positioner to the destination pose

        Always uses the principle motor positions and not the alternate.

        Arguments
        ---------
        destination_pose : pose
            Destination pose
        """
        m = self.get_motors_from_pose(destination_pose)
        self.alpha_motor.set_path(m.alpha)
        self.beta_motor.set_path(m.beta)


    def set_path_to_target(self):
        """
        Set the motor paths to move the positioner to the current
        target.

        Always uses the principle motor positions and not the alternate.

        Does nothing if there is no target assigned
        """
        if self.target:
            m = self.get_motors_from_pose(self.target_pose)
            self.alpha_motor.set_path(m.alpha)
            self.beta_motor.set_path(m.beta)


    def uncollide(self, step=11.0):
        """
        Moves the positioner pose to somewhere that doesn't collide with any of its
        neighbours.

        Parameters
        ----------
        step : double
            Step size (degrees)
        """
        a_start = self.theta_1
        a_end = a_start + 2.0 * pi
        b_start = self.theta_2
        b_end = b_start + 2.0 * pi
        a = a_start
        while a < a_end:
            b = b_start
            while b < b_end:
                self.set_pose(pose(a, b))
                if not self.has_collision():
                    return
                b += radians(step)
            a += radians(step)
        raise RuntimeError("no position found without a collision")


    def zoom_to(self, figure, winsize=360):
        """
        Zoom plot and centre on this positioner

        Arguments
        ---------
        figure : mathplotlib figure

        winsize : int
            Window size

        """
        xmin = self._axis_1_base.x() - winsize/2
        xmax = self._axis_1_base.x() + winsize/2
        ymin = self._axis_1_base.y() - winsize/2
        ymax = self._axis_1_base.y() + winsize/2
        if figure:
            figure.gca().axis([xmin,xmax,ymin,ymax])
            plt.draw()
            plt.pause(0.002)
        return


    def _arm_angles(self, p, targ_type):

        # Bearing of target
        t = atan2(p.y() - self._axis_1_base.y(), p.x() - self._axis_1_base.x())

        # Solve for the angles of a triangle formed by axis 1 (A), axis 2 (B)
        # and the fiber (C)
        if targ_type == target_type.IR:
            a = self._axis_2_to_ir_fiber
        elif (targ_type == target_type.VIS_LR or
              targ_type == target_type.VIS_HR):
            a = self._axis_2_to_vis_fiber
        elif targ_type == target_type.IFU:
            a = self._axis_2_to_ifu
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

        if targ_type == target_type.IR:
            arm_2_angle_offset = self._ir_arm_2_angle_offset
        elif targ_type == target_type.VIS_LR or targ_type == target_type.VIS_HR:
            arm_2_angle_offset = self._vis_arm_2_angle_offset
        elif targ_type == target_type.IFU:
            arm_2_angle_offset = self._ifu_arm_2_angle_offset
        return [pose(arm_1_1 - self._arm_1_angle_offset, arm_2_1 - arm_2_angle_offset),
                pose(arm_1_2 - self._arm_1_angle_offset, arm_2_2 - arm_2_angle_offset)]



    def _arm_angles_to_pose(self, theta):
        # need to wrap angles here
        pose = theta.copy()
        pose[1] = self._wrap_angle_pmpi(theta[1]+theta[0]+pi)
        return pose


    def _build_collision_array(self, matrix, alt1, alt2):
        """
        Build the collision matrix for this positioner
        """
        # Save our current pose
        my_pose = pose(self.theta_1, self.theta_2)

        # For each neighbour
        np = 0
        for p in self.neighbours:
            other_pose = pose(p.theta_1, p.theta_2)
            n1 = 0
            for t1 in self.targets:
                self.set_pose(self.targets[t1][alt1])
                n2 = 0
                for t2 in p.targets:
                    p.set_pose(p.targets[t2][alt2])
                    matrix[self.id, n1, alt1, np, n2, alt2] = self.collides_with(p)
                    n2 += 1
                n1 += 1
            p.set_pose(other_pose)
            np += 1
        self.set_pose(my_pose)


    def _pose_to_arm_angles(self, theta):
        # need to wrap angles here
        t = _wrap_angle_pmpi(theta)
        arm_angles=t.copy()
        arm_angles[1] = _wrap_angle_pmpi(_wrap_angle_pmpi(t[1] - t[0]) - pi)
        return arm_angles

    def _update_geometry(self):

        # Get pose


        # Rotate arm 1
        c = cos(self.theta_1 - self._theta_1_base)
        s = sin(self.theta_1 - self._theta_1_base)
        self.arm_1 = rotate_polygon(self._arm_1_base, self._axis_1_base, c, s)
        self.axis_2 = rotate_point(self._axis_2_base, self._axis_1_base, c, s)

        # Move arm 2 to the new axis 2 position.
        arm_2_top = move_polygon(self._arm_2_top_base, self.axis_2.x() - self._axis_2_base.x(),
                             self.axis_2.y() - self._axis_2_base.y())
        arm_2_bot = move_polygon(self._arm_2_bot_base, self.axis_2.x() - self._axis_2_base.x(),
                             self.axis_2.y() - self._axis_2_base.y())
        ir_fiber = move_point(self._ir_fiber_base, self.axis_2.x() - self._axis_2_base.x(),
                             self.axis_2.y() - self._axis_2_base.y())
        vis_fiber = move_point(self._vis_fiber_base, self.axis_2.x() - self._axis_2_base.x(),
                             self.axis_2.y() - self._axis_2_base.y())
        ifu = move_point(self._ifu_base, self.axis_2.x() - self._axis_2_base.x(),
                             self.axis_2.y() - self._axis_2_base.y())

        # Rotate arm 2
        c = cos(self.theta_2 - self._theta_2_base)
        s = sin(self.theta_2 - self._theta_2_base)
        self.arm_2_top = rotate_polygon(arm_2_top, self.axis_2, c, s)
        self.arm_2_bot = rotate_polygon(arm_2_bot, self.axis_2, c, s)
        self.ir_fiber = rotate_point(ir_fiber, self.axis_2, c, s)
        self.vis_fiber = rotate_point(vis_fiber, self.axis_2, c, s)
        self.vis_fiber = rotate_point(vis_fiber, self.axis_2, c, s)
        self.ifu = rotate_point(ifu, self.axis_2, c, s)



def _wrap_angle_pmpi(theta):
    # wrap an angle into -pi < theta < pi
    return np.arctan2(np.sin(theta), np.cos(theta))


def _wrap_angle_ztpi(theta):
    # wrap an angle into 0 < theta < 2*pi
    angle = _wrap_angle_pmpi(theta)
    if angle < 0:
        angle = abs(angle) + 2 * (np.pi - abs(angle))
    return angle


class pose(NamedTuple):
    a: float
    b: float
