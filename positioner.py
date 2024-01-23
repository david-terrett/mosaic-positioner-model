# -*- coding utf-8 -*-

from math import acos
from math import atan2
from math import cos
from math import fabs
from math import pi
from math import sin

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


    def __init__(self, position, id):
        """
        Create positioner

        Parameters
        ----------
        position : point
            position of positioner axis 1 in focal plane
        """
        self.position = position
        self.id = id
        self.targets = {}
        self.target = None

        # Define the rotation axis of arm 1 for a positioner at placed at 0,0
        axis_1 = point(0.0, 0.0)

        # Construct outline of lower positioner arm in the initial position
        # (angle 90, so parallel to the Y axis)
        arm_1 = polygon()
        w = 24.7 / 2.0
        l1 = 39.8
        l2 = w/2.0
        arm_1.append(point(w, -l2))
        arm_1.append(point(w, l1))
        arm_1.append(point(-w, l1))
        arm_1.append(point(-w, -l2))
        arm_1.append(point(w, -l2))

        # Position of arm2 rotation axis when arm 1 is parked
        axis_2 = point(0.0, 27.7)

        # Outline of arm 2 with axis at 0.0 and angle -90 (so folded
        # back on top of the lower arm).
        arm_2 = polygon()
        l1 = 57.0
        l2 = 10.0
        arm_2.append(point(w, l2))
        arm_2.append(point(-w, l2))
        arm_2.append(point(-w, -l1 - l2 + w * 2.0))
        arm_2.append(point(-w * 3.0, -l1 - l2 + w * 2.0))
        arm_2.append(point(-w * 3.00, -l1 - l2))
        arm_2.append(point(w, -l1 - l2))
        arm_2.append(point(w, l2))

        # Fibre position
        fiber = point(0.0, -57.0)

        # Move arm 2 onto its axis postion
        arm_2 = move_polygon(arm_2, axis_2.x(), axis_2.y())
        fiber = move_point(fiber, axis_2.x(), axis_2.y())

        # Move everything to the postioner's position
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
        self._arm_1_angle_offset = atan2(axis_2.y(), axis_2.x()) - pi/2.0
        self._arm_2_angle_offset = atan2(fiber.y() - axis_2.y(),
                                        fiber.x() - axis_2.x()) + pi/2.0

        # Park the positioner
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
                self.pose(self.targets[t][0])
            else:
                self.pose(self.targets[t][1])
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
            True if the fiber can be postioned at the specified point
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
        Postioner collides with another

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
        self.pose([0.0, pi])


    def plot(self, plt):
        """
        Plot the positioner outline
        """
        plt.plot(self.arm_1.x(), self.arm_1.y(), color='gray')
        plt.plot(self.arm_2.x(), self.arm_2.y(), color='black')
        plt.plot(self._axis_1_base.x(), self._axis_1_base.y(), '+', color='black',
                 markersize=4.0)
        plt.plot(self.axis_2.x(), self.axis_2.y(), '+', color='black',
                 markersize=4.0)
        if self.target:
            c = 'blue'
        else:
            c = 'red'
        plt.plot(self.fiber.x(), self.fiber.y(), 'o', color=c,
                 markersize=4.0)


    def pose(self, theta):
        """
        Set the axis positions

        Parmeters
        ---------
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
