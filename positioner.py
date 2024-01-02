# -*- coding utf-8 -*-

import matplotlib.pyplot as plt

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
    """


    def __init__(self, position):
        """
        Create positioner

        Parameters
        ----------
        position : point
            position of positioner axis 1 in focal plane
        """
        self.targets = {}

        # Define the rotation axis of arm 1 for a positioner at placed at 0,0
        axis_1 = point(0.0, 0.0)

        # Construct outline of lower positioner arm in the park position (angle 90)
        arm_1 = polygon()
        w = 24.7
        l1 = 39.8
        l2 = w/2.0
        arm_1.append(point(w/2.0, -l2))
        arm_1.append(point(w/2.0, l1))
        arm_1.append(point(-w/2.0, l1))
        arm_1.append(point(-w/2.0, -l2))
        arm_1.append(point(w/2.0, -l2))

        # Position of arm2 rotation axis when arm 1 is parked
        axis_2 = point(0.0, 27.7)

        # Outline of arm 2 with axis at 0.0 and angle -90.
        arm_2 = polygon()
        arm_2.append(point(w/2.0, 10.0))
        arm_2.append(point(-w/2.0, 10.0))
        arm_2.append(point(-w/2.0, -57.0 - 10.0))
        arm_2.append(point(w/2.0, -57.0 - 10.0))
        arm_2.append(point(w/2.0, 10.0))

        # Fibre position
        fiber = point(0.0, -57.0)

        # Move arm 2 onto its axis postion
        arm_2 = move_polygon(arm_2, axis_2.x(), axis_2.y())
        fiber = move_point(fiber, axis_2.x(), axis_2.y())

        # Move everything to the postioner's position
        self.axis_1_0 = move_point(axis_1, position.x(), position.y())
        self.arm_1_0 = move_polygon(arm_1, position.x(), position.y())
        self.axis_2_0 = move_point(axis_2, position.x(), position.y())
        self.arm_2_0 = move_polygon(arm_2, position.x(), position.y())
        self.fiber_0 = move_point(fiber, position.x(), position.y())

        # Set the axes to the park orientations
        self.theta_1_0 = pi/2.0
        self.theta_2_0 = -pi/2.0

        # Calculate the distances from axis 1 to axis 2 and axis 2 to the
        # fiber (need for calculating the arm angles to reach a point).
        self.axis_1_to_axis_2 = distance(axis_1, axis_2)
        self.axis_2_to_fiber = distance(axis_2, fiber)

        # Define the maximum and minimum radius the fibre can reach from the
        # arm 1 axis
        self.max_r = self.axis_1_to_axis_2 + self.axis_2_to_fiber
        self.min_r = fabs(self.axis_2_to_fiber - self.axis_1_to_axis_2)

        # Define the angle offsets between the arm 1 and axis 2 and arm 2
        # and the fiber.
        self.arm_1_angle_offset = atan2(axis_2.y(), axis_2.x()) - pi/2.0
        self.arm_2_angle_offset = atan2(fiber.y() - axis_2.y(),
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


    def assign(self, t, alt):
        """
        Assign a target to the positioner

        Parameters
        ----------
        t : target
            target to assign
        a : boolean
            Use alternate arm position
        """
        if alt:
            self.pose(self.targets[t][0])
        else:
            self.pose(self.targets[t][1])
        self.target = t
        t.positioner = self


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
        r2 = ((p.x() - self.axis_1_0.x()) * (p.x() - self.axis_1_0.x()) +
              (p.y() - self.axis_1_0.y()) * (p.y() - self.axis_1_0.y()))
        return r2 < self.max_r * self.max_r and r2 > self.min_r * self.min_r


    def collides_with(self, other):
        """
        Postioner collides with another

        Parameters
        ----------
        other : positioner

        Returns
        -------
        Boolean
            True if the positioners collide
        """
        return (intersects(self.arm_1, other.arm_1) or
                intersects(self.arm_2, other.arm_1))

    def park(self):
        """
        Park the positioner
        """
        self.pose([0.0, pi])


    def plot(self):
        """
        Plot the positioner outline
        """
        plt.plot(self.arm_1.x(), self.arm_1.y(), color='black')
        plt.plot(self.arm_2.x(), self.arm_2.y(), color='black')
        plt.plot(self.axis_1_0.x(), self.axis_1_0.y(), '+')
        plt.plot(self.axis_2.x(), self.axis_2.y(), '+')
        plt.plot(self.fiber.x(), self.fiber.y(), 'o')


    def pose(self, theta):
        """
        Set the axis positions

        Parmeters
        ---------
        theta : list[float]
            Axis angles (radians)
        """

        # Rotate arm 1
        c = cos(theta[0] - self.theta_1_0)
        s = sin(theta[0] - self.theta_1_0)
        self.arm_1 = rotate_polygon(self.arm_1_0, self.axis_1_0, c, s)
        self.axis_2 = rotate_point(self.axis_2_0, self.axis_1_0, c, s)

        # Move arm 2 to the new axis 2 position.
        arm_2 = move_polygon(self.arm_2_0, self.axis_2.x() - self.axis_2_0.x(),
                             self.axis_2.y() - self.axis_2_0.y())
        fiber = move_point(self.fiber_0, self.axis_2.x() - self.axis_2_0.x(),
                             self.axis_2.y() - self.axis_2_0.y())

        # Rotate arm 2
        c = cos(theta[1] - self.theta_2_0)
        s = sin(theta[1] - self.theta_2_0)
        self.arm_2 = rotate_polygon(arm_2, self.axis_2, c, s)
        self.fiber = rotate_point(fiber, self.axis_2, c, s)


    def _arm_angles(self, p):

        # Bearing of target
        t = atan2(p.y() - self.axis_1_0.y(), p.x() - self.axis_1_0.x())

        # Solve for the angles of a triangle formed by axis 1 (A), axis 2 (B)
        # and the fiber (C)
        a = self.axis_2_to_fiber
        b = distance(self.axis_1_0, p)
        c = self.axis_1_to_axis_2
        A = acos((b * b + c * c - a * a)/(2.0 * b * c))
        B = acos((a * a + c * c - b * b)/(2.0 * a * c))

        # Arm angles
        arm_1_1 = t - A
        arm_2_1 = arm_1_1 + pi - B

        # Alternate arm angles
        arm_1_2 = t + A
        arm_2_2 = arm_1_2 - pi + B

        return ([arm_1_1 - self.arm_1_angle_offset, arm_2_1 - self.arm_2_angle_offset],
                [arm_1_2 - self.arm_1_angle_offset, arm_2_2 - self.arm_2_angle_offset])
