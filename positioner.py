# -*- coding utf-8 -*-

import matplotlib.pyplot as plt

from math import cos
from math import fabs
from math import pi
from math import radians
from math import sin
from math import sqrt

from geometry import move_point
from geometry import move_polygon
from geometry import point
from geometry import polygon
from geometry import rotate_point
from geometry import rotate_polygon

class positioner(object):


    def __init__(self, position):
        """
        Create positioner

        Parameters
        ----------
            position : point
                position of positioner axis 1 in focal plane
        """

        # Define the rotation axis of arm 1 for a positioner at placed at 0,0
        axis_1 = point(0.0, 0.0)

        # Construct outline of lower positioner arm in the park position
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

        # Outline of arm 2 with axis at 0.0
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
        self.theta_1_0 = 0.0
        self.theta_2_0 = pi

        # Define the maximum and minimum radius the fibre can reach from the
        # arm 1 axis
        axis_1_to_axis_2 = sqrt((axis_1.x() - axis_2.x()) *
                               (axis_1.x() - axis_2.x()) +
                               ((axis_1.y() - axis_2.y()) *
                               (axis_1.y() - axis_2.y())))
        axis_2_to_fiber = sqrt((axis_2.x() - fiber.x()) *
                               (axis_2.x() - fiber.x()) +
                               ((axis_2.y() - fiber.y()) *
                               (axis_2.y() - fiber.y())))
        self.max_r = axis_1_to_axis_2 + axis_2_to_fiber
        self.min_r = fabs(axis_2_to_fiber - axis_1_to_axis_2)

        # Park the positioner
        self.park()

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


    def park(self):
        """
        Park the positioner
        """
        self.pose(0.0, pi)


    def plot(self):
        """
        Plot the positioner outline
        """
        plt.plot(self.arm_1.x(), self.arm_1.y(), color='black')
        plt.plot(self.arm_2.x(), self.arm_2.y(), color='black')
        plt.plot(self.axis_1_0.x(), self.axis_1_0.y(), '+')
        plt.plot(self.axis_2.x(), self.axis_2.y(), '+')
        plt.plot(self.fiber.x(), self.fiber.y(), 'o')


    def pose(self, theta_1, theta_2):
        """
        Set the axis positions

        Parmeters
        ---------
        theta_1 : float
            Angle (relative to y axis) of axis 1 (radians)
        theta_2 : float
            Angle (relative to y axis) of axis 2 (radians)
        """

        # Rotate arm 1
        c = cos(theta_1 - self.theta_1_0)
        s = sin(theta_1 - self.theta_1_0)
        self.arm_1 = rotate_polygon(self.arm_1_0, self.axis_1_0, c, s)
        self.axis_2 = rotate_point(self.axis_2_0, self.axis_1_0, c, s)

        # Move arm 2 to the new axis 2 position.
        arm_2 = move_polygon(self.arm_2_0, self.axis_2.x() - self.axis_2_0.x(),
                             self.axis_2.y() - self.axis_2_0.y())
        fiber = move_point(self.fiber_0, self.axis_2.x() - self.axis_2_0.x(),
                             self.axis_2.y() - self.axis_2_0.y())

        # Rotate arm 2
        c = cos(theta_2 - self.theta_2_0)
        s = sin(theta_2 - self.theta_2_0)
        self.arm_2 = rotate_polygon(arm_2, self.axis_2, c, s)
        self.fiber = rotate_point(fiber, self.axis_2, c, s)
