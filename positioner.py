# -*- coding utf-8 -*-

import matplotlib.pyplot as plt

from math import cos
from math import pi
from math import radians
from math import sin

from geometry import move_point
from geometry import move_polygon
from geometry import point
from geometry import polygon
from geometry import rotate_point
from geometry import rotate_polygon

class positioner(object):


    def __init__(self, position):

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

        # Park the positioner
        self.park()

    def park(self):
        self.pose(0.0, pi)


    def plot(self):
        plt.plot(self.arm_1.x(), self.arm_1.y(), color='black')
        plt.plot(self.arm_2.x(), self.arm_2.y(), color='black')
        plt.plot(self.axis_1_0.x(), self.axis_1_0.y(), '+')
        plt.plot(self.axis_2.x(), self.axis_2.y(), '+')
        plt.plot(self.fiber.x(), self.fiber.y(), 'o')


    def pose(self, theta_1, theta_2):

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
