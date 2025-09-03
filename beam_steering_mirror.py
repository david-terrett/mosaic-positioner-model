# -*- coding utf-8 -*-

from math import atan2
from math import cos
from math import degrees
from math import sin
from math import sqrt
from math import radians
from matplotlib import pyplot as plt

from .geometry import distance
from .geometry import point
from .geometry import polygon
from .geometry_utilities import rotate_polygon
from .motor import motor

class beam_steering_mirror(object):
    """
    Beam steering mirror
    Attributes
    ----------
    mirror_motor : motor
        mirror axis motor
    """

    def __init__(self, position):
        """
        Create beam steering mirror

        Parameters
        ----------
        position : point
            Position of mirror rotation axis in focal plane
        """
        self.position = position
        self._d = []

        # Calculate the mid point of travel for the mirror.
        mid = degrees(atan2(-position.y(), -position.x()))
        self.mirror_motor = motor(low_limit=mid - 45.0,
                                  high_limit=mid + 45.0)
        self.mirror_motor.set(mid)

    def plot(self, axes):
        """
        Plot the mirror outline

        Parameters
        ----------
            axes : matplotlib axes
                plot axes
        """
        # Delete existing drawing
        for d in self._d:
            d[0].remove()
        self._d = []

        # Draw the mirror as a hexagon
        x = 100.0
        l1 = x / 2.0
        l2 = x / 2.0 + sqrt(0.5) * x
        outline = polygon()
        outline.append(point(self.position.x() + l1, self.position.y() + l2))
        outline.append(point(self.position.x() + l2, self.position.y() + l1))
        outline.append(point(self.position.x() + l2, self.position.y() - l1))
        outline.append(point(self.position.x() + l1, self.position.y() - l2))
        outline.append(point(self.position.x() - l1, self.position.y() - l2))
        outline.append(point(self.position.x() - l2, self.position.y() - l1))
        outline.append(point(self.position.x() - l2, self.position.y() + l1))
        outline.append(point(self.position.x() - l1, self.position.y() + l2))
        outline.append(point(self.position.x() + l1, self.position.y() + l2))

        # Rotate into position
        c = cos(radians(self.mirror_motor.position))
        s = sin(radians(self.mirror_motor.position))
        outline = rotate_polygon(outline, self.position, c, s)

        # draw outline
        self._d.append(axes.plot(outline.x(), outline.y(), color='gray'))

        # Add an arrow pointing at the target
        a = radians(self.mirror_motor.position)
        x = self.position.x() + 150.0 * cos(a)
        y = self.position.y() + 150.0 * sin(a)
        self._d.append(axes.plot([self.position.x(), x], [self.position.y(), y],
                       color='grey'))



    def point(self, t):
        """
        Point the mirror at a position in the focal plane.

        Parameters
        ----------
        t : any
            Target position
        """

        # Check that the target isn't too far away
        max_d = distance(self.position, point(0.0, 0.0)) + 50.0
        if distance(t.position, self.position) >max_d:
            raise RuntimeError("target is too far from mirror")
        a = degrees(atan2(t.position.y() - self.position.y(),
                          t.position.x() - self.position.x()))
        self.mirror_motor.set(a)
