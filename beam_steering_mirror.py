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

        # Draw the mirror as a rectangle
        x = 20.0
        y = 100.0
        outline = polygon()
        outline.append(point(self.position.x() + x, self.position.y() + y))
        outline.append(point(self.position.x() + x, self.position.y() - y))
        outline.append(point(self.position.x() - x, self.position.y() - y))
        outline.append(point(self.position.x() - x, self.position.y() + y))
        outline.append(point(self.position.x() + x, self.position.y() + y))

        # Rotate into position
        c = cos(radians(self.mirror_motor.position))
        s = sin(radians(self.mirror_motor.position))
        outline = rotate_polygon(outline, self.position, c, s)

        # draw outline
        self._d.append(axes.plot(outline.x(), outline.y(), color='gray'))


    def point(self, t):
        """
        Point the mirror at a position in the focal plane.

        Parameters
        ----------
        t : target
            Target position
        """

        # Check that the target isn't too far away
        max_d = distance(self.position, point(0.0, 0.0)) + 50.0
        if distance(t.position, self.position) >max_d:
            raise RuntimeError("target is too far from mirror")
        a = degrees(atan2(t.position.y() - self.position.y(),
                          t.position.x() - self.position.x()))
        self.mirror_motor.set(a)
