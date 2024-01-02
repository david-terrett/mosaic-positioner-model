# -*- coding utf-8 -*-

from math import pi

from geometry import point

class target(object):
    """
    A target is a focal plane position, the positioner it is allocated
    to (if any) and the two sets of arm angles the put the fiber onto
    the target.
    """

    def __init__(self, x, y):
        """
        Parameters
        ----------
        x : float
            x position in focal plane
        y : float
            y position in focal plane
        """
        self.position = point(x, y)
        self.positioner = None
