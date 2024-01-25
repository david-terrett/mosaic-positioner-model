# -*- coding utf-8 -*-

from geometry import point

class target(object):
    """
    A target is a focal plane position, the positioner it is allocated
    to (if any) and the two sets of arm angles the put the fiber onto
    the target.

    Attributes
    ----------
        position : point
            Position of target in the focal plane

        positioner : positioner
            The positioner assigned to this target (can be None)

        reachable : [positioner]
            A list of the positioners that can reach this target
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
        self.reachable = []
