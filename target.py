# -*- coding utf-8 -*-

from .geometry import point

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

    def __init__(self, x, y, ir=False, vis_lr=False, vis_hr=False):
        """
        Parameters
        ----------
        x : float
            x position in focal plane
        y : float
            y position in focal plane
        ir : bool
            target is IR
        vis_lr : bool
            target is visible low res
        vis_hr : bool
            target is visible high res
        """
        self.position = point(x, y)
        self.positioner = None
        self.reachable = []
        self.ir = ir
        self.vis_lr = vis_lr
        self.vis_hr = vis_hr
