# -*- coding utf-8 -*-

class motor(object):
    """
    Model of a positioner motor

    Attributes
    ----------
    high_limit : double
        high limit (deg)
    low_limit : double
        low limit (deg)
    p : double
        Current motor position (deg)
    """


    def __init__(self, low_limit=-200.0, high_limit=200.0):
        """
        Create positioner

        Parameters
        ----------
        high_limit : double
            high limit (deg)
        low_limit : double
            low limit (deg)
        """
        self.p = 0.0
        self.low_limit = low_limit
        self.high_limit = high_limit


    def set(self, p):
        """
        Set motor position

        Parameters
        ----------
        p : double
        """
        if p < self.low_limit or p > self.high_limit:
            raise RuntimeError('Motor position out of range')
        self.p = p
