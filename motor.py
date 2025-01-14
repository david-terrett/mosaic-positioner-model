# -*- coding utf-8 -*-

from math import trunc
from typing import NamedTuple

class motor(object):
    """
    Model of a positioner motor

    Attributes
    ----------
    high_limit : float
        high limit (deg)
    low_limit : float
        low limit (deg)
    position : float
        Current motor position (deg)
    """


    def __init__(self, low_limit=-200.0, high_limit=200.0):
        """
        Create positioner

        Parameters
        ----------
        high_limit : float
            high limit (deg)
        low_limit : float
            low limit (deg)
        """
        self.position = 0.0
        self.path = None
        self.low_limit = low_limit
        self.high_limit = high_limit


    def set(self, position):
        """
        Set motor position

        Parameters
        ----------
        position : float
            motor position (degrees)
        """
        if position < self.low_limit or position > self.high_limit:
            raise RuntimeError('Motor position out of range')
        self.position = position


    def set_path(self, target, step=0.1):
        """
        Set path assuming that the motor runs at maximum speed

        Parameters
        ----------
        target : float
            target position (degrees)
        step : float
            step size (seconds)
        """

        # Calculate the step size in degrees. Assume 18 deg/s
        step_d = step * 18.0
        if target < self.position:
            step_d = -step_d

        # Calculate how many steps to get within 1 step of the end
        n = int(trunc((target - self.position) / step_d)) + 1

        # Generate the path
        self.path = path(self.position)
        for i in range(1, n):
            self.path.append(self.position + i * step_d)
        self.path.append(target)


    def step(self, back=False):
        """
        Moves the motor one step along its path

        Returns
        -------
        boolean
            Motor has reached its target
        """
        if back:
            p = self.path.prev()
        else:
            p = self.path.next()
        if p is not None:
            self.position = p
            return False
        return True


class motor_positions(NamedTuple):
    alpha: float
    alpha_alt: float
    beta: float
    beta_alt: float


class path(object):
    """
    A path is (for now) just a list of motor positions
    """
    def __init__(self, start):
        """
        Arguments
        ---------
        start : float
            start position
        """
        self.p = [start]
        self.i = 0


    def append(self, p):
        """
        Add a point to the path

        Arguments
        ---------
        p : float
            position (degrees)
        """
        self.p.append(p)


    def start(self):
        """
        Start position

        Returns the first position in the path and resets the
        index to the beginning.

        Returns
        -------
        float
            First position in path
        """
        self.i = 0
        return self.p[0]


    def next(self):
        """
        Next position

        Returns
        -------
        float
            Next position in path or None if there are no more points
        """
        self.i += 1
        if self.i >= len(self.p):
            return None
        return self.p[self.i]


    def prev(self):
        """
        Previous position

        Returns
        -------
        float
            Previous position in path or None
        """
        self.i -= 1
        if self.i >= len(self.p):
            return self.p[-1]
        elif self.i < 0:
            return None
        return self.p[self.i]


def step_all(motors, back=False):
    in_pos = True
    for m in motors:
        if not m.step(back):
            in_pos = False
    return in_pos
