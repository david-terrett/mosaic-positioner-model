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
        self.low_limit = low_limit
        self.high_limit = high_limit
        self._path = None


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
        self._path = path(self.position)
        for i in range(1, n):
            self._path.append(self.position + i * step_d)
        self._path.append(target)


    def step(self, back=False):
        """
        Moves the motor one step along its path

        Returns
        -------
        : bool
            Motor has reached its target
        """
        if back:
            p = self._path.prev()
        else:
            p = self._path.next()
        if p is not None:
            self.position = p
            return False
        return True


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
        self._p = [start]
        self._i = 0


    def append(self, p):
        """
        Add a point to the path

        Arguments
        ---------
        p : float
            position (degrees)
        """
        self._p.append(p)


    def start(self):
        """
        Start position

        Returns the first position in the path and resets the
        index to the beginning.

        Returns
        -------
        : float
            First position in path
        """
        self._i = 0
        return self._p[0]


    def next(self):
        """
        Next position

        Returns
        -------
        : float
            Next position in path or the last point if the motor is
            at the end of its path
        """
        self._i += 1
        if self._i >= len(self._p):
            return None
        return self._p[self._i]


    def prev(self):
        """
        Previous position in path or the first point if the motor
        is at the begging of its path

        Returns
        -------
        : float
            Previous position in path or None
        """
        self._i -= 1
        if self._i >= len(self._p):
            return self._p[-1]
        elif self._i < 0:
            return None
        return self._p[self._i]


def step_all(motors, back=False):
    """
    Step a set of motors

    Arguments
    ---------
    motors : iterable of motors
        Motors to step
    back : bool
        Step in reverse

    Returns
    -------
    : bool
        True if all the motors have reached the end of thier paths
    """
    in_pos = True
    for m in motors:
        if not m.step(back):
            in_pos = False
    return in_pos
