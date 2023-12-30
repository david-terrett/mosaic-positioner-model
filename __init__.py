# -*- coding utf-8 -*-

from .geometry import point, polygon

def move_point(p, dx, dy):
    """
    Move a point

    Parameters
    ----------
    p : point
        point to be moved
    dx : float
        shift in x
    dy : float
        shift in y

    Returns
    -------
    point :
        Moved point
    """
    return point(p.x() + dx, p.y() + dy)


def rotate_point(p, axis, cos_t, sin_t):
    """
    rotate a point around another point

    Parameters
    ----------
    p : point
        point to be moved
    axis : point
        axis of rotation
    cos_t : float
        cosine of angle
    sin_t : float
        sine of angle

    Returns
    -------
    point :
        Moved point
    """
    dx = p.x() - axis.x()
    dy = p.y() - axis.y()
    dx1 = dx * cos_t - dy * sin_t
    dy1 = dx * sin_t + dy * cos_t
    return point(axis.x() + dx1, axis.y() + dy1)


def move_polygon(p, dx, dy):
    """
    Move a polygon

    Parameters
    ----------
    p : polygon
        polygon to be moved
    dx : float
        shift in x
    dy : float
        shift in y

    Returns
    -------
    polygon :
        Moved polygon
    """
    poly = polygon()
    for p1 in p.points():
        poly.append(move_point(p1, dx, dy))
    return poly


def rotate_polygon(p, axis, cos_t, sin_t):
    """
    rotate a polygon around a point

    Parameters
    ----------
    p : polygon
        polygon to be moved
    axis : point
        axis of rotation
    cos_t : float
        cosine of angle
    sin_t : float
        sine of angle

    Returns
    -------
    polygon :
        Moved polygon
    """
    poly = polygon()
    for p1 in p.points():
        poly.append(rotate_point(p1, axis, cos_t, sin_t))
    return poly
