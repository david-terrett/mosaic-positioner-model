# -*- coding utf-8 -*-
from random import random

from .target import target

def add_random_targets(fp, density, ir=False, vis_lr=False, vis_hr=False,
                       ifu=False):
    """
    Create targets in random positions and add them to the focal plane

    Parameters
    ---------
    fp : focal_plane
        focal_plane
    density : float
        Density of targets to create (number per arcmin^2)
    ir : bool
        create IR targets
    vis_lr : bool
        create VIS low res targets
    vis_hr : bool
        create VIS high res targets
    ifu : bool
        create IFU targets
    """

    # Plate scale (mm/arcsec)
    plate_scale = 3.316

    # Area to cover (arcsec^2)
    area = ((fp.x_max - fp.x_min) * (fp.y_max - fp.y_min) /
            (plate_scale * plate_scale))

    # Number of targets to create
    n = int(density * area / 3600.0)

    targets = []
    for _ in range(0, n):
        x = fp.x_min + random() * (fp.x_max - fp.x_min)
        y = fp.y_min + random() * (fp.y_max - fp.y_min)
        targets.append(target(x, y, ir=ir, vis_lr=vis_lr, vis_hr=vis_hr,
                              ifu=ifu))
    fp.add_targets(targets)
