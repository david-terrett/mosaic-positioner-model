# -*- coding utf-8 -*-
if __name__ == "__main__" and __package__ is None:
    __package__ = "positioner_model"

from .add_random_targets import add_random_targets
from .focal_plane import focal_plane
from .simple_allocator import simple_allocator

def test_setup():
    fp = focal_plane()
    add_random_targets(fp, density=6, ir=True)
    add_random_targets(fp, density=5, vis_lr=True)
    add_random_targets(fp, density=5, vis_hr=True)
    simple_allocator(fp)
    fp.park_all()
    return fp
