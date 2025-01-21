# -*- coding utf-8 -*-
if __name__ == "__main__" and __package__ is None:
    __package__ = "positioner_model"

from matplotlib import pyplot as plt

from .test_setup import test_setup

def simple_path_planner(fp):
    """
    Simple positioner path planning algorithm

    Just try moving each positioner in turn. This is a very poor algorithm
    that only manages to deploy a few positioners but tests the positioner
    move functions.

    Parameters
    ---------
    fp : focal_plane
        focal_plane
    """

    # Repeat until we don't make any more progress
    progress = True
    while progress:
        progress = False

        # For each positioner that isn't in position and has a target
        for pos in fp.positioners:
            if not pos.on_target and pos.target:

                # Try moving
                pos.zoom_to(fp.figure)
                if pos.move_to_target(axes=fp.axes):
                    print(pos.id, " moved to target")
                    progress = True
                else:
                    print(pos.id, " blocked by", pos.blocker.id)
        fp.report()

if __name__ == "__main__":
    fp = test_setup()
    plt.interactive(True)
    fp.plot()
    simple_path_planner(fp)
    plt.interactive(False)
    fp.plot()
    plt.show()
