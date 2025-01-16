# -*- coding utf-8 -*-

def simple_configure(fp, log=False):
    """
    Simple positioner deployment algorithm

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
            if not pos.in_position and pos.target:

                # Try moving
                pos.zoom_to(fp.figure)
                if pos.move_to_target(axes=fp.axes, log=log):
                    progress = True
                print(pos.on_target, pos.in_position)
        if log:
            fp.report()
