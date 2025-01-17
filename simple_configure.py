# -*- coding utf-8 -*-

def simple_configure(fp):
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
            if not pos.on_target and pos.target:

                # Try moving
                pos.zoom_to(fp.figure)
                if pos.move_to_target(axes=fp.axes):
                    print(pos.id, " moved to target")
                    progress = True
                else:
                    print(pos.id, " blocked by", pos.blocker.id)
        fp.report()
