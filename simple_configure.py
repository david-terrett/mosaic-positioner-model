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
            if not pos.in_position and pos.target:

                # Try moving
                if pos.move_to_target():
                    progress = True
        fp.report()
