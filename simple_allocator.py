# -*- coding utf-8 -*-
from .focal_plane import focal_plane
from .positioner import positioner
from .target import target

def simple_allocator(fp):
    """
    Simple target to positioner assignment algorithm

    Parameters
    ---------
    fp : focal_plane
        focal_plane
    """
    fp.clear_all_target_assignments()

    # sort the list of positioners so that we allocate the ones
    # with the fewest targets first
    positioners = sorted(fp.positioners, key=lambda p: len(p.targets))

    for pos in positioners:

        # If this positioner doesn't have a target
        if not pos.target:

            # Create an iterator for this positioner's targets
            for t in pos.targets:
                if not t.positioner:
                    if pos.try_assigning_target(t, alt=False,
                                                ignore_no_target=True):
                        pos.set_pose_to_target()
                        break
                    else:
                        if pos.try_assigning_target(t, alt=True,
                                                    ignore_no_target=True):
                            pos.set_pose_to_target()
                            break

    # Look for places for the unallocated positioners
    for pos in fp.positioners:
        if not pos.target:
            try:
                pos.uncollide()
            except:
                print("Warning: no position found for positioner", pos.id)
