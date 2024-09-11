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
    fp.clear_target_assignments()

    # sort the list of positioners so that we allocate the ones
    # with the fewest targets first
    positioners = sorted(fp.positioners,
                         key=lambda p: len(p.ir_targets) + len(p.vis_targets))

    # Until there are no unallocated positioners left...
    alloc = fp.ir_allocated + fp.vis_allocated
    while fp.ir_allocated + fp.vis_allocated < len(fp.positioners):
        for pos in positioners:

            # If this positioner doesn't have a target
            if not pos.target:

                # Try to assign an IR target
                for t in [*pos.ir_targets]:
                    if not t.positioner:
                        if fp._assign_target_to_positioner(pos, t, True):
                            break

                # If still no target then try for a visible one
                if not pos.target:
                    for t in [*pos.vis_targets]:
                        if not t.positioner:
                            if fp._assign_target_to_positioner(pos, t, False):
                                break

        # If that pass didn't find any more, give up.
        if alloc == fp.ir_allocated + fp.vis_allocated:
            break
        alloc = fp.ir_allocated + fp.vis_allocated
