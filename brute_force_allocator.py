# -*- coding utf-8 -*-

from .focal_plane import focal_plane
from .positioner import positioner
from .target import target

_best_so_far = 0


def brute_force_allocator(fp):
    """
    Brute force target to positioner assignment algorithm

    Parameters
    ---------
    fp : focal_plane
        focal_plane
    """

    # A collision matrix is required
    if not fp.collision_matrix:
        fp.build_collision_matrix()

    fp.clear_all_target_assignments()
    _config = fp.positioners.copy()
    _scan_targets(fp, 0)
    print(_best_so_far)
    fp.positioners = _config.copy()

def _scan_targets(fp, np):
    pos = fp.positioners[np]
    next_target = target_iterator(pos)
    targets = iter(next_target)

    # Try each one in turn
    for t in targets:
        if not t.positioner:
            if not pos.will_collide(t, False):
                fp.assign_target_to_positioner(pos, t, False,
                                               collision_check=False)

                # Success so move onto the next positioner
                #if np  + 1 < len(fp.positioners):
                if np  + 1 < 10:
                    _scan_targets(fp, np + 1)

                # Or if we are already at the end compare this with the best
                # configuration so far
                else:
                    global _best_so_far
                    global _config
                    goodness = fp.ir_allocated + fp.vis_allocated
                    if goodness > _best_so_far:
                        _best_so_far = goodness;
                        _config = fp.positioners.copy()
                        print('found a better one', _best_so_far)

    # No target found
    if np  + 1 < 10:
         _scan_targets(fp, np + 1)
