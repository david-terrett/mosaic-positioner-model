# -*- coding utf-8 -*-
if __name__ == "__main__" and __package__ is None:
    __package__ = "positioner_model"

from matplotlib import pyplot as plt

from .focal_plane import focal_plane
from .add_random_targets import add_random_targets

_max_allocated = 0
_allocated = 0
_config = []


def brute_force_allocator(fp, max_depth):
    """
    Brute force target to positioner assignment algorithm

    Parameters
    ---------
    fp : focal_plane
        Focal_plane
    max_depth : int
        Number of positioners
    """

    # A collision matrix is required
    if not fp.collision_matrix:
        fp.build_collision_matrix()

    fp.clear_all_target_assignments()
    global _config
    _scan_targets(fp, 0, max_depth)

    # Move all the positioners onto their targets
    p = 0
    for t in _config:
        pos = fp.positioners[p]
        if t[0] != -1:
            targ = list(pos.targets)[t[0]]
            pos.try_assigning_target(targ, t[1],
                                     collision_check=False)
        p += 1

    # Look for places for the unallocated positioners
    for pos in fp.positioners:
        if not pos.target:
            pos.uncollide()

def _scan_targets(fp, p, max_depth):

    # Try each target reachable by this positioner in turn
    for t in range(len(fp.reachable_targets[p])):
        if fp.reachable_targets[p, t] != -1:
            if not fp.will_collide(p, t, 0):

                # If we are now replacing an existing allocation, increment
                # the number of allocated positioners
                if fp.pos_to_targ_array[p][0] == -1:
                    global _allocated
                    _allocated += 1
                else:
                    # Remove existing allocation
                    fp.targ_to_pos_array[t] = -1

                # Set the new allocation
                fp.targ_to_pos_array[fp.reachable_targets[p, t]] = p
                fp.pos_to_targ_array[p] = [t, 0]

                # move onto the next positioner
                if p  + 1 < max_depth:
                    _scan_targets(fp, p + 1, max_depth)

                # If we have reached the maximum positioner compare this
                # configuration with the best so far
                else:
                    global _max_allocated
                    global _config
                    if _allocated > _max_allocated:
                        _max_allocated = _allocated
                        _config = fp.pos_to_targ_array.copy()

    # No target found so try the next positioner
    if p + 1 < max_depth:
         _scan_targets(fp, p + 1, max_depth)

    # We have finished with this positioner and going up a level
    # so if there is a target allocated, unallocate it and decrement
    # the total allocated.
    t =  fp.pos_to_targ_array[p][0]
    if t != -1:
        _allocated -= 1
        fp.targ_to_pos_array[t] = -1
        fp.pos_to_targ_array[p] = [-1, -1]



if __name__ == "__main__":
    fp = focal_plane()
    add_random_targets(fp, density=6, ir=True)
    add_random_targets(fp, density=5, vis_lr=True)
    add_random_targets(fp, density=5, vis_hr=True)
    brute_force_allocator(fp, 10)
    fp.report()
    fp.plot()
    plt.show()
