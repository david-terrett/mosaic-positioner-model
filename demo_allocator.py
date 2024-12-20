# -*- coding utf-8 -*-
from .focal_plane import focal_plane
from .positioner import positioner
from .target import target

def demo_allocator(fp):
    """
    Target to positioner assignment algorithm that demostrates the use of the
    collission matrix. It uses the same algorithm as the simple allocator
    which takes less time than building the collision matrix.

    Parameters
    ---------
    fp : focal_plane
        focal_plane
    """

    # A collision matrix is required
    if fp.collision_matrix is None:
        fp.build_collision_matrix()

    fp.clear_all_target_assignments()

    # sort the list of positioners so that we allocate the ones
    # with the fewest targets first
    # positioners = sorted(fp.positioners, key=lambda p: len(p.targets))

    # Until there are no unallocated positioners left...
    for p in range(len(fp.positioners)):

        # If this positioner doesn't have a target
        if fp.pos_to_targ_array[p, 0] == -1:

            # Try each reachable target in turn
            n = 0
            for t in range(len(fp.reachable_targets[p])):

                # If this one doesn't exist, give up
                if fp.reachable_targets[p, t] == -1:
                    break

                # Try this target unless it is already assigned to a positioner
                if fp.targ_to_pos_array[n] == -1:
                    if not fp.will_collide(p, n, 0):

                        # Record which positioner this target is assigned to
                        fp.targ_to_pos_array[fp.reachable_targets[p, t]] = p

                        # And which target is assigned to this positioner
                        fp.pos_to_targ_array[p] = [t, 0]
                        break
                    else:

                        # Try the alternate pose
                        if not fp.will_collide(p, n, 1):
                            fp.targ_to_pos_array[fp.reachable_targets[p, t]] = p
                            fp.pos_to_targ_array[p] = [t, 1]
                            break
                n += 1

    # Move all the positioners onto their targets
    p = 0
    for t in fp.pos_to_targ_array:
        pos = fp.positioners[p]
        if t[0] != -1:
            targ = list(pos.targets)[t[0]]
            fp.assign_target_to_positioner(pos, targ, t[1],
                                       collision_check=False)
        p += 1

    # Look for places for the unallocated positioners
    for pos in fp.positioners:
        if not pos.target:
            pos.uncollide()
