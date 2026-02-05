
# -*- coding utf-8 -*-

from .add_random_targets import add_random_targets
from .focal_plane import focal_plane
from .simple_allocator import simple_allocator


def allocation_test(tests, density):

    unallocated = []
    fp = focal_plane()
    for i in range(tests):
        fp.clear_all_target_assignments()
        fp.clear_targets()
        add_random_targets(fp, density, ir=True)
        add_random_targets(fp, density, vis_hr=True)
        add_random_targets(fp, density, vis_lr=True)
        print(len(fp.targets), " targets")
        simple_allocator(fp)
        n = 0
        for p in fp.positioners:
            if p.type != 0:
                if not p.target:
                    n += 1
        unallocated.append(n)
    return unallocated
