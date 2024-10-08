from matplotlib import pyplot as plt
from math import pi

from positioner_model import add_random_targets
from positioner_model import focal_plane
from positioner_model import rerun
from positioner_model import simple_allocator

fp = focal_plane()
plt.interactive(True)
fp.live_view=True
add_random_targets(fp, density=6, ir=True)
add_random_targets(fp, density=5, vis_lr=True)
add_random_targets(fp, density=5, vis_hr=True)
fp.live_view=True
simple_allocator(fp)
fp.park_all()
fp.plot()
fp.live_view=True

for i in fp.positioners:
    if i.target and not i.in_position:
        i.trajectory_from_here_simultaneous(i.tpose)
rerun(fp, 0)
