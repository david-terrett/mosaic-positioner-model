from matplotlib import pyplot as plt
from math import pi

from positioner_model import focal_plane
from positioner_model import rerun

fp = focal_plane()
plt.interactive(True)
fp.live_view=True
fp.create_random_targets(density=6.8)
fp.live_view=True
fp.simple_allocator()
fp.park_all()
fp.plot()
fp.live_view=True

for i in fp.positioners:
    if i.target and not i.in_position:
        i.trajectory_from_here_simultaneous(i.tpose)
rerun(fp, 0)