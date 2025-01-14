from matplotlib import pyplot as plt

from positioner_model import add_random_targets
from positioner_model import focal_plane
from positioner_model import rerun
from positioner_model import simple_allocator
from positioner_model import simple_configure

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

#rerun(fp, 0, pause=False)
simple_configure(fp)
fp.report()
