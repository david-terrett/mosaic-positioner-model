from matplotlib import pyplot as plt

from positioner_model import focal_plane
from positioner_model import simple_allocator

fp = focal_plane()
fp.add_random_targets(density=6, ir=True)
fp.add_random_targets(density=5, vis_lr=True)
fp.add_random_targets(density=5, vis_hr=True)
simple_allocator(fp)
fp.report()
fp.plot()
plt.show()
