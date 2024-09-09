from matplotlib import pyplot as plt

from positioner_model import focal_plane

fp = focal_plane()
fp.add_random_targets(density=4, ir=True)
fp.add_random_targets(density=2, vis_lr=True)
fp.add_random_targets(density=2, vis_hr=True)
fp.simple_allocator()
fp.plot()
plt.show()
