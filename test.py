from matplotlib import pyplot as plt

from positioner_model import add_random_targets
from positioner_model import focal_plane
from positioner_model import simple_allocator

fp = focal_plane()
add_random_targets(fp, density=6, ir=True)
add_random_targets(fp, density=5, vis_lr=True)
add_random_targets(fp, density=5, vis_hr=True)
simple_allocator(fp)
fp.report()
fp.plot()
plt.show()
