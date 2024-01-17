This is a model of the MOSAIC fibre positioners written in Python. It can be
used to prototype target assignment strategies and explore the effect of
changes to the positioner design on the efficiency of target assignment.

Building the software requires that the Boost geometry library is installed
but boost geometry is an include file only library so all that is required
is that the boost include files are installed; there is no need to build the
boost binary libraries.

Installation with cmake
-----------------------
cd src
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=somwhere_on_PYTHONPATH
cmake --build .
cmake --install .

If you have boost or Python installed in an unconventional place you may
also have to define BOOST_DIR, BOOST_INCLUDEDIR and/or Python_ROOT_DIR.

Example
-------
from focal_plane import focal_plane

# Create a focal_plane
fp = focal_plane()

# Create some targets and compute which are reachable by each positioner.
# density is per square arcmin.
fp.create_random_targets(density=10)

# See how many targets were created
print(len(fp.targets))

# Run the simple allocation algorithm
fp.simple_allocator()

# See how many positions are left without targets
print(len(fp.positioners) - fp.allocated)

# Plot - fibers without targets are plotted in red.
import matplotlib.pyplot as plt
fp.plot(plt).show()
