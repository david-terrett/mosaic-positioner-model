This is a model of the MOSAIC fibre positioners written in Python. It can be
used to prototype target assignment strategies and explore the effect of
changes to the positioner design on the efficiency of target assignment.

Building the software requires that the Boost geometry library is installed
but boost geometry is an include file only library so all that is required
is that the boost include files are installed; there is no need to build the
boost binary libraries.

# Dependencies

- boost geometry
- matplotlib

# Unpacking

If you received the software as a git bundle (positioner_model.bundle) you
unpack it with:

    $ git clone positioner_model.bundle -b main

This will create a directory positioner_model containing the source code.

To update an existing copy from a new bundle

    $ git pull positioner_model.bundle main

# Installation with cmake

    $ cd positioner_model
    $ mkdir build
    $ cd build
    $ cmake .. -DCMAKE_INSTALL_PREFIX=somewhere_on_PYTHONPATH
    $ cmake --build .
    $ cmake --install .

If you have boost or Python installed in an unconventional place you may
also have to define BOOST_DIR, BOOST_INCLUDEDIR and/or Python_ROOT_DIR.

# Issue with anaconda

The supplied cmake file does not build the geometry library correctly for
use with anaconda.

# Example

    >>> from positioner_model import focal_plane

    >>> # Create a focal_plane
    >>> fp = focal_plane()

    >>> # Create some targets and compute which are reachable by each
    >>> # positioner. density is per square arcmin.
    >>> fp.create_random_targets(density=10)

    >>> # See how many targets were created
    >>> print(len(fp.targets))

    >>> # Run the simple allocation algorithm
    >>> fp.simple_allocator()

    >>> # Print a summary of the allocations
    >>> fp.report()

    >>> # Plot
    >>> import matplotlib.pyplot as plt
    >>> fp.plot()
    >>> plt.show()

    >>> # Get help on the focal plane model classes.
    >>> import focal_plane
    >>> help(focal_plane.focal_plane)
    >>> help(focal_plane.target)
    >>> help(focal_plane.positioner)

# The simple allocator

The simple_allocator method implements the following algorithm:

- Sort the positioners into ascending order of the number of targets
  the positioner can reach.

- For each positioner not yet allocated a target, sort the targets into
  ascending order of the number of positioners that can reach it and
  try placing the fiber on each target in both possible configurations
  until one that doesn't collide with another positioner is found.

- Repeat step 2 until no more targets can be assigned

# Contributing updates

If you fix any bugs or make enhancements that you want merged into the model,
please follow the following procedure.

If you are new to git, set your name and email address

    $ git config --global user.name "John Doe"
    $ git config --global user.email johndoe@example.com

Create a branch for your changes

    $ cd positioner_model
    $ git branch your_branch_name
    $ git checkout your_branch_name

Commit the files you have modified

    $ git add files_you_have_modified
    $ git commit -m "short message with changes listed"

Push this branch to Github

    $ git push -u origin your_branch_name

Follow the link from the output of the previous command to create a pull request.

Describe what kind of changes you have made (documentation changes, bug fixing, new features implemented, etc); any additional information that makes it easier for the reviewer to review the pull request.

Click on the green button "Create pull request".
