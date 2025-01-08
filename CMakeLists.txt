cmake_minimum_required(VERSION 3.11)


set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/MOSAIC/python")

set(CMAKE_CXX_STANDARD 14)

project(positioner)

add_subdirectory(geometry)

install(FILES
    __init__.py
    add_random_targets.py
    demo_allocator.py
    focal_plane.py
    geometry_utilities.py
    motor.py
    positioner.py
    rerun.py
    resolve_conflict.py
    simple_allocator.py
    target.py
    DESTINATION positioner_model
)
