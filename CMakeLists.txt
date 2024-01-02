cmake_minimum_required(VERSION 3.11)

set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/MOSAIC/python")

set(CMAKE_CXX_STANDARD 14)

project(poistioner)

add_subdirectory(geometry)

install(FILES
    focal_plane.py
    positioner.py
    target.py
    DESTINATION .
)
