cmake_minimum_required(VERSION 3.11)

set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/MOSAIC")

set(CMAKE_CXX_STANDARD 14)

project(geometry)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.53.0 REQUIRED)

find_package(Python 3.7
    COMPONENTS Interpreter Development REQUIRED
)

add_library(geometry SHARED
    geometry.cpp
)

target_include_directories(geometry PRIVATE
    "${Python_INCLUDE_DIRS}"
)

add_compile_definitions(BOOST_GEOMETRY_DISABLE_DEPRECATED_03_WARNING)
add_compile_definitions(BOOST_ALLOW_DEPRECATED_HEADERS)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/libgeometry.so"
    RENAME geometry.so
    DESTINATION .
)
