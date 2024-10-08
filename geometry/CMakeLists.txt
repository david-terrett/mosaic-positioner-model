set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra)
add_compile_options(-Wno-cast-function-type)
add_compile_options(-Wno-missing-field-initializers)
if(APPLE)
    add_compile_options(-Wno-deprecated-declarations)
endif()

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.53.0 REQUIRED)

find_package(Python 3.7
    COMPONENTS Interpreter Development REQUIRED
)

Python_add_library(geometry SHARED
    geometry.cpp
)

target_include_directories(geometry PRIVATE
    "${Python_INCLUDE_DIRS}"
    "${Boost_INCLUDE_DIRS}"
)

add_compile_definitions(BOOST_GEOMETRY_DISABLE_DEPRECATED_03_WARNING)
add_compile_definitions(BOOST_ALLOW_DEPRECATED_HEADERS)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/libgeometry${CMAKE_SHARED_LIBRARY_SUFFIX}"
    RENAME geometry.so
    DESTINATION positioner_model
)
