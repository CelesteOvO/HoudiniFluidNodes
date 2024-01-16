cmake_minimum_required(VERSION 3.26)

project(HinaPE-jet)

set(CMAKE_CXX_STANDARD 17)
set(TARGET SIM_JETFluid)

# Find Houdini
if (MSVC)
    set(Houdini_PATH "E:/Program Files/Side Effects Software/Houdini 20.0.547")
elseif (APPLE)
    set(Houdini_PATH "/Applications/Houdini/Houdini20.0.547/Frameworks/Houdini.framework/Versions/20.0/Resources")
endif ()
set(Houdini_DIR ${Houdini_PATH}/toolkit/cmake)
find_package(Houdini REQUIRED)

# Load Source
add_library(
        ${TARGET}
        SHARED
        Entrance.cpp
        GAS_Initialize.cpp
        GAS_Initialize.h
        GAS_BuildNeighborList.cpp
        GAS_BuildNeighborList.h
        GAS_InitializeMass.cpp
        GAS_InitializeMass.h
        GAS_UpdateDensity.cpp
        GAS_UpdateDensity.h
        GAS_UpdatePositionAndVelocity.cpp
        GAS_UpdatePositionAndVelocity.h
        GAS_AddExternalForce.cpp
        GAS_AddExternalForce.h
        GAS_ClearForce.cpp
        GAS_ClearForce.h
        GAS_ComputePressureForce.cpp
        GAS_ComputePressureForce.h
        GAS_ComputeViscosityForce.cpp
        GAS_ComputeViscosityForce.h

        size.h
        sph_kernel.cpp
        sph_kernel.h
        point_neighbor_search.cpp
        point_neighbor_search.h
)

# Link Houdini Toolkit
target_link_libraries(
        ${TARGET}
        PUBLIC
        Houdini
        jet
)
target_link_directories(
        ${TARGET}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        install/lib
)
target_include_directories(
        ${TARGET}
        PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        install/include
)
target_compile_options(
        ${TARGET}
        PRIVATE
        -D_LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION
)
houdini_configure_target(${TARGET})

# Build Test
set(TEST_TARGET mytest)
add_executable(
        ${TEST_TARGET}
        test/main.cpp
)
target_link_libraries(
        ${TEST_TARGET}
        PUBLIC
        Houdini
        jet
)
target_link_directories(
        ${TEST_TARGET}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        install/lib
)
target_include_directories(
        ${TEST_TARGET}
        PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        install/include
)