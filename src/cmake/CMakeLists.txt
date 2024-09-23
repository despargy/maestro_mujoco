cmake_minimum_required(VERSION 3.10)

project(maestro_mujoco)
add_compile_options(-std=c++11 -lorocos-kdl)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -Wl,-no-as-needed")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/src/cmake")

set(PCE_CPP ${PCE_CPP} "/home/despinar/Workspaces/ProbabilisticContactEstimation")

# You may need to adjust the path to Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS} -I/usr/include/eigen3)

# Include other dependencies
# Include directories
find_path(NLOPT_INCLUDE_DIRS
    NAMES nlopt.h
    HINTS ${PC_NLOPT_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Libraries
find_library(NLOPT_LIBRARIES
    NAMES nlopt nlopt_cxx
    HINTS ${PC_NLOPT_LIBDIR})



# find_package(nlopt REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
include_directories(${orocos_kdl_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(
    ../../include
    "${PCE_CPP}/kdepp/include/kdepp/"
    "${PCE_CPP}/include"    
    ${NLOPT_INCLUDE_DIRS}
    ${OROCOS_KDL_INCLUDE_DIRS}
)

set(MAESTRO_CPP
    ../FSM.cpp
    ../Robot.cpp
    ../Leg.cpp
    ../Controller.cpp
    ../Data.cpp
    ../TopLevelControl.cpp
    ../LocomotionTopLevelControl.cpp
    ../TrajectoryTopLevelControl.cpp
    ../Math.cpp
    ../Trajectory.cpp
    ../Wrapper.cpp
    ../LocomotionController.cpp
    ../TrajController.cpp
    "${PCE_CPP}/src/pce.cpp"
)

add_executable(dynawalk ../dynawalkMujoco.cpp ${MAESTRO_CPP} )
target_link_libraries(dynawalk ${NLOPT_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} m mujoco glfw)
