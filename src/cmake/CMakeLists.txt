cmake_minimum_required(VERSION 3.10)

project(maestro_mujoco)
add_compile_options(-std=c++11 -lorocos-kdl)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -Wl,-no-as-needed")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/src/cmake")

# You may need to adjust the path to Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS} -I/usr/include/eigen3)

# Include Mujoco if needed
# find_package(Mujoco REQUIRED)
# include_directories(${MUJOCO_INCLUDE_DIRS})
# link_directories(${MUJOCO_LIBRARY_DIRS})

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
    /home/despinar/mujoco_ws/maestro_mujoco/src/include
    /home/despinar/Workspaces/ProbabilisticContactEstimation_cpp/kdepp/include/kdepp
    /home/despinar/Workspaces/ProbabilisticContactEstimation_cpp/include    
    ${NLOPT_INCLUDE_DIRS}
    ${OROCOS_KDL_INCLUDE_DIRS}
)

set(MAESTRO_CPP
    ../main/FSM.cpp
    ../main/Robot.cpp
    ../main/Leg.cpp
    ../main/Controller.cpp
    ../main/Data.cpp
    ../main/TopLevelControl.cpp
    ../main/LocomotionTopLevelControl.cpp
    ../main/TrajectoryTopLevelControl.cpp
    ../main/Math.cpp
    ../main/Trajectory.cpp
    ../main/Wrapper.cpp
    ../main/LocomotionController.cpp
    ../main/TrajController.cpp
    /home/despinar/Workspaces/ProbabilisticContactEstimation_cpp/src/pce.cpp
)

add_executable(walk ../main/walkMujoco.cpp ${MAESTRO_CPP})
target_link_libraries(walk ${NLOPT_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} m mujoco glfw)

add_executable(dynawalk ../main/dynawalkMujoco.cpp ${MAESTRO_CPP})
target_link_libraries(dynawalk ${NLOPT_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} m mujoco glfw)


# add_executable(trajectory ../main/trajMujoco.cpp ${MAESTRO_CPP})
# target_link_libraries(trajectory ${NLOPT_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} m mujoco glfw)


# add_executable(explore ../main/explorationMujoco.cpp ${MAESTRO_CPP})
# target_link_libraries(explore ${NLOPT_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} m mujoco glfw)
