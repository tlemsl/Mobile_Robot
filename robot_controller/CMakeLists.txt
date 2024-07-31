cmake_minimum_required(VERSION 3.12)
project(robot_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

SET(CMAKE_EXE_LINKER_FLAGS 
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,${CMAKE_SOURCE_DIR}/build")

find_package(casadi REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# set the g++ compiler flags
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_LOADED)
    message(STATUS "Compiler Path: ${CMAKE_CXX_COMPILER}")
    message(STATUS "Compiler ID: ${CMAKE_CXX_COMPILER_ID}")
    message(STATUS "Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}")
endif()

# set the Ipopt library and include directory
set(LIBRARY_DIRS /usr/local/lib)
set(IPOPT_INCLUDE_DIRS /usr/include/coin)

# set casadi include directory
set(CASADI_INCLUDE_DIR /usr/local/include/casadi)

# find casadi library
find_library(CASADI_LIBRARY
    NAMES casadi
    HINTS ${CASADI_INCLUDE_DIR}/../lib $ENV{CASADI_PREFIX}/lib)
if(CASADI_LIBRARY)
    set(CASADI_LIBRARIES ${CASADI_LIBRARIES} ${CASADI_LIBRARY})
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  gazebo_msgs
  visualization_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Eigen3 REQUIRED)
# find_package(CasADi REQUIRED)

## Uncomment if the package has a setup.py
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_controller
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs gazebo_msgs visualization_msgs tf
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
include_directories(
    SYSTEM ${IPOPT_INCLUDE_DIRS}
    SYSTEM ${CASADI_INCLUDE_DIR}
    ${CODE_GEN_INCLUDE_DIR}
)
# library directories
link_directories(${LIBRARY_DIRS})

  
## Declare a C++ executable
add_executable(robot_controller_node src/MPCController.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robot_controller_node
  ${catkin_LIBRARIES}
  ${CASADI_LIBRARIES}
  ipopt
)

## Add cmake target dependencies of the executable
# add_dependencies(robot_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
