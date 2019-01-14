cmake_minimum_required(VERSION 2.8.3)
project(kraft_move)

add_compile_options(-std=c++11)

#set(CMAKE_CXX_COMPILER "clang++")
#set(CMAKE_C_COMPILER "clang") 
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_visual_tools
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

#message(STATUS "SSSTATUSSSSS" ${catkin_LIBRARIES})

add_executable(kraft_move src/kraft_move.cpp)
target_link_libraries(kraft_move ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${QT_LIBRARIES})
#install(TARGETS kraft_move DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
