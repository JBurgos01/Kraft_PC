cmake_minimum_required(VERSION 2.8.3)
project(kraft_planner)

add_definitions(-std=c++11)
add_compile_options(-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  rospy
  tf
  image_transport
  pluginlib
  trac_ik_lib
  geometry_msgs
  kdl_conversions
  tf_conversions
)

# find_package(Eigen REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mbzirc_planner
  CATKIN_DEPENDS cmake_modules moveit_core moveit_ros_planning moveit_ros_planning_interface roscpp rospy sensor_msgs tf trac_ik_lib
  DEPENDS boost
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_executable(kraft_planner src/kraft_planner.cpp)
target_link_libraries(kraft_planner ${catkin_LIBRARIES})

add_executable(kraft_planner_openrave src/kraft_planner_openrave.cpp)
target_link_libraries(kraft_planner_openrave ${catkin_LIBRARIES})

add_executable(kraft_planner_tracik src/kraft_planner_tracik.cpp)
target_link_libraries(kraft_planner_tracik ${catkin_LIBRARIES})

add_executable(test_points src/test_points.cpp)
target_link_libraries(test_points ${catkin_LIBRARIES})

add_executable(kraft_connect src/kraft_connect.cpp)
target_link_libraries(kraft_connect ${catkin_LIBRARIES})

add_executable(kraft_manipulate src/kraft_manipulate.cpp)
target_link_libraries(kraft_manipulate ${catkin_LIBRARIES})
