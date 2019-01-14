cmake_minimum_required(VERSION 2.8.3)
project(kraft_teleop)

add_definitions(-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
#   genmsg
   roscpp
   rospy
#   std_msgs
   pluginlib
)

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  CATKIN_DEPENDS 
#	genmsg 
	roscpp 
	rospy 
#	std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(kraft_teleop src/kraft_teleop.cpp)
target_link_libraries(kraft_teleop ${catkin_LIBRARIES})
