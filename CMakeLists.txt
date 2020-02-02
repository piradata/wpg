cmake_minimum_required(VERSION 2.8.3)
project(wpg)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  geographic_msgs
  libmavconn
  mavros
  mavros_msgs
)


catkin_package(
   CATKIN_DEPENDS geometry_msgs std_msgs
)


include_directories(include
	${catkin_INCLUDE_DIRS}
)


link_directories(
	${catkin_LIBRARY_DIRS}
)


## add_executable ( mains src/offb_node.cpp )

## target_link_libraries(mains ${catkin_LIBRARIES}  )

## add_definitions(-std=c++17)
