CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(realtime_occupancy_mapping)

IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release)
ENDIF()

FIND_PACKAGE(catkin REQUIRED COMPONENTS
	roscpp
	tf
	sensor_msgs
	pcl_ros
)

## System dependencies are found with CMake's conventions
find_package(superray-gridmap3d REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES realtime_occupancy_mapping
 CATKIN_DEPENDS roscpp tf sensor_msgs pcl_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
	${catkin_INCLUDE_DIRS}
	${SUPERRAY_GRIDMAP3D_INCLUDE_DIRS}
)

##### Grid3D Real-time Occupancy Mapping Node #####
add_executable(realtime_occupancy_mapping
	src/realtime_occupancy_mapping.cpp
)
target_link_libraries(realtime_occupancy_mapping
	${catkin_LIBRARIES}
	${SUPERRAY_GRIDMAP3D_LIBRARIES}
)
