CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(realtime_occupancy_mapping)

##### Compile options #####
IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE Release)
ENDIF()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##### Find dependencies #####
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

## System dependencies are found with CMake's conventions
find_package(superray-gridmap3d REQUIRED)

###########
## Build ##
###########
include_directories(
	${SUPERRAY_GRIDMAP3D_INCLUDE_DIRS}
)

##### Grid3D Real-time Occupancy Mapping Node #####
add_executable(realtime_occupancy_mapping
	src/realtime_occupancy_mapping.cpp
)
ament_target_dependencies(realtime_occupancy_mapping
  rclcpp
  tf2_ros
	sensor_msgs
	message_filters
	pcl_ros
	pcl_conversions
)
target_link_libraries(realtime_occupancy_mapping
	${SUPERRAY_GRIDMAP3D_LIBRARIES}
)

##### Install commands #####
install(TARGETS
	realtime_occupancy_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  bag
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()