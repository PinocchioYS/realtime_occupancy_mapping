CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(realtime_occupancy_mapping)

SET(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    ADD_COMPILE_OPTIONS(-std=c++11)
else()
    ADD_COMPILE_OPTIONS(-std=c++11 -O3)
endif()

FIND_PACKAGE(catkin REQUIRED COMPONENTS
    roscpp
    tf
    sensor_msgs
    pcl_ros
)

find_package(superray_gridmap3d REQUIRED)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES realtime_occupancy_mapping
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GRIDMAP3D_INCLUDE_DIRS}
)

add_executable(realtime_occupancy_mapping
    src/realtime_occupancy_mapping.cpp
)
target_link_libraries(realtime_occupancy_mapping
    ${catkin_LIBRARIES}
    ${GRIDMAP3D_LIBRARIES}
)