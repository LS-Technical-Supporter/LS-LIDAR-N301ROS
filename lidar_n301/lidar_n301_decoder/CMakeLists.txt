cmake_minimum_required(VERSION 2.8.3)
project(lidar_n301_decoder)

add_definitions(-std=c++0x)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  sensor_msgs
  pcl_ros
  pcl_conversions
  lidar_n301_msgs
)
find_package(Boost REQUIRED)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lidar_n301_decoder
  CATKIN_DEPENDS
    roscpp sensor_msgs pluginlib
    pcl_ros pcl_conversions
    lidar_n301_msgs
  DEPENDS
    Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

# Lidar N301 Decoder
add_library(lidar_n301_decoder
  src/lidar_n301_decoder.cpp
)
target_link_libraries(lidar_n301_decoder
  ${catkin_LIBRARIES}
)
add_dependencies(lidar_n301_decoder
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Lidar N301 Decoder node
add_executable(lidar_n301_decoder_node
  src/lidar_n301_decoder_node.cpp
)
target_link_libraries(lidar_n301_decoder_node
  lidar_n301_decoder
  ${catkin_LIBRARIES}
)
add_dependencies(lidar_n301_decoder_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS lidar_n301_decoder_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
