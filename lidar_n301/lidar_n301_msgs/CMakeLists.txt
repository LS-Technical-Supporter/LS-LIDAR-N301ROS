cmake_minimum_required(VERSION 2.8.3)
project(lidar_n301_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  LidarN301Packet.msg
  LidarN301Point.msg
  LidarN301Scan.msg
  LidarN301Sweep.msg
  LidarN301Difop.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
