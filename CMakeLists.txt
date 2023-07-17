cmake_minimum_required(VERSION 3.5)
project(map_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_localization src/map_localization.cpp)
target_link_libraries(map_localization ${catkin_LIBRARIES})

install(TARGETS map_localization
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)