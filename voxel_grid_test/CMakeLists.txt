cmake_minimum_required(VERSION 2.8.3)
project(voxel_grid_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  block_map
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES voxel_grid_test
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(voxel_grid_test
    src/voxel_grid_test_node.cpp)

target_link_libraries(voxel_grid_test
${catkin_LIBRARIES}
)
