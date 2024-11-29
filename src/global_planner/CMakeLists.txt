cmake_minimum_required(VERSION 3.0.2)
project(global_planner)

# add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  angles
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  tf2_geometry_msgs
  tf2_ros
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES global_planner
 CATKIN_DEPENDS roscpp rospy
)



###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# file(GLOB path_planner_src_files src/*.cpp)
file(GLOB graph_path_planner_src_files src/graph_planner/*.cc)
# file(GLOB sample_path_planner_src_files src/sample_planner/*.cpp)
# file(GLOB evolutionary_path_planner_src_files src/evolutionary_planner/*.cpp)

add_library(${PROJECT_NAME}
#   # ${path_planner_src_files}
  ${graph_path_planner_src_files}
#   # ${sample_path_planner_src_files}
#   # ${evolutionary_path_planner_src_files}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
