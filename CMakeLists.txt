cmake_minimum_required(VERSION 3.0.2)
project(pcl_graph_cut)

#add_compile_options(-std=c++11)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
 pcl_conversions
 pcl_msgs
 pcl_ros
 roscpp
 rospy
 sensor_msgs
 dynamic_reconfigure
)

find_package(PCL 1.8 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/GraphCutRos.cfg
)

catkin_package(
)

include_directories(
 include
 include/maxflow
 include/maxflow/maxflow
 ${PCL_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
)

add_executable(vccs src/vccs.cpp)
target_link_libraries(vccs
  ${PCL_LIBRARIES}
)

add_executable(graph_cut src/graph_cut.cpp include/maxflow/maxflow.cpp include/maxflow/maxflow/graph.cpp)
target_link_libraries(graph_cut
  ${PCL_LIBRARIES}
)


add_executable(graph_cut_ros src/graph_cut_ros.cpp include/maxflow/maxflow.cpp include/maxflow/maxflow/graph.cpp)
target_link_libraries(graph_cut_ros
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)


