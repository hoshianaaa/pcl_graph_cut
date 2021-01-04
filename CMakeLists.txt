cmake_minimum_required(VERSION 3.0.2)
project(pcl_graph_cut)

find_package(catkin REQUIRED)
find_package(PCL 1.8 REQUIRED)

catkin_package(
)

include_directories(
 include
 include/maxflow
 include/maxflow/maxflow
 ${PCL_INCLUDE_DIRS}
 #${catkin_INCLUDE_DIRS}
)

add_executable(vccs src/vccs.cpp)
target_link_libraries(vccs
  ${PCL_LIBRARIES}
)

add_executable(graph_cut src/graph_cut.cpp include/maxflow/maxflow.cpp include/maxflow/maxflow/graph.cpp)
target_link_libraries(graph_cut
  ${PCL_LIBRARIES}
)

add_executable(detect_changes src/detect_changes.cpp)
target_link_libraries(detect_changes
  ${PCL_LIBRARIES}
)


