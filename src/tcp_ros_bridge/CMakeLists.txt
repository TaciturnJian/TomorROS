cmake_minimum_required(VERSION 3.0.2)
project(tcp_ros_bridge)

add_subdirectory(LangYa)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

add_executable(${PROJECT_NAME}_node src/tcp_ros_bridge_node.cpp)
target_include_directories(${PROJECT_NAME}_node BEFORE 
  PUBLIC include
  PUBLIC ${catkin_INCLUDE_DIRS}
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "" CXX_STANDARD 20)
target_link_libraries(${PROJECT_NAME}_node
  PUBLIC ${catkin_LIBRARIES}
  PUBLIC LangYa::Network
  PUBLIC LangYa::Messages
)

