cmake_minimum_required(VERSION 2.8.3)
project(nautilus_udp_bridge)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

find_package(Poco REQUIRED COMPONENTS
  Foundation Net
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nautilus_bridge
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include/${PROJECT_NAME}/
  ${catkin_INCLUDE_DIRS}
)

add_executable(odometry_stub_node src/odometry_stub.cpp)
add_executable(bridge_node src/bridge.cpp)

target_link_libraries(odometry_stub_node
   ${catkin_LIBRARIES}
)

target_link_libraries(bridge_node
   ${catkin_LIBRARIES}
   Poco::Foundation
   Poco::Net
   pthread
)

add_executable(dump_client src/dumb_client)

target_link_libraries(dump_client
   Poco::Net
)