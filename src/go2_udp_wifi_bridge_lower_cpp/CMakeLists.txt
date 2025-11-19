cmake_minimum_required(VERSION 3.8)
project(go2_udp_wifi_bridge_lower_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(go2_udp_wifi_bridge_common REQUIRED)

add_executable(lower_bridge_cpp_node src/lower_bridge_node.cpp)

ament_target_dependencies(lower_bridge_cpp_node
  rclcpp
  ament_index_cpp
  go2_udp_wifi_bridge_common)

target_link_libraries(lower_bridge_cpp_node
  go2_udp_wifi_bridge_common::go2_udp_wifi_bridge_common
  ament_index_cpp::ament_index_cpp
)

target_include_directories(lower_bridge_cpp_node PRIVATE include)

install(TARGETS lower_bridge_cpp_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
