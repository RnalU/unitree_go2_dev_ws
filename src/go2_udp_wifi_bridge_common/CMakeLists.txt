cmake_minimum_required(VERSION 3.8)
project(go2_udp_wifi_bridge_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(go2_interfaces REQUIRED)

add_library(${PROJECT_NAME}
  src/bridge_config.cpp
  src/packetizer.cpp
  src/udp_bridge_core.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    go2_interfaces
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC rclcpp::rclcpp yaml-cpp
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "GO2_UDP_WIFI_BRIDGE_VERSION=\"0.1.0\""
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp yaml-cpp geometry_msgs nav_msgs sensor_msgs go2_interfaces)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

ament_package()
