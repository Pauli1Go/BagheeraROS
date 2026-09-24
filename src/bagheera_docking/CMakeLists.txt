cmake_minimum_required(VERSION 3.16)
project(bagheera_docking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mowgli_interfaces REQUIRED)
find_package(nav2_util REQUIRED)
find_package(opennav_docking_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(tag_charging_dock SHARED src/tag_charging_dock.cpp)
target_include_directories(tag_charging_dock PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(tag_charging_dock PUBLIC
  angles::angles
  ${geometry_msgs_TARGETS}
  ${mowgli_interfaces_TARGETS}
  nav2_util::nav2_util_core
  opennav_docking_core::opennav_docking_core
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
)

pluginlib_export_plugin_description_file(opennav_docking_core plugins.xml)

install(TARGETS tag_charging_dock
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()
