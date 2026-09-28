cmake_minimum_required(VERSION 3.16)
project(bagheera_sensors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# One library with all three nodes: they run together in one component
# container (manual_control.launch.py) instead of three Python processes.
add_library(bagheera_sensors_components SHARED
  src/pmw3901.cpp
  src/wt901_i2c.cpp
  src/optical_flow_node.cpp
  src/wt901_node.cpp
  src/measurement_normalizer_node.cpp
)
target_include_directories(bagheera_sensors_components PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(bagheera_sensors_components PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

# Also generates stand-alone executables with the same names as the former
# Python nodes (bagheera_optical_flow, bagheera_wt901,
# bagheera_measurement_normalizer).
rclcpp_components_register_node(bagheera_sensors_components
  PLUGIN "bagheera_sensors::OpticalFlowNode"
  EXECUTABLE bagheera_optical_flow)
rclcpp_components_register_node(bagheera_sensors_components
  PLUGIN "bagheera_sensors::Wt901Node"
  EXECUTABLE bagheera_wt901)
rclcpp_components_register_node(bagheera_sensors_components
  PLUGIN "bagheera_sensors::MeasurementNormalizerNode"
  EXECUTABLE bagheera_measurement_normalizer)

install(TARGETS bagheera_sensors_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sensor_math test/test_sensor_math.cpp)
  target_include_directories(test_sensor_math PRIVATE include)
endif()

ament_package()
