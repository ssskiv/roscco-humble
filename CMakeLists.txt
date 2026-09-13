cmake_minimum_required(VERSION 3.8)
project(roscco)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_EXAMPLES "Build the teleop example node" ON)

# ---------------------------------------------------------------------------
# OSCC API (git submodule at ./oscc)
# ---------------------------------------------------------------------------
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/oscc/api/src/oscc.c")
  message(FATAL_ERROR
    "OSCC submodule not found. Run:\n"
    "  git submodule update --init oscc\n"
    "from the package root.")
endif()

# Sets the vehicle-specific compile definitions from -DKIA_SOUL=ON etc.
include(oscc/api/OsccConfig.cmake)

add_library(oscc_api STATIC oscc/api/src/oscc.c)
set_target_properties(oscc_api PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(oscc_api PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/oscc/api/include>
  $<INSTALL_INTERFACE:include>)

# ---------------------------------------------------------------------------
# ROS 2 dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/BrakeCommand.msg
  msg/BrakeReport.msg
  msg/BrakeReportData.msg
  msg/CanFrame.msg
  msg/CanFrameData.msg
  msg/EnableDisable.msg
  msg/FaultOriginId.msg
  msg/FaultReport.msg
  msg/FaultReportData.msg
  msg/SteeringCommand.msg
  msg/SteeringReport.msg
  msg/SteeringReportData.msg
  msg/ThrottleCommand.msg
  msg/ThrottleReport.msg
  msg/ThrottleReportData.msg
  DEPENDENCIES std_msgs
)

# Link generated C++ types back into targets in this same package.
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---------------------------------------------------------------------------
# Bridge libraries
# ---------------------------------------------------------------------------
add_library(roscco_bridge SHARED
  src/ros_to_oscc.cpp
  src/oscc_to_ros.cpp
)
target_include_directories(roscco_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(roscco_bridge rclcpp std_msgs)
target_link_libraries(roscco_bridge oscc_api "${cpp_typesupport_target}")

# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
add_executable(roscco_node src/roscco_node.cpp)
ament_target_dependencies(roscco_node rclcpp)
target_link_libraries(roscco_node roscco_bridge oscc_api "${cpp_typesupport_target}")

install(TARGETS roscco_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_EXAMPLES)
  add_executable(roscco_teleop example/roscco_teleop.cpp)
  ament_target_dependencies(roscco_teleop rclcpp sensor_msgs)
  target_link_libraries(roscco_teleop "${cpp_typesupport_target}")
  install(TARGETS roscco_teleop
    DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS roscco_bridge oscc_api
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY oscc/api/include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h")

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(roscco_bridge oscc_api)
ament_export_dependencies(rclcpp std_msgs rosidl_default_runtime)
ament_package()
