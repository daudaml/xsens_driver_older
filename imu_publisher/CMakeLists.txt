cmake_minimum_required(VERSION 3.5)
project(imu_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# Add xsens source files
set(XSENS_SRC
  src/cmt1.cpp
  src/cmt2.cpp
  src/cmt3.cpp
  src/cmtmessage.cpp
  src/cmtpacket.cpp
  src/cmtscan.cpp
  src/xsens_std.cpp
  src/xsens_time.cpp
  src/xsens_exception.cpp
)

# Add the xsens sources to the executable
add_executable(imu_publisher_node src/xsens_driver.cpp ${XSENS_SRC})

# Link necessary libraries
ament_target_dependencies(imu_publisher_node rclcpp sensor_msgs tf2 tf2_geometry_msgs)
target_link_libraries(imu_publisher_node ncurses pthread)

# Install the executable
install(TARGETS
  imu_publisher_node
  DESTINATION lib/${PROJECT_NAME})

# Ament package configuration
ament_package()

