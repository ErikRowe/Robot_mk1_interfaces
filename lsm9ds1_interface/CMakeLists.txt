cmake_minimum_required(VERSION 3.5)
project(lsm9ds1_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include include/lsm9ds1_interface)

# Add executable
add_executable(lsm9ds1_interface
  src/lsm9ds1_interface.cpp
  src/lsm9ds1_driver.cpp)

# Link against libraries
ament_target_dependencies(lsm9ds1_interface rclcpp sensor_msgs)

# Install executables
install(TARGETS
  lsm9ds1_interface
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()