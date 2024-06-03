cmake_minimum_required(VERSION 3.8)
project(camera_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include/camera_interface)

add_executable(camera_interface
  src/camera_interface_node.cpp
)

ament_target_dependencies(camera_interface rclcpp sensor_msgs cv_bridge OpenCV image_transport)
target_link_libraries(camera_interface ${GST_LIBRARIES})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# install(
#   DIRECTORY include
#   DESTINATION include/${PROJECT_NAME}
# )

install(
  TARGETS camera_interface
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
