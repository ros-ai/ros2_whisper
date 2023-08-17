cmake_minimum_required(VERSION 3.8)
project(ros2_whisper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_whisper_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(whisper_cpp_vendor REQUIRED)

# ros2_whisper
add_library(ros2_whisper SHARED
  src/audio_buffer.cpp
  src/inference_server.cpp
  src/model_manager.cpp
  src/whisper.cpp
)

set_target_properties(ros2_whisper PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

target_include_directories(ros2_whisper
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ros2_whisper
  rclcpp
  ros2_whisper_msgs
  std_msgs
  std_srvs
  whisper_cpp_vendor  
)

target_link_libraries(ros2_whisper
  whisper_cpp_vendor::whisper
)

ament_export_targets(export_ros2_whisper HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ros2_whisper
  EXPORT export_ros2_whisper
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# nodes


add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node rclcpp std_msgs whisper_cpp_vendor)
target_link_libraries(test_node
  ros2_whisper
)

install(
  TARGETS test_node
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
