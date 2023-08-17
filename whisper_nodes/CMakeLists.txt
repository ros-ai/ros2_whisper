cmake_minimum_required(VERSION 3.8)
project(whisper_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(whisper_msgs REQUIRED)
find_package(whisper_wrapper REQUIRED)

# whisper nodes
add_library(whisper_nodes SHARED
  src/audio_buffer_node.cpp
  src/whisper_node.cpp
)

ament_target_dependencies(whisper_nodes
  rclcpp
  std_msgs
  std_srvs
  whisper_msgs
  whisper_wrapper
)

target_include_directories(whisper_nodes
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
  whisper_msgs
  whisper_wrapper
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS whisper_nodes
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# audio buffer component
add_library(audio_buffer_component SHARED
  src/audio_buffer_component.cpp
)

ament_target_dependencies(audio_buffer_component
  rclcpp
  rclcpp_components
)

target_link_libraries(audio_buffer_component
  whisper_nodes
)

rclcpp_components_register_node(audio_buffer_component
  PLUGIN whisper::AudioBufferComponent
  EXECUTABLE audio_buffer
)

# whisper component
add_library(whisper_component SHARED
  src/whisper_component.cpp
)

ament_target_dependencies(whisper_component
  rclcpp
  rclcpp_components
)

target_link_libraries(whisper_component
  whisper_nodes
)

rclcpp_components_register_node(whisper_component
  PLUGIN whisper::WhisperComponent
  EXECUTABLE whisper
)

# install components
install(
  TARGETS audio_buffer_component whisper_component
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install launch mixins
ament_python_install_package(${PROJECT_NAME}_launch_mixin)

# install configurations
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
