cmake_minimum_required(VERSION 3.8)
project(transcript_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
set(TRANSCRIPT_MANAGER_DEPENDENCIES
  builtin_interfaces
  rcl_interfaces
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  whisper_idl
  whisper_util
)

foreach(DEPENDENCY IN LISTS TRANSCRIPT_MANAGER_DEPENDENCIES)
  find_package(${DEPENDENCY} REQUIRED)  
endforeach()

# transcript_manager component
add_library(transcript_manager_component SHARED
  src/transcript_manager.cpp
  src/transcript.cpp
  src/transcript_operations.cpp
  src/transcript_algorithms.cpp
)
# Add the include directory
target_include_directories(transcript_manager_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Component dependencies
ament_target_dependencies(transcript_manager_component
  rclcpp
  rclcpp_components
  ${TRANSCRIPT_MANAGER_DEPENDENCIES}
)
rclcpp_components_register_node(transcript_manager_component
  PLUGIN whisper::TranscriptManager
  EXECUTABLE transcript_manager_comp
)

# install components
install(
  TARGETS transcript_manager_component
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_package()
