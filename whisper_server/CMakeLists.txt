cmake_minimum_required(VERSION 3.8)
project(whisper_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)

set(WHISPER_NODES_DEPENDENCIES
  builtin_interfaces
  rcl_interfaces
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  whisper_idl
  whisper_util
)

foreach(DEPENDENCY IN LISTS WHISPER_NODES_DEPENDENCIES)
  find_package(${DEPENDENCY} REQUIRED)  
endforeach()

# whisper nodes
add_library(${PROJECT_NAME} SHARED
  src/inference_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${WHISPER_NODES_DEPENDENCIES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${WHISPER_NODES_DEPENDENCIES}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# whisper component
add_library(inference_component SHARED
  src/inference_component.cpp
)

ament_target_dependencies(inference_component
  rclcpp
  rclcpp_components
)

target_link_libraries(inference_component
  ${PROJECT_NAME}
)

rclcpp_components_register_node(inference_component
  PLUGIN whisper::InferenceComponent
  EXECUTABLE whisper
)

# install components
install(
  TARGETS inference_component
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install configurations
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
