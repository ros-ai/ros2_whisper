cmake_minimum_required(VERSION 3.8)
project(whisper_cpp_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)

find_package(ament_cmake REQUIRED)

set(WHISPER_VERSION_MAJOR 1 CACHE STRING "Major whisper.cpp version.")
set(WHISPER_VERSION_MINOR 7 CACHE STRING "Minor whisper.cpp version.")
set(WHISPER_VERSION_PATCH 2 CACHE STRING "Patch whisper.cpp version.")

FetchContent_Declare(
  whisper
  GIT_REPOSITORY https://github.com/ggerganov/whisper.cpp.git
  GIT_TAG        v${WHISPER_VERSION_MAJOR}.${WHISPER_VERSION_MINOR}.${WHISPER_VERSION_PATCH}
)

FetchContent_MakeAvailable(whisper)

#######################################################################
# note that target properties need change as whisper.cpp CMake is buggy
#######################################################################
set_target_properties(
  whisper PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES
    $<INSTALL_INTERFACE:.>
  CXX_STANDARD
    11
)

# install ggml header
install(
  FILES ${whisper_SOURCE_DIR}/ggml/include/ggml.h
  DESTINATION include
)
##############
# end of fixes
##############
# Export header files for downstream packages
install(
  DIRECTORY
    ${whisper_SOURCE_DIR}/include/
    ${whisper_SOURCE_DIR}/ggml/include/
  DESTINATION include
)

install(
  TARGETS ggml
  EXPORT export_whisper
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_whisper HAS_LIBRARY_TARGET)

# Install the library
install(
  TARGETS whisper
  EXPORT export_whisper
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
  PUBLIC_HEADER DESTINATION include
)

ament_package()
