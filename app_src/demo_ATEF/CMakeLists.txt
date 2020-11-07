cmake_minimum_required(VERSION 3.5)
project(demo_ATEF)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(demo_float_object REQUIRED)
find_package(ATEF_BaseNode REQUIRED)                         # CHANGE

## Declare a C++ executable
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
add_executable(${PROJECT_NAME}_node ${SOURCES})

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${astc_vmate_base_INCLUDE_DIRS}
  ${demo_float_object_INCLUDE_DIRS}
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}_node ATEF_BaseNode::ATEF_BaseNode demo_float_object::demo_float_object)

install(TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()