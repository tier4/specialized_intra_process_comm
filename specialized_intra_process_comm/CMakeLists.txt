cmake_minimum_required(VERSION 3.5)
project(specialized_intra_process_comm)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(notification_msgs REQUIRED)

include_directories(include)

add_library(specialized_intra_process_comm SHARED
  src/intra_process_manager.cpp
  src/publisher_base.cpp
  src/subscription_base.cpp
)

ament_target_dependencies(specialized_intra_process_comm
  "rclcpp"
  "notification_msgs"
)

target_include_directories(specialized_intra_process_comm
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_export_targets(export_specialized_intra_process_comm HAS_LIBRARY_TARGET)
ament_export_dependencies(
  "rclcpp"
  "notification_msgs"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS specialized_intra_process_comm
  EXPORT export_specialized_intra_process_comm
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(specialized_intra_process_comm)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
