cmake_minimum_required(VERSION 3.5)
project(specialized_intra_process_demo)

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
find_package(std_msgs REQUIRED)
find_package(specialized_intra_process REQUIRED)
find_package(tlsf_cpp REQUIRED)


add_executable(cyclic_pipeline
  src/cyclic_pipeline.cpp
)

ament_target_dependencies(cyclic_pipeline
  "rclcpp"
  "std_msgs"
  "specialized_intra_process"
)

install(TARGETS
  cyclic_pipeline
  DESTINATION lib/${PROJECT_NAME})

add_executable(talker_listener
  src/talker_listener.cpp
  )
ament_target_dependencies(talker_listener
  "rclcpp"
  "std_msgs"
  "specialized_intra_process"
  )
install(TARGETS
  talker_listener
  DESTINATION lib/${PROJECT_NAME})

add_executable(talker_listener_with_allocator
  src/talker_listener_with_allocator.cpp
  )
ament_target_dependencies(talker_listener_with_allocator
  "rclcpp"
  "std_msgs"
  "specialized_intra_process"
  "tlsf_cpp"
  )
install(TARGETS
  talker_listener_with_allocator
  DESTINATION lib/${PROJECT_NAME})

add_executable(debug_interface
  src/debug_interface.cpp
  )
ament_target_dependencies(debug_interface
  "rclcpp"
  "std_msgs"
  "specialized_intra_process"
  "tlsf_cpp"
  )
install(TARGETS
  debug_interface
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
