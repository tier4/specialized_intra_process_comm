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
find_package(specialized_intra_process REQUIRED)


add_executable(cyclic_pipeline
  src/cyclic_pipeline.cpp
)

ament_target_dependencies(cyclic_pipeline
  "rclcpp"
  "specialized_intra_process"
)

install(TARGETS
  cyclic_pipeline
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
