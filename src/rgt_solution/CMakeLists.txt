cmake_minimum_required(VERSION 3.10)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()
project(rgt_solution)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include 
  rgt_solution/include/CircularBuffer
  rgt_solution/include/ParallelProcessor
)

set(EXECUTABLES
  main_2
  main_3
)

foreach(target ${EXECUTABLES})
  if (${target} STREQUAL "main_2")
    add_executable(${target} src/${target}.cpp) 
    ament_target_dependencies(${target} rclcpp std_msgs)
  else()
    add_executable(${target} src/${target}.cpp) 
    ament_target_dependencies(${target} rclcpp sensor_msgs)
  endif()

  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()


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
