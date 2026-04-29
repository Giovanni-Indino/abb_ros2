cmake_minimum_required(VERSION 3.8)
project(smart_gripper_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(abb_librws REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_executable(smart_gripper_node
  src/smart_gripper_control.cpp
  src/node.cpp
)

ament_target_dependencies(smart_gripper_node
  rclcpp
  abb_librws
  std_msgs
  std_srvs
)

target_link_libraries(smart_gripper_node
  rclcpp::rclcpp
)

install(TARGETS
  smart_gripper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()