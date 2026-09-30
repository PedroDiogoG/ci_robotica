cmake_minimum_required(VERSION 3.8)
project(pose_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_toolbox REQUIRED)

# Nó de controle de pose (Exercício 1.1)
add_executable(manager src/manager.cpp)
target_include_directories(manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(manager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(manager
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  control_toolbox
)

# Nó de seguimento de missão (Exercício 1.2)
add_executable(mission_follower src/mission_follower.cpp)
target_include_directories(mission_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mission_follower PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(mission_follower
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS manager mission_follower
  DESTINATION lib/${PROJECT_NAME})

# Launch e YAMLs de parâmetros
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})

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
