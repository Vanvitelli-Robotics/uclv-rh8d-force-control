cmake_minimum_required(VERSION 3.8)
project(repo_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uclv_seed_robotics_ros_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(euler_integrator_controller src/euler_integrator_controller_node.cpp)
ament_target_dependencies(
  euler_integrator_controller
  "rclcpp"
  "uclv_seed_robotics_ros_interfaces" # std_msgs sta quì
  "std_srvs"
)

add_executable(proportional_controller src/proportional_controller_node.cpp)
ament_target_dependencies(
  proportional_controller
  "rclcpp"
  "uclv_seed_robotics_ros_interfaces" # std_msgs sta quì
)

add_executable(gain_node src/gain_node.cpp)
ament_target_dependencies(
  gain_node
  "rclcpp"
  "uclv_seed_robotics_ros_interfaces" # std_msgs sta quì
)

install(TARGETS 
      euler_integrator_controller
      proportional_controller
      gain_node
      DESTINATION 
        lib/${PROJECT_NAME})

install(DIRECTORY
      launch
      DESTINATION
        share/${PROJECT_NAME}/
    )
  

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
