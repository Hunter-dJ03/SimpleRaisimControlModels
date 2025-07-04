cmake_minimum_required(VERSION 3.8)
project(double_pendulum_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

# Standard ROS2 stuffs
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Raimsim stuffs
set(CMAKE_PREFIX_PATH "$ENV{HOME}/Documents/github/importedRepos/raisimLib/raisim/linux")
find_package(Eigen3 REQUIRED)
find_package(PythonLibs 3.10 REQUIRED)  # Change Python version as needed
include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
)
find_package(raisim REQUIRED)


add_executable(raisim_bridge src/raisim_bridge.cpp)
ament_target_dependencies(raisim_bridge rclcpp sensor_msgs std_msgs Eigen3 raisim PythonLibs)

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

# Install
install(TARGETS
  raisim_bridge
  DESTINATION lib/${PROJECT_NAME}
) 

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
