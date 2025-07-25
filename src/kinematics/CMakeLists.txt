cmake_minimum_required(VERSION 3.8)
project(ros2_swervebot_sim_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_swervebot_sim_msgs REQUIRED)

add_executable(kinematics_worker src/kinematics_worker.cpp)
ament_target_dependencies(kinematics_worker
    rclcpp
    geometry_msgs
    std_msgs
    ros2_swervebot_sim_msgs
)

add_executable(differential_converter src/differential_converter.cpp)
ament_target_dependencies(differential_converter
    rclcpp
    std_msgs
    ros2_swervebot_sim_msgs
)

set(COMMON_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(kinematics_worker PUBLIC ${COMMON_INCLUDE_DIRS})
target_include_directories(differential_converter PUBLIC ${COMMON_INCLUDE_DIRS})


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

install(TARGETS
    kinematics_worker
    differential_converter
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
