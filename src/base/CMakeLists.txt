cmake_minimum_required(VERSION 3.8)
project(ros2_swervebot_sim_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  rviz2
  robot_desc
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()