cmake_minimum_required(VERSION 3.5)
project(crs_gazebo_plugins)

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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(control_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)

include_directories(include
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${gazebo_plugins_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
)

link_directories(${gazebo_dev_LIBRARY_DIRS})

# oint_pose_trajectory plugin
add_library(${PROJECT_NAME}_joint_pose_trajectory SHARED
  src/joint_pose_trajectory.cpp
)
ament_target_dependencies(${PROJECT_NAME}_joint_pose_trajectory
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "control_msgs"
  "rclcpp_action"
  "rclcpp_components"
)
ament_export_libraries(${PROJECT_NAME}_joint_pose_trajectory)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    ${PROJECT_NAME}_joint_pose_trajectory
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

