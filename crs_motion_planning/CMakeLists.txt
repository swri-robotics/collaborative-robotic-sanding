cmake_minimum_required(VERSION 3.5)
project(crs_motion_planning)

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
find_package(ament_index_cpp REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(crs_msgs REQUIRED)

find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_rosutils REQUIRED)

find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)

add_executable(${PROJECT_NAME}_freespace_planning_server src/freespace_planning_server.cpp)

target_include_directories(${PROJECT_NAME}_freespace_planning_server PUBLIC
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${crs_msgs_INCLUDE_DIRS}
  ${tesseract_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  )
target_link_libraries(${PROJECT_NAME}_freespace_planning_server
  ${rclcpp_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
  ${crs_msgs_LIBRARIES}
  ${tesseract_msgs_LIBRARIES}
  ${trajectory_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_eigen_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  tesseract::tesseract
  tesseract::tesseract_motion_planners_trajopt
  tesseract::tesseract_rosutils
  )

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(TARGETS ${PROJECT_NAME}_freespace_planning_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
