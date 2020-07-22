cmake_minimum_required(VERSION 3.5)
project(crs_robot_comms)

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

find_package(crs_msgs REQUIRED)

find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)

add_executable(${PROJECT_NAME}_ur_comms src/ur_comms.cpp)
target_include_directories(${PROJECT_NAME}_ur_comms PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_ur_comms PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${crs_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${ur_dashboard_msgs_INCLUDE_DIRS}
  )
target_link_libraries(${PROJECT_NAME}_ur_comms
  ${rclcpp_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
  ${crs_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${ur_dashboard_msgs_LIBRARIES}
  )

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


set(ROS_LIB_DESTINATION lib)
set(ROS_BIN_DESTINATION bin)
set(ROS_INCLUDE_DESTINATION include)

install(TARGETS
  ${PROJECT_NAME}_ur_comms
  EXPORT ${PACKAGE_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_interfaces(${PACKAGE_NAME}-targets)
ament_export_dependencies(
    ament_cmake
    ament_index_cpp
    rosidl_default_generators
    rclcpp
    crs_msgs
    std_srvs
    std_msgs
    ur_dashboard_msgs)

ament_package()
