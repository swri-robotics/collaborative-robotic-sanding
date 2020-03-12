cmake_minimum_required(VERSION 3.5)
project(crs_motion_execution)

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

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(crs_msgs REQUIRED)

find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_rosutils REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(${PROJECT_NAME}_motion_execution_server src/motion_execution_server.cpp)
target_include_directories(${PROJECT_NAME}_motion_execution_server PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_motion_execution_server PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${crs_msgs_INCLUDE_DIRS}
  ${tesseract_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
  )
target_link_libraries(${PROJECT_NAME}_motion_execution_server
  ${rclcpp_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
  ${crs_msgs_LIBRARIES}
  ${tesseract_msgs_LIBRARIES}
  ${trajectory_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_eigen_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${control_msgs_LIBRARIES}
  tesseract::tesseract_common
  Eigen3::Eigen
  tesseract::tesseract
  tesseract::tesseract_rosutils
  Eigen3::Eigen
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h*"
  PATTERN ".svn" EXCLUDE)
install(TARGETS
  ${PROJECT_NAME}_motion_execution_server
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
    rclcpp_action
    crs_msgs
    control_msgs
    tesseract_motion_planners
    tesseract_rosutils
    ur_ikfast_kinematics
    descartes_light
    descartes_ikfast
    std_srvs
    std_msgs
    sensor_msgs
    action_msgs
    visualization_msgs
    geometry_msgs
    trajectory_msgs
    tf2
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    yaml-cpp)

  ament_package()
