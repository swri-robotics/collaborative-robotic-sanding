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
find_package(ur_ikfast_kinematics REQUIRED)
find_package(descartes_light REQUIRED)
find_package(descartes_samplers REQUIRED)
find_package(descartes_ikfast REQUIRED)

find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)

find_package(iterative_spline_parameterization REQUIRED)

add_library(${PROJECT_NAME}_path_processing_utils SHARED src/path_processing_utils.cpp)
target_include_directories(${PROJECT_NAME}_path_processing_utils PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
  ${crs_msgs_INCLUDE_DIRS}
  )
target_link_libraries(${PROJECT_NAME}_path_processing_utils
  ${YAML_CPP_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${trajectory_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_eigen_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${control_msgs_LIBRARIES}
  ${crs_msgs_LIBRARIES}
  tesseract::tesseract_common
  Eigen3::Eigen
  iterative_spline_parameterization::iterative_spline_parameterization
  )

add_library(${PROJECT_NAME}_path_planning_utils SHARED src/path_planning_utils.cpp)
target_include_directories(${PROJECT_NAME}_path_planning_utils PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${geometry_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${tesseract_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
  )
target_link_libraries(${PROJECT_NAME}_path_planning_utils
  ${PROJECT_NAME}_path_processing_utils
  ${geometry_msgs_LIBRARIES}
  ${trajectory_msgs_LIBRARIES}
  ${tesseract_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_eigen_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${control_msgs_LIBRARIES}
  ur_ikfast_kinematics::ur10e_ikfast
  tesseract::tesseract
  tesseract::tesseract_motion_planners_trajopt
  tesseract::tesseract_motion_planners_ompl
  tesseract::tesseract_motion_planners_descartes
  tesseract::tesseract_rosutils
  Eigen3::Eigen
  descartes::descartes_light
  descartes::descartes_light_core
  descartes::descartes_samplers
  descartes::descartes_ikfast
  )

add_executable(${PROJECT_NAME}_motion_planning_server src/motion_planning_server.cpp)
target_include_directories(${PROJECT_NAME}_motion_planning_server PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_motion_planning_server PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${crs_msgs_INCLUDE_DIRS}
  ${tesseract_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
  )
target_link_libraries(${PROJECT_NAME}_motion_planning_server
  ${PROJECT_NAME}_path_planning_utils
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
  ${control_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  )

add_executable(${PROJECT_NAME}_process_planner_test src/process_planner_test.cpp)
target_include_directories(${PROJECT_NAME}_process_planner_test PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_process_planner_test PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${crs_msgs_INCLUDE_DIRS}
  ${tesseract_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
  )
target_link_libraries(${PROJECT_NAME}_process_planner_test
  ${PROJECT_NAME}_path_planning_utils
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
  ${control_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
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

ament_export_include_directories(
    include)


set(ROS_LIB_DESTINATION lib)
set(ROS_BIN_DESTINATION bin)
set(ROS_INCLUDE_DESTINATION include)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h*"
  PATTERN ".svn" EXCLUDE)
install(TARGETS
  ${PROJECT_NAME}_path_processing_utils
  ${PROJECT_NAME}_path_planning_utils
  ${PROJECT_NAME}_motion_planning_server
  ${PROJECT_NAME}_process_planner_test
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
