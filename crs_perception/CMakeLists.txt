cmake_minimum_required(VERSION 3.5)
project(crs_perception)

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
find_package(crs_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED NO_MODULE)

include_directories(include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/model_to_point_cloud.cpp
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES})
  
# node as component plugin
add_library(${PROJECT_NAME}_nodes SHARED
  src/nodes/localize_to_part.cpp)
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "crs_perception::LocalizeToPart")
target_link_libraries(${PROJECT_NAME}_nodes ${PROJECT_NAME} ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_nodes rclcpp rclcpp_components crs_msgs tf2_ros visualization_msgs)

# node as executable, component plugin can't load parameters
add_executable(localize_to_part src/nodes/localize_to_part.cpp src/model_to_point_cloud.cpp)
target_include_directories(localize_to_part PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(localize_to_part SYSTEM PUBLIC)
target_link_libraries(localize_to_part ${PCL_LIBRARIES})
ament_target_dependencies(localize_to_part rclcpp rclcpp_components crs_msgs tf2_eigen tf2_ros std_msgs visualization_msgs)

# test executable
add_executable(test_localize_to_part src/nodes/test_localize_to_part.cpp)
target_link_libraries(test_localize_to_part ${PROJECT_NAME} ${PCL_LIBRARIES})
ament_target_dependencies(test_localize_to_part rclcpp crs_msgs std_msgs)

### Install
install(TARGETS localize_to_part test_localize_to_part
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_nodes
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
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

ament_export_dependencies(ament_cmake)
ament_package()
