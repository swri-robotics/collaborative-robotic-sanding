cmake_minimum_required(VERSION 3.5)
project(crs_perception)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(crs_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
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
  

add_library(${PROJECT_NAME}_nodes SHARED
  src/nodes/localize_to_part.cpp)
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "crs_perception::LocalizeToPart")
target_link_libraries(${PROJECT_NAME}_nodes ${PROJECT_NAME} ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_nodes rclcpp rclcpp_components crs_msgs tf2_ros)

add_executable(test_localize_to_part src/nodes/test_localize_to_part.cpp)
target_link_libraries(test_localize_to_part ${PROJECT_NAME} ${PCL_LIBRARIES})
ament_target_dependencies(test_localize_to_part rclcpp crs_msgs std_msgs)
install(TARGETS test_localize_to_part
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
