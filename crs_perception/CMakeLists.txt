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
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(crs_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)

include_directories(include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
  src/model_to_point_cloud.cpp
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
  

add_library(${PROJECT_NAME}_nodes SHARED
  src/nodes/localize_to_part.cpp)
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes "crc_perception::LocalizeToPart")
target_link_libraries(${PROJECT_NAME}_nodes ${PROJECT_NAME} ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_nodes
  crs_msgs
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
