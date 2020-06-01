cmake_minimum_required(VERSION 3.5.0)
project(crs_area_selection VERSION 0.1.0 LANGUAGES CXX)

# Find all dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS sample_consensus segmentation surface)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(crs_msgs REQUIRED)

# Targets
# Library containing tools for specifying regions and determining inlying points
add_library(${PROJECT_NAME}_area_selection SHARED src/area_selector.cpp src/selection_artist.cpp)
set_target_properties(${PROJECT_NAME}_area_selection PROPERTIES CXX_STANDARD 14)
target_include_directories(${PROJECT_NAME}_area_selection PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_area_selection PUBLIC ${PCL_INCLUDE_DIRS})
  target_link_libraries (${PROJECT_NAME}_area_selection ${PCL_LIBRARIES})
  foreach(DEF ${PCL_DEFINITIONS})
    string(STRIP ${DEF} DEF)
    if (NOT "${DEF}" STREQUAL "")
       string(SUBSTRING "${DEF}" 0 2 DEF_PREFIX)
       if ("${DEF_PREFIX}" STREQUAL "-m")
           string(REPLACE " " ";" DEF ${DEF})
           foreach(OPTION_DEF ${DEF})
               target_compile_options(${PROJECT_NAME}_area_selection PUBLIC ${OPTION_DEF})
           endforeach()
       else()
          target_compile_definitions(${PROJECT_NAME}_area_selection PUBLIC ${DEF})
       endif()
    endif()
  endforeach()
target_include_directories(${PROJECT_NAME}_area_selection PUBLIC
  ${crs_msgs_INCLUDE_DIRS}
  )
target_link_libraries(${PROJECT_NAME}_area_selection
  ${crs_msgs_LIBRARIES}
  )
ament_target_dependencies(${PROJECT_NAME}_area_selection
  ament_index_cpp
  rclcpp
  crs_msgs
  geometry_msgs sensor_msgs shape_msgs std_msgs std_srvs visualization_msgs tf2_ros tf2_eigen)



# Node that runs area selection service
add_executable(${PROJECT_NAME}_area_selection_node src/area_selection_node.cpp)
target_link_libraries(${PROJECT_NAME}_area_selection_node ${PROJECT_NAME}_area_selection)
set_target_properties(${PROJECT_NAME}_area_selection_node PROPERTIES CXX_STANDARD 14)
target_include_directories(${PROJECT_NAME}_area_selection_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

install(
  TARGETS ${PROJECT_NAME}_area_selection  ${PROJECT_NAME}_area_selection_node
  EXPORT ${PACKAGE_NAME}-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_interfaces(${PACKAGE_NAME}-targets)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
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


# Setup Ament
ament_export_dependencies(
    ament_cmake
    ament_index_cpp rclcpp crs_msgs geometry_msgs sensor_msgs shape_msgs std_msgs std_srvs visualization_msgs PCL tf2_ros tf2_eigen)

ament_package()
