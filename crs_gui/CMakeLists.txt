cmake_minimum_required(VERSION 3.5.0)
project(crs_gui VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rviz2 REQUIRED)

find_package(crs_area_selection REQUIRED path)
find_package(crs_msgs REQUIRED)
find_package(crs_motion_planning REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

set(crs_gui_UIS
  ui/crs_application.ui
  ui/part_selection.ui
  ui/polygon_area_selection_widget.ui
  ui/state_machine_interface.ui
)

set(crs_gui_widget_HEADERS
  include/${PROJECT_NAME}/widgets/crs_application_widget.h
  include/${PROJECT_NAME}/widgets/part_selection_widget.h
  include/${PROJECT_NAME}/widgets/polygon_area_selection_widget.h
  include/${PROJECT_NAME}/widgets/state_machine_interface_widget.h
)

set(crs_gui_widget_SOURCES
  src/widgets/crs_application_widget.cpp
  src/widgets/part_selection_widget.cpp
  src/widgets/polygon_area_selection_widget.cpp
  src/widgets/state_machine_interface_widget.cpp
)

set(crs_gui_panel_SOURCES
  src/application_panel.cpp
)

# Run the MOC on the Qt-related UIs and headers
qt5_wrap_ui(crs_gui_UIS_H
  ${crs_gui_UIS}
)

qt5_wrap_cpp(crs_gui_widget_MOCS
  ${crs_gui_widget_HEADERS}
)

qt5_wrap_cpp(crs_gui_panel_MOCS
  include/${PROJECT_NAME}/application_panel.h
)

# Qt Widgets
add_library(${PROJECT_NAME}_widgets SHARED
  ${crs_gui_UIS_H}
  ${crs_gui_widget_MOCS}
  ${crs_gui_widget_SOURCES}
)
set_target_properties(${PROJECT_NAME}_widgets PROPERTIES CXX_STANDARD 17)
target_include_directories(${PROJECT_NAME}_widgets PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
# This must be added because qt5_wrap_ui creates a header in the root of the pkg build dir
target_include_directories(${PROJECT_NAME}_widgets PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(${PROJECT_NAME}_widgets PUBLIC ${crs_area_selection_INCLUDE_DIRS})
  target_link_libraries (${PROJECT_NAME}_widgets ${CMAKE_CURRENT_BINARY_DIR})
  foreach(DEF ${crs_area_selection_DEFINITIONS})
    string(STRIP ${DEF} DEF)
    if (NOT "${DEF}" STREQUAL "")
       string(SUBSTRING "${DEF}" 0 2 DEF_PREFIX)
       if ("${DEF_PREFIX}" STREQUAL "-m")
           string(REPLACE " " ";" DEF ${DEF})
           foreach(OPTION_DEF ${DEF})
               target_compile_options(${PROJECT_NAME}_widgets PUBLIC ${OPTION_DEF})
           endforeach()
       else()
          target_compile_definitions(${PROJECT_NAME}_widgets PUBLIC ${DEF})
       endif()
    endif()
  endforeach()
target_link_libraries(${PROJECT_NAME}_widgets
  Qt5::Widgets
  crs_area_selection::crs_area_selection_area_selection
  crs_motion_planning::crs_motion_planning_path_processing_utils
)

# Rviz Panel
add_library(${PROJECT_NAME}_panels SHARED
  ${crs_gui_panel_MOCS}
  ${crs_gui_panel_SOURCES}
)
target_link_libraries(${PROJECT_NAME}_panels
  ${PROJECT_NAME}_widgets
)


ament_target_dependencies(${PROJECT_NAME}_widgets rclcpp shape_msgs std_srvs crs_msgs)

######################
## Demo Executables ##
######################

# Area selection widget demo
add_executable(${PROJECT_NAME}_area_selection_demo src/nodes/area_selection_widget_demo.cpp)
target_link_libraries(${PROJECT_NAME}_area_selection_demo ${PROJECT_NAME}_widgets)
set_target_properties(${PROJECT_NAME}_area_selection_demo PROPERTIES CXX_STANDARD 17)
target_include_directories(${PROJECT_NAME}_area_selection_demo PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

add_executable(${PROJECT_NAME}_part_selection_demo src/nodes/part_selection_widget_demo.cpp)
target_link_libraries(${PROJECT_NAME}_part_selection_demo ${PROJECT_NAME}_widgets)
set_target_properties(${PROJECT_NAME}_part_selection_demo PROPERTIES CXX_STANDARD 17)
target_include_directories(${PROJECT_NAME}_part_selection_demo PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

add_executable(${PROJECT_NAME}_crs_application_demo src/nodes/crs_application_demo.cpp)
target_link_libraries(${PROJECT_NAME}_crs_application_demo ${PROJECT_NAME}_widgets)
set_target_properties(${PROJECT_NAME}_crs_application_demo PROPERTIES CXX_STANDARD 17)
target_include_directories(${PROJECT_NAME}_crs_application_demo PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

add_executable(${PROJECT_NAME}_state_machine_interface_demo src/nodes/state_machine_interface_widget_demo.cpp)
target_link_libraries(${PROJECT_NAME}_state_machine_interface_demo ${PROJECT_NAME}_widgets)
set_target_properties(${PROJECT_NAME}_state_machine_interface_demo PROPERTIES CXX_STANDARD 17)
target_include_directories(${PROJECT_NAME}_state_machine_interface_demo PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")


#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}_panels
    ${PROJECT_NAME}_widgets
    ${PROJECT_NAME}_area_selection_demo
    ${PROJECT_NAME}_part_selection_demo
    ${PROJECT_NAME}_crs_application_demo
    ${PROJECT_NAME}_state_machine_interface_demo
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

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
  PATTERN ".svn" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY ui/
  DESTINATION share/${PROJECT_NAME}/ui
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

#############
## Ament ##
#############
ament_export_dependencies(
    ament_cmake
    crs_area_selection
    pluginlib
    rclcpp
    shape_msgs
    std_srvs)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()
