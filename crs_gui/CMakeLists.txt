cmake_minimum_required(VERSION 3.5.0)
project(crs_gui VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(crs_area_selection REQUIRED path)

find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
  Sql
)

###########
## Build ##
###########

set(crs_gui_UIS
  ui/polygon_area_selection_widget.ui
)

set(crs_gui_widget_HEADERS
  include/${PROJECT_NAME}/widgets/polygon_area_selection_widget.h
)

set(crs_gui_widget_SOURCES
  src/widgets/polygon_area_selection_widget.cpp
)

# Run the MOC on the Qt-related UIs and headers
qt5_wrap_ui(crs_gui_UIS_H
  ${crs_gui_UIS}
)

qt5_wrap_cpp(crs_gui_widget_MOCS
  ${crs_gui_widget_HEADERS}
)

# Qt Widgets
add_library(${PROJECT_NAME}_widgets
  ${crs_gui_UIS_H}
  ${crs_gui_widget_MOCS}
  ${crs_gui_widget_SOURCES}
)
set_target_properties(${PROJECT_NAME}_widgets PROPERTIES CXX_STANDARD 14)
target_include_directories(${PROJECT_NAME}_widgets PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
# This must be added because qt5_wrap_ui creates a header in the root of the pkg build dir
target_include_directories(${PROJECT_NAME}_widgets PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(${PROJECT_NAME}_widgets
  Qt5::Widgets
  crs_area_selection::crs_area_selection_area_selection
)
ament_target_dependencies(${PROJECT_NAME}_widgets crs_area_selection rclcpp shape_msgs std_srvs)

######################
## Demo Executables ##
######################

# Area selection widget demp
add_executable(${PROJECT_NAME}_area_selection_demo src/nodes/area_selection_widget_demo.cpp)
target_link_libraries(${PROJECT_NAME}_area_selection_demo ${PROJECT_NAME}_widgets)
set_target_properties(${PROJECT_NAME}_area_selection_demo PROPERTIES CXX_STANDARD 14)
target_include_directories(${PROJECT_NAME}_area_selection_demo PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")



#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}_widgets
    ${PROJECT_NAME}_area_selection_demo
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

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY ui/
  DESTINATION share/${PROJECT_NAME}/ui
)


#############
## Ament ##
#############
ament_export_dependencies(
    ament_cmake
    crs_area_selection
    rclcpp
    shape_msgs
    std_srvs)

ament_package()
