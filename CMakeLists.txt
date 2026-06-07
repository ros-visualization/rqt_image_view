cmake_minimum_required(VERSION 3.20)

project(rqt_image_view)

find_package(ament_cmake REQUIRED)

if(WIN32)
  message(STATUS "rqt_image_view is not yet supported on Windows. Package will not be built.")
  ament_package()
  return()
endif()

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)

find_package(qt_gui_cpp REQUIRED)
set(QT_VERSION_MAJOR "${qt_gui_cpp_USE_QT_MAJOR_VERSION}")
find_package(Qt${qt_gui_cpp_USE_QT_MAJOR_VERSION} REQUIRED COMPONENTS Widgets Core)

find_package(rqt_gui_cpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_cmake_python REQUIRED)

message("Using Qt ${QT_VERSION_MAJOR}")

set(rqt_image_view_SRCS
  src/rqt_image_view/detail/framerate_estimator.cpp
  src/rqt_image_view/detail/pixel_format.cpp
  src/rqt_image_view/detail/pixel_mapping.cpp
  src/rqt_image_view/image_view.cpp
  src/rqt_image_view/ratio_layouted_frame.cpp
)

set(rqt_image_view_HDRS
  include/rqt_image_view/image_view.hpp
  include/rqt_image_view/ratio_layouted_frame.hpp
)

set(rqt_image_view_UIS
  src/rqt_image_view/image_view.ui
)

if(${QT_VERSION_MAJOR} GREATER "5")
  qt_wrap_cpp(rqt_image_view_MOCS ${rqt_image_view_HDRS})
  qt_wrap_ui(rqt_image_view_UIS_H ${rqt_image_view_UIS})
else()
  qt5_wrap_cpp(rqt_image_view_MOCS ${rqt_image_view_HDRS})
  qt5_wrap_ui(rqt_image_view_UIS_H ${rqt_image_view_UIS})
endif()

add_library(${PROJECT_NAME} SHARED
  ${rqt_image_view_SRCS}
  ${rqt_image_view_MOCS}
  ${rqt_image_view_UIS_H}
)
set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOMOC 1)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${rclcpp_TARGETS}
  ${qt_gui_cpp_TARGETS}
  ${rqt_gui_cpp_TARGETS}
  image_transport::image_transport
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  Qt${QT_VERSION_MAJOR}::Widgets
PRIVATE
  Qt${QT_VERSION_MAJOR}::Core
  cv_bridge::cv_bridge
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME})

install(PROGRAMS scripts/rqt_image_view
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/image_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  PATTERN "detail" EXCLUDE
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export new-style CMake variables
ament_export_targets(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package()
