cmake_minimum_required(VERSION 3.14)
project(autoware_geography_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

# GeographicLib
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES NAMES Geographic)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/height.cpp
  src/projection.cpp
  src/lanelet2_projector.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${GeographicLib_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  file(GLOB_RECURSE test_files test/*.cpp)

  ament_add_ros_isolated_gtest(test_${PROJECT_NAME} ${test_files})

  target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}
  )
endif()

add_executable(project_control_points
  src/project_control_points.cpp
)

ament_target_dependencies(project_control_points
  geographic_msgs
  autoware_map_msgs
)

target_link_libraries(project_control_points
  ${PROJECT_NAME}
)

install(
  TARGETS project_control_points
  DESTINATION lib/${PROJECT_NAME}
)

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
