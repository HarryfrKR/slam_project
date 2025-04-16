cmake_minimum_required(VERSION 3.5)
project(multi_sensor_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/CMake/")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/karto_sdk/cmake)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/DBoW2/cmake)

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(BUILD_SHARED_LIBS ON)
add_subdirectory(lib/karto_sdk)
add_subdirectory(lib/DBoW2)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(dependencies
  rclcpp
  message_filters
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
  pluginlib
  tf2_geometry_msgs
  tf2_sensor_msgs
  std_msgs
  std_srvs
  builtin_interfaces
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  interactive_markers
  Qt5
  rclcpp_components
  OpenCV
  PCL
  cv_bridge
)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(CHOLMOD REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)
find_package(Boost REQUIRED system serialization filesystem thread)

# Include directories
include_directories(
  include
  lib/karto_sdk/include
  lib/DBoW2
  lib/DBoW2/DBoW2
  ${EIGEN3_INCLUDE_DIRS}
  # ${CHOLMOD_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})
add_definitions(${OpenCV_DEFINITIONS})

# find_library(KARTO_LIB kartoSlamToolbox
#   PATHS ${CMAKE_PREFIX_PATH}/../lib  
#   NO_DEFAULT_PATH
#   REQUIRED
# )

target_include_directories(DBoW2 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
# ORBextractor
add_library(ORBextractor src/ORBextractor.cpp)
target_link_libraries(ORBextractor kartoSlamToolbox ${OpenCV_LIBRARIES})
ament_target_dependencies(ORBextractor rclcpp sensor_msgs std_msgs cv_bridge OpenCV)

add_library(feature_matching src/feature_matching.cpp)
target_link_libraries(feature_matching ORBextractor DBoW2 kartoSlamToolbox ${OpenCV_LIBRARIES})
ament_target_dependencies(feature_matching rclcpp tf2 tf2_ros tf2_geometry_msgs sensor_msgs std_msgs cv_bridge OpenCV)

# # camera_utils
# add_library(camera_utils src/camera_utils.cpp)
# target_link_libraries(camera_utils ORBextractor DBoW2 kartoSlamToolbox ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
# ament_target_dependencies(camera_utils rclcpp tf2 tf2_ros tf2_geometry_msgs sensor_msgs std_msgs cv_bridge OpenCV)

# keyframes
add_library(keyframes src/keyframes.cpp)
target_link_libraries(keyframes ORBextractor DBoW2 kartoSlamToolbox ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
ament_target_dependencies(keyframes rclcpp tf2 tf2_ros tf2_geometry_msgs sensor_msgs std_msgs cv_bridge OpenCV)

# # Main feature extraction node
# add_executable(camera_feature_extractor src/camera_feature_extraction_node.cpp)
# target_link_libraries(camera_feature_extractor ORBextractor kartoSlamToolbox ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
# ament_target_dependencies(camera_feature_extractor rclcpp sensor_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge OpenCV)

# add_library(camera_loop_closure_assistant src/camera_loop_closure_assistant.cpp)
# target_link_libraries(camera_loop_closure_assistant ORBextractor kartoSlamToolbox ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
# ament_target_dependencies(camera_loop_closure_assistant rclcpp tf2_geometry_msgs nav_msgs visualization_msgs std_srvs tf2_ros OpenCV)


install(TARGETS DBoW2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY DBoW2/
  DESTINATION include/DBoW2
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY DUtils/
  DESTINATION include/DUtils
  FILES_MATCHING PATTERN "*.h"
)
install(
  PROGRAMS src/tf_broadcaster_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${libraries}
  kartoSlamToolbox
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ORBextractor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS feature_matching
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS camera_utils
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

install(TARGETS keyframes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${libraries} kartoSlamToolbox)
ament_export_dependencies(${dependencies})
ament_package()
