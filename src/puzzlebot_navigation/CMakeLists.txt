  cmake_minimum_required(VERSION 3.16)

# Package name and enabled languages.
# CUDA is enabled because the MCL SLAM node uses GPU kernels.
project(puzzlebot_navigation LANGUAGES CXX CUDA)

# Use the modern CUDA architecture policy when available.
# This avoids warnings or incorrect default CUDA architecture selection.
if(POLICY CMP0104)
  cmake_policy(SET CMP0104 NEW)
endif()


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Keep CUDA kernels in CUDA C++14 for Jetson Nano compatibility.
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  message(STATUS "Building for Jetson Nano / ARM64")
  set(CMAKE_CUDA_ARCHITECTURES 53)
else()
  message(STATUS "Building for laptop / desktop")
  set(CMAKE_CUDA_ARCHITECTURES 53 75 86 89)
endif()

# Enable useful compiler warnings for GCC and Clang.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
  )
endif()
# -------------------------------------------------------------------
# ROS 2 and external dependencies
# -------------------------------------------------------------------

find_package(ament_cmake REQUIRED)

# Core ROS 2 client library.
find_package(rclcpp REQUIRED)

# ROS 2 message packages used by the MCL SLAM and RRT planner nodes.
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)

# TF2 is required by the planner to read the robot pose from transforms.
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Used to locate files installed inside this package, such as maps.
find_package(ament_index_cpp REQUIRED)

# OpenCV is used for map image loading, obstacle inflation, and SLAM utilities.
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


# CUDA runtime and toolkit support for GPU-accelerated MCL.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  message(STATUS "Using legacy FindCUDA for Jetson")
  find_package(CUDA REQUIRED)

  set(CUDA_INCLUDE_PATHS ${CUDA_INCLUDE_DIRS})
  set(CUDA_LINK_LIBS ${CUDA_LIBRARIES})
else()
  message(STATUS "Using modern CUDAToolkit for laptop/desktop")
  find_package(CUDAToolkit REQUIRED)

  set(CUDA_INCLUDE_PATHS ${CUDAToolkit_INCLUDE_DIRS})
  set(CUDA_LINK_LIBS CUDA::cudart)
endif()

find_package(yaml-cpp REQUIRED)

find_package(puzzlebot_interfaces REQUIRED)



add_executable(path_planner_node
  src/path_planner_node.cpp
)

add_executable(path_planner_node_v2
  src/path_planner_node_v2.cpp
)



add_executable(dynamic_map_node
  src/dynamic_map_node.cpp
  src/occupancy_grid_map.cpp
  src/scan_processor.cpp
)


add_executable(go_to_server
  src/go_to_server.cpp

)

add_executable(map_server_node
  src/map_server.cpp
)


target_include_directories(path_planner_node PRIVATE
  include
)


target_include_directories(path_planner_node_v2 PRIVATE
  include
)


target_include_directories(dynamic_map_node PRIVATE
  include
)

target_link_libraries(path_planner_node
  yaml-cpp
  ${OpenCV_LIBS}
)

target_link_libraries(path_planner_node_v2
  yaml-cpp
  ${OpenCV_LIBS}
)

target_link_libraries(go_to_server
  yaml-cpp
)

target_link_libraries(map_server_node
  yaml-cpp
  ${OpenCV_LIBS})


ament_target_dependencies(
  path_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
  OpenCV
  puzzlebot_interfaces
)


ament_target_dependencies(
  path_planner_node_v2
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
  OpenCV
  puzzlebot_interfaces
)

ament_target_dependencies(
  map_server_node
  rclcpp
  nav_msgs
  ament_index_cpp
  OpenCV
)


ament_target_dependencies(
  dynamic_map_node
  rclcpp
  nav_msgs
  sensor_msgs
  ament_index_cpp
  OpenCV
  tf2
  tf2_ros
  tf2_geometry_msgs
)


ament_target_dependencies(
  go_to_server
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
  OpenCV
  puzzlebot_interfaces
)




# With older GCC versions, std::filesystem may require linking stdc++fs.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
  target_link_libraries(path_planner_node stdc++fs)
  target_link_libraries(path_planner_node_v2 stdc++fs)
  target_link_libraries(dynamic_map_node stdc++fs)
endif()


install(TARGETS
  path_planner_node
  path_planner_node_v2
  dynamic_map_node
  map_server_node
  go_to_server
  DESTINATION lib/${PROJECT_NAME}
)


# Install public headers.
install(DIRECTORY include/
  DESTINATION include/
)

# Install maps so they can be found using:
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# Install launch files.
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install parameter/configuration files.
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install RViz configuration files.
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()