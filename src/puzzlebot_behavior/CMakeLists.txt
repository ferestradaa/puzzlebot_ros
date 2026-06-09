cmake_minimum_required(VERSION 3.8)
project(puzzlebot_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(puzzlebot_interfaces REQUIRED)


include_directories(bt_nodes)


add_executable(bt_executor
   src/bt_executor.cpp)

add_executable(bt_qr
   src/bt_qr.cpp)

add_executable(bt_inference
   src/bt_inference.cpp)

add_executable(bt_voice
   src/bt_voice.cpp)

add_executable(process_manager
   src/process_manager.cpp)

   
target_include_directories(bt_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)


ament_target_dependencies(bt_executor
   behaviortree_cpp 
   rclcpp
   rclcpp_action
   ament_index_cpp
   geometry_msgs
   std_srvs
   puzzlebot_interfaces
   tf2_ros
   tf2_geometry_msgs
   tf2
   
)


ament_target_dependencies(bt_qr
   behaviortree_cpp 
   rclcpp
   rclcpp_action
   ament_index_cpp
   geometry_msgs
   std_srvs
   puzzlebot_interfaces
   tf2_ros
   tf2_geometry_msgs
   tf2
   
)


ament_target_dependencies(bt_inference
   behaviortree_cpp 
   rclcpp
   rclcpp_action
   ament_index_cpp
   geometry_msgs
   std_srvs
   puzzlebot_interfaces
   tf2_ros
   tf2_geometry_msgs
   tf2
)


ament_target_dependencies(bt_voice
   behaviortree_cpp 
   rclcpp
   rclcpp_action
   ament_index_cpp
   geometry_msgs
   std_srvs
   puzzlebot_interfaces
   tf2_ros
   tf2_geometry_msgs
   tf2
   
)

ament_target_dependencies(process_manager
    rclcpp
    rclcpp_action
    puzzlebot_interfaces
   
)
  
install(TARGETS bt_executor bt_inference process_manager bt_qr bt_voice
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

target_link_libraries(bt_executor
   yaml-cpp)

target_link_libraries(bt_qr
   yaml-cpp)

target_link_libraries(bt_inference
   yaml-cpp)


target_link_libraries(bt_voice
   yaml-cpp)






# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
