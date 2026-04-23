cmake_minimum_required(VERSION 3.8)
project(puzzlebot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)



add_executable(joint_state_pub src/joint_state_pub.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(sim_encoders src/sim_encoders.cpp)

ament_target_dependencies(joint_state_pub
rclcpp
sensor_msgs
std_msgs
)


ament_target_dependencies(odometry
rclcpp
nav_msgs
std_msgs
tf2_geometry_msgs
apriltag_msgs
tf2
tf2_ros
ament_index_cpp
)


ament_target_dependencies(sim_encoders
rclcpp
sensor_msgs
std_msgs
)

target_include_directories(odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(odometry yaml-cpp Eigen3::Eigen)


install(TARGETS joint_state_pub sim_encoders odometry
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

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
