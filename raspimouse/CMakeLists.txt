cmake_minimum_required(VERSION 3.5)
project(raspimouse)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(raspimouse_msgs REQUIRED)
find_package(ros_ign_gazebo REQUIRED)
find_package(PCL REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# Add the executable target
add_library(raspimouse_component SHARED src/raspimouse_component.cpp)

target_compile_definitions(raspimouse_component PRIVATE "RASPIMOUSE_BUILDING_DLL")
ament_target_dependencies(raspimouse_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  raspimouse_msgs)

rclcpp_components_register_nodes(raspimouse_component "raspimouse::Raspimouse")

add_executable(self_drive_camera src/self_drive_camera.cpp)
ament_target_dependencies(self_drive_camera
  rclcpp
  sensor_msgs
  cv_bridge)
target_link_libraries(self_drive_camera ${OpenCV_LIBRARIES})

add_executable(raspimouse src/raspimouse.cpp)
target_link_libraries(raspimouse raspimouse_component)


add_executable(self_drive src/self_drive.cpp)
ament_target_dependencies(self_drive
 rclcpp geometry_msgs sensor_msgs)
target_link_libraries(raspimouse ${PCL_LIBRARIES})




ament_target_dependencies(
  raspimouse
  rclcpp_lifecycle
  class_loader
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  raspimouse_msgs)

ament_export_include_directories(include)
ament_export_libraries(raspimouse_component)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)
# install(DIRECTORY worlds models DESTINATION share/${PROJECT_NAME}/)

add_executable(fusionnode2 src/fusionnode2.cpp) # add this line to create an executable for fusionnode.cpp
ament_target_dependencies(fusionnode2
  rclcpp
  geometry_msgs
  sensor_msgs
  message_filters)  # add this line to specify the dependencies for fusionnode
target_link_libraries(fusionnode2 ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}) # link the required libraries

install(
  TARGETS 
  raspimouse_component 
  self_drive
  self_drive_camera
  fusionnode
  fusionnode2
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
