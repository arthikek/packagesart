cmake_minimum_required(VERSION 3.5)
project(raspimouse)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)

# Include directories
include_directories(include)

# Create executable
add_executable(fusionnode2 src/fusionnode2.cpp)

# Set dependencies for this executable
ament_target_dependencies(fusionnode2
  rclcpp
  geometry_msgs
  sensor_msgs
  message_filters)

# Link libraries
target_link_libraries(fusionnode2 ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Installation rules
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS fusionnode2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate package information
ament_package()
