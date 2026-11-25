cmake_minimum_required(VERSION 3.16)
project(map_localizer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)  # Jazzy's rclcpp needs C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
# TODO(step 2): find small_gicp as installed by the Dockerfile (its CMake package config), and OpenMP.

# The node's logic, as a library: the executable and the tests both link it.
add_library(${PROJECT_NAME}_lib src/map_localizer.cpp)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# TODO(step 2): link the library against the packages above and small_gicp
#   (ament_target_dependencies for the ROS ones, target_link_libraries for PCL, Eigen, small_gicp).

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

# TODO(step 2): install the executable to lib/${PROJECT_NAME}, so `ros2 run map_localizer
#   map_localizer_node` finds it.

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # TODO(step 2): ament_add_gtest(test_map_localizer test/test_map_localizer.cpp) linked to the library.
endif()

ament_package()
