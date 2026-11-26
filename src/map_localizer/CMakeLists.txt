cmake_minimum_required(VERSION 3.16)
project(map_localizer LANGUAGES C CXX)

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
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
# small_gicp as installed by the Dockerfile.
find_package(small_gicp REQUIRED)
find_package(OpenMP REQUIRED)

# The node's logic, as a library: the executable and the tests both link it.
add_library(${PROJECT_NAME}_lib src/map_localizer.cpp)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  message_filters
  tf2_ros
  tf2_eigen
  pcl_conversions
)
target_link_libraries(${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  small_gicp::small_gicp
  OpenMP::OpenMP_CXX
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

# lib/${PROJECT_NAME}, so `ros2 run map_localizer map_localizer_node` finds it.
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_map_localizer test/test_map_localizer.cpp)
  target_link_libraries(test_map_localizer ${PROJECT_NAME}_lib)
endif()

ament_package()
