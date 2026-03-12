cmake_minimum_required(VERSION 3.5)
project(perception)

# C++17 for structured bindings and std::optional
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)
find_package(OpenCV REQUIRED)

add_executable(pointcloud_processor
  src/pointcloud_processor.cpp
)

target_include_directories(pointcloud_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(pointcloud_processor
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

ament_target_dependencies(pointcloud_processor
  rclcpp
  sensor_msgs
  geometry_msgs
  vision_msgs
  cv_bridge
  image_transport
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  message_filters
)

install(TARGETS pointcloud_processor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
