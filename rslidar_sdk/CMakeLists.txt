cmake_minimum_required(VERSION 3.8)
project(ground_segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io search segmentation)
find_package(tf2 REQUIRED)

add_executable(ground_segmentation_node
  src/ground_segmentation_node.cpp
)

ament_target_dependencies(ground_segmentation_node
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2
)

target_include_directories(ground_segmentation_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(ground_segmentation_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  ground_segmentation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
