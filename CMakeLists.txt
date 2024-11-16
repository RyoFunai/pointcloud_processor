cmake_minimum_required(VERSION 3.5)
project(pointcloud_processor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/pointcloud_processor.cpp
)

add_library(pointcloud_processor::pointcloud_processor ALIAS ${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  visualization_msgs
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

ament_package()