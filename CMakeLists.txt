cmake_minimum_required(VERSION 3.8)
project(detect)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(detect_node
  src/detect_node.cpp
  src/detect.cpp
  src/process.cpp
)


target_link_libraries(detect_node
  ${OpenCV_LIBS}
)

ament_target_dependencies(detect_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

install(TARGETS detect_node
  DESTINATION lib/${PROJECT_NAME}
)


if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

ament_package()
