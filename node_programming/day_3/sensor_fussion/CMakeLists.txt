cmake_minimum_required(VERSION 3.8)
project(sensor_fussion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  vision_msgs
  lifecycle_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/LaserDriver.cpp
  src/${PROJECT_NAME}/CameraDriver.cpp
  src/${PROJECT_NAME}/FussionNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

rclcpp_components_register_nodes(${PROJECT_NAME} "sensor_fussion::LaserDriver")
rclcpp_components_register_nodes(${PROJECT_NAME} "sensor_fussion::CameraDriver")
rclcpp_components_register_nodes(${PROJECT_NAME} "sensor_fussion::FussionNode")

add_executable(startup_script src/startup_script.cpp)
ament_target_dependencies(startup_script ${dependencies})

install(TARGETS
  ${PROJECT_NAME}
  startup_script
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
