cmake_minimum_required(VERSION 3.5)
project(fsm_bumpgo)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    kobuki_ros_interfaces
    geometry_msgs
    ament_index_cpp
)

add_executable(fsm_bumpgo src/bumpgo_node.cpp)
ament_target_dependencies(fsm_bumpgo ${dependencies})

install(TARGETS
  fsm_bumpgo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_export_dependencies(${dependencies})

ament_package()
