cmake_minimum_required(VERSION 3.5)
project(bt_bumpgo)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)


set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    behaviortree_cpp_v3
    sensor_msgs
    geometry_msgs
    ament_index_cpp
)

include_directories(include)

add_library(forward_bt_node SHARED src/bt_bumpgo/Forward.cpp)
add_library(back_bt_node SHARED src/bt_bumpgo/Back.cpp)
add_library(turn_bt_node SHARED src/bt_bumpgo/Turn.cpp)
add_library(is_obstacle_bt_node SHARED src/bt_bumpgo/IsObstacle.cpp)

list(APPEND plugin_libs
  forward_bt_node
  back_bt_node
  turn_bt_node
  is_obstacle_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(bt_bumpgo src/bt_bumpgo_main.cpp)
ament_target_dependencies(bt_bumpgo ${dependencies})

install(TARGETS
  ${plugin_libs}
  bt_bumpgo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY behavior_tree_xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY behavior_tree_xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
