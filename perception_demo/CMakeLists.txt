cmake_minimum_required(VERSION 3.8)
project(perception_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL 1.8 REQUIRED QUIET)
find_package(pcl_conversions REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_geometry
  image_transport
  OpenCV
  PCL
  pcl_conversions
)

# OpenCV and PCL node
include_directories(include)
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/OpenCVPubSub.cpp
  src/${PROJECT_NAME}/PCLPubSub.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(main_opencv_pub_sub src/main_opencv_pub_sub.cpp)
ament_target_dependencies(main_opencv_pub_sub ${dependencies})
target_link_libraries(main_opencv_pub_sub ${PROJECT_NAME})

add_executable(main_pcl_pub_sub src/main_pcl_pub_sub.cpp)
ament_target_dependencies(main_pcl_pub_sub ${dependencies})
target_link_libraries(main_pcl_pub_sub ${PROJECT_NAME})

add_executable(main_image_transport_pub src/main_image_transport_pub.cpp)
ament_target_dependencies(main_image_transport_pub ${dependencies})
target_link_libraries(main_image_transport_pub ${PROJECT_NAME})

add_executable(main_image_transport_sub src/main_image_transport_sub.cpp)
ament_target_dependencies(main_image_transport_sub ${dependencies})
target_link_libraries(main_image_transport_sub ${PROJECT_NAME})

add_executable(main_image_pub src/main_image_pub.cpp)
ament_target_dependencies(main_image_pub ${dependencies})
target_link_libraries(main_image_pub ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  main_opencv_pub_sub
  main_pcl_pub_sub
  main_image_transport_pub
  main_image_transport_sub
  main_image_pub
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
