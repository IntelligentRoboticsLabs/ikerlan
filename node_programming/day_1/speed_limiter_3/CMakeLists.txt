cmake_minimum_required(VERSION 3.8)
project(speed_limiter_3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}/SpeedLimiter.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(speed_limiter src/speed_limiter.cpp)
ament_target_dependencies(speed_limiter ${dependencies})
target_link_libraries(speed_limiter ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  speed_limiter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_package()
