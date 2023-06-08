cmake_minimum_required(VERSION 3.8)
project(vector_transmission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vector_transmission_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  vector_transmission_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/VectorConsumer.cpp
  src/${PROJECT_NAME}/VectorProducer.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(vector_transmission_mt src/vector_transmission_mt.cpp)
ament_target_dependencies(vector_transmission_mt ${dependencies})
target_link_libraries(vector_transmission_mt ${PROJECT_NAME})

add_executable(vector_transmission_mtr src/vector_transmission_mtr.cpp)
ament_target_dependencies(vector_transmission_mtr ${dependencies})
target_link_libraries(vector_transmission_mtr ${PROJECT_NAME})

add_executable(vector_transmission_st src/vector_transmission_st.cpp)
ament_target_dependencies(vector_transmission_st ${dependencies})
target_link_libraries(vector_transmission_st ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  vector_transmission_mt
  vector_transmission_mtr
  vector_transmission_st
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
