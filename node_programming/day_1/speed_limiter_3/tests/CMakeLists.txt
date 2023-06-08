
ament_add_gtest(speed_limiter_test speed_limiter_test.cpp)
ament_target_dependencies(speed_limiter_test ${dependencies})
target_link_libraries(speed_limiter_test ${PROJECT_NAME})
