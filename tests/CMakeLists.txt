include(GoogleTest)

add_executable(g2o_dual_tests
  dual_test.cpp
  optimization_test.cpp)

target_link_libraries(g2o_dual_tests PRIVATE g2o_dual::g2o_dual gtest_main)
target_include_directories(g2o_dual_tests PRIVATE ${CMAKE_SOURCE_DIR})

gtest_discover_tests(g2o_dual_tests)
