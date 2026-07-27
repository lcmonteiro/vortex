include(GoogleTest)

add_executable(vortex_tests
  dual_test.cpp
  optimization_test.cpp)

target_link_libraries(vortex_tests PRIVATE vortex::vortex gtest_main)
target_include_directories(vortex_tests PRIVATE ${CMAKE_SOURCE_DIR})

gtest_discover_tests(vortex_tests)
