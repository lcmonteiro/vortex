include(GoogleTest)

add_executable(vortex_tests
  dual_test.cpp
  optimization_test.cpp
  optimization_trajectory_test.cpp
  graph_container_test.cpp
  vector_set_test.cpp
  linear_solver_test.cpp
  information_matrix_test.cpp
  graph_operations_test.cpp)

target_link_libraries(vortex_tests PRIVATE vortex::vortex gtest_main)
target_include_directories(vortex_tests PRIVATE ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/sources)

gtest_discover_tests(vortex_tests)
