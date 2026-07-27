cmake_minimum_required(VERSION 3.24)
project(g2o_dual LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Global settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(G2O_DUAL_BUILD_TESTS "Build g2o-dual tests" ON)

include(FetchContent)

# ---------------------------------------------------------------------------
# Blaze (linear algebra backend, header-only)
# ---------------------------------------------------------------------------
FetchContent_Declare(
  blaze
  GIT_REPOSITORY https://bitbucket.org/blaze-lib/blaze.git
  GIT_TAG v3.8.2
  GIT_SHALLOW TRUE)
# Blaze's own CMake pulls in extra tooling; we only need the headers.
FetchContent_GetProperties(blaze)
if(NOT blaze_POPULATED)
  FetchContent_Populate(blaze)
endif()

# LAPACK / BLAS are required by the Blaze dense solvers (sysv/potrf/potrs).
find_library(LAPACK_LIB NAMES lapack liblapack.so.3 PATHS /lib/x86_64-linux-gnu)
find_library(BLAS_LIB NAMES blas libblas.so.3 PATHS /lib/x86_64-linux-gnu)

# ---------------------------------------------------------------------------
# g2o_dual library
# ---------------------------------------------------------------------------
add_library(g2o_dual STATIC sources/g2o_dual.cpp)
target_compile_features(g2o_dual PUBLIC cxx_std_23)
target_include_directories(g2o_dual PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(g2o_dual SYSTEM PUBLIC ${blaze_SOURCE_DIR})
target_link_libraries(g2o_dual PUBLIC ${LAPACK_LIB} ${BLAS_LIB})
add_library(g2o_dual::g2o_dual ALIAS g2o_dual)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(G2O_DUAL_BUILD_TESTS)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
    GIT_SHALLOW TRUE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  add_subdirectory(tests)
endif()
