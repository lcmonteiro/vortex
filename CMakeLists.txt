cmake_minimum_required(VERSION 3.24)
project(vortex LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Global settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(VORTEX_BUILD_TESTS "Build vortex tests" ON)

include(FetchContent)

# ---------------------------------------------------------------------------
# Blaze (linear algebra backend, header-only)
# ---------------------------------------------------------------------------
FetchContent_Declare(
  blaze
  GIT_REPOSITORY https://bitbucket.org/blaze-lib/blaze.git
  GIT_TAG v3.8.2
  GIT_SHALLOW TRUE
  # Blaze's own CMake pulls in extra tooling and we only need the headers.
  # Pointing SOURCE_SUBDIR at a non-existent directory makes
  # FetchContent_MakeAvailable download/populate Blaze without configuring it.
  SOURCE_SUBDIR do-not-build)
FetchContent_MakeAvailable(blaze)

# LAPACK / BLAS are required by the Blaze dense solvers (sysv/potrf/potrs).
# No PATHS override here -- CMake's default search paths already cover the
# common cases (Debian/Ubuntu multiarch dirs, Termux's $PREFIX/lib, Homebrew,
# Fedora/Arch /usr/lib, etc). NAMES includes openblas' library names since on
# several platforms (Termux among them) that's the package that provides
# BLAS/LAPACK, and it may or may not also symlink libblas.so/liblapack.so.
find_library(LAPACK_LIB NAMES lapack liblapack.so.3 openblas)
find_library(BLAS_LIB NAMES blas libblas.so.3 openblas)

if(NOT LAPACK_LIB OR NOT BLAS_LIB)
  message(FATAL_ERROR
    "Could not find BLAS/LAPACK, required by Blaze's dense solvers.\n"
    "Install them for your platform, then re-run CMake:\n"
    "  Debian/Ubuntu : sudo apt-get install libblas-dev liblapack-dev\n"
    "  Fedora/RHEL   : sudo dnf install blas-devel lapack-devel\n"
    "  Arch          : sudo pacman -S blas lapack\n"
    "  macOS         : brew install openblas lapack\n"
    "  Termux        : pkg install blas-openblas\n")
endif()

# ---------------------------------------------------------------------------
# vortex library
# ---------------------------------------------------------------------------
add_library(vortex STATIC sources/vortex.cpp)
target_compile_features(vortex PUBLIC cxx_std_20)
target_include_directories(vortex PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/sources)
target_include_directories(vortex SYSTEM PUBLIC ${blaze_SOURCE_DIR})
target_link_libraries(vortex PUBLIC ${LAPACK_LIB} ${BLAS_LIB})
add_library(vortex::vortex ALIAS vortex)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(VORTEX_BUILD_TESTS)
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
