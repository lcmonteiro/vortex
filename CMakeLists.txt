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
# Both off by default, so a consumer's build is never broken by a newer compiler's warnings. CI
# turns them on.
option(VORTEX_WARNINGS_AS_ERRORS "Fail the tests' build on any compiler warning" OFF)
option(VORTEX_SANITIZE "Build the tests with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

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

# vortex's memory model runs through blaze's AlignedAllocator, redirected into the active
# helpers::memory_scope -- the only way to reach blaze's expression temporaries, since blaze cannot
# be retargeted from the project side (see include/vortex/foundation/math/types.hpp). The patch is
# therefore not optional, and applying it here rather than via FetchContent's PATCH_COMMAND covers
# the case where blaze was supplied through FETCHCONTENT_SOURCE_DIR_BLAZE, which skips that step.
# Testing for the marker first makes this idempotent across reconfigures.
set(VORTEX_BLAZE_PATCH ${CMAKE_CURRENT_SOURCE_DIR}/include/vortex/foundation/math/patch/blaze-scoped-allocator.patch)
set(VORTEX_BLAZE_PATCH_TARGET ${blaze_SOURCE_DIR}/blaze/util/AlignedAllocator.h)

file(READ ${VORTEX_BLAZE_PATCH_TARGET} VORTEX_BLAZE_PATCH_TARGET_TEXT)
string(FIND "${VORTEX_BLAZE_PATCH_TARGET_TEXT}" "VORTEX_BLAZE_SCOPED_ALLOCATOR" VORTEX_BLAZE_PATCH_FOUND)

if(VORTEX_BLAZE_PATCH_FOUND EQUAL -1)
  message(STATUS "Applying ${VORTEX_BLAZE_PATCH} to ${blaze_SOURCE_DIR}")
  execute_process(
    COMMAND git apply --unsafe-paths --directory=. ${VORTEX_BLAZE_PATCH}
    WORKING_DIRECTORY ${blaze_SOURCE_DIR}
    RESULT_VARIABLE VORTEX_BLAZE_PATCH_STATUS
    ERROR_VARIABLE VORTEX_BLAZE_PATCH_ERROR)
  if(NOT VORTEX_BLAZE_PATCH_STATUS EQUAL 0)
    message(FATAL_ERROR
      "Failed to apply ${VORTEX_BLAZE_PATCH} to ${blaze_SOURCE_DIR}:\n"
      "${VORTEX_BLAZE_PATCH_ERROR}\n"
      "vortex requires it: without it blaze allocates outside helpers::memory_scope and "
      "nothing is arena-backed.")
  endif()
endif()

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
# vortex library (header-only)
# ---------------------------------------------------------------------------
add_library(vortex INTERFACE)
target_compile_features(vortex INTERFACE cxx_std_20)
target_include_directories(vortex INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(vortex SYSTEM INTERFACE ${blaze_SOURCE_DIR})
target_link_libraries(vortex INTERFACE ${LAPACK_LIB} ${BLAS_LIB})
target_compile_definitions(vortex INTERFACE VORTEX_BLAZE_SCOPED_ALLOCATOR)
add_library(vortex::vortex ALIAS vortex)

# ---------------------------------------------------------------------------
# Warnings, for the tests
#
# -Wshadow because generic cost functions pull math names in with `using std::sin;` and similar,
# and the library must not then reuse those names for its own variables.
# ---------------------------------------------------------------------------
function(vortex_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
    if(VORTEX_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wshadow)
    if(VORTEX_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()

# ---------------------------------------------------------------------------
# Sanitizers, for the tests
#
# -fno-sanitize-recover makes undefined behaviour fail the test rather than print and carry on.
# ---------------------------------------------------------------------------
function(vortex_sanitize target)
  if(VORTEX_SANITIZE AND NOT MSVC)
    set(flags -fsanitize=address,undefined -fno-sanitize-recover=undefined -fno-omit-frame-pointer)
    target_compile_options(${target} PRIVATE ${flags})
    target_link_options(${target} PRIVATE ${flags})
  endif()
endfunction()

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
