/// ===============================================================================================
/// @file
/// @brief Scope guard that forbids heap allocation.
/// ===============================================================================================
#ifndef VORTEX_TESTS_FIXTURES_MEMORY_GUARD_HPP
#define VORTEX_TESTS_FIXTURES_MEMORY_GUARD_HPP

#include <new>

namespace vortex::test {

/// ===============================================================================================
/// @brief Thrown by `operator new` when an allocation happens inside a `memory_guard` scope.
/// Carries a literal and nothing else: building it must not itself allocate.
/// ===============================================================================================
class unexpected_allocation : public std::bad_alloc {
 public:
  [[nodiscard]] auto what() const noexcept -> const char* override {
    return "heap allocation inside a memory_guard scope";
  }
};

/// ===============================================================================================
/// @brief Forbids heap allocation while in scope, throwing at the offending allocation itself so a
/// stack trace points at it.
///
/// The test binary replaces every form of the global `operator new`, so this covers raw `new`,
/// standard containers, and any `std::pmr` resource over `std::pmr::new_delete_resource`. A
/// resource owning its storage (`arena_memory_resource`) never reaches `operator new` and stays
/// allowed: the work under test can still get memory, just not from the heap.
///
/// Only the first allocation is reported. The guard disarms itself before throwing, because the
/// throw unwinds and a destructor allocating on the way out would otherwise throw into an
/// exception already in flight -- `std::terminate`, not a test failure.
///
/// @note Not nestable, and arming is per-thread: the flag is plain on/off, so an inner guard's
/// destructor disarms an enclosing one. One guard per scope under test.
///
/// @note `blaze::AlignedAllocator` splits on `AlignmentOf_v<T>`: 8 or above reaches
/// `posix_memalign` and is invisible here, below it calls `operator new[]` and is caught. Under
/// SSE2 every vectorizable type reports 16 and slips past; an unvectorized build reports plain
/// `alignof`, leaving only `double` outside -- and blaze 3.8.2 has no NEON, so that is aarch64.
///
/// The `nothrow` forms cannot throw and return `nullptr` instead.
/// ===============================================================================================
class memory_guard {
 public:
  memory_guard() noexcept;
  ~memory_guard() noexcept;

  memory_guard(const memory_guard&) = delete;
  memory_guard(memory_guard&&) = delete;
  auto operator=(const memory_guard&) -> memory_guard& = delete;
  auto operator=(memory_guard&&) -> memory_guard& = delete;
};

}  // namespace vortex::test

#endif  // VORTEX_TESTS_FIXTURES_MEMORY_GUARD_HPP
