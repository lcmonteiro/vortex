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
/// @brief Forbids heap allocation while in scope, throwing at the offending allocation so a stack
/// trace points at it.
///
/// The test binary replaces every form of the global `operator new`, so this covers raw `new`,
/// standard containers, and any `std::pmr` resource over `new_delete_resource`. A resource owning
/// its storage (`arena_memory_resource`) never reaches `operator new` and stays allowed.
///
/// Only the first allocation is reported: the guard disarms before throwing, since a destructor
/// allocating during the unwind would throw into an exception in flight -- `std::terminate`.
///
/// @note Not nestable. The flag is plain on/off, so an inner guard's destructor disarms an
/// enclosing one. Arming is per-thread. The `nothrow` forms return `nullptr` instead of throwing.
///
/// @note `blaze::AlignedAllocator` is only partly covered: it reaches `posix_memalign` when
/// `AlignmentOf_v<T>` is 8 or more and `operator new[]` below that. Under SSE2 every vectorizable
/// type reports 16 and slips past; unvectorized builds (aarch64, blaze 3.8.2 having no NEON)
/// report plain `alignof`, leaving only `double` outside.
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
