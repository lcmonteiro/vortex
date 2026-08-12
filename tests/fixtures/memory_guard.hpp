/// ===============================================================================================
/// @file
/// @brief Scope guard that forbids heap allocation.
/// ===============================================================================================
#ifndef VORTEX_TESTS_FIXTURES_MEMORY_GUARD_HPP
#define VORTEX_TESTS_FIXTURES_MEMORY_GUARD_HPP

#include <cstddef>
#include <new>

namespace vortex::test {

/// ===============================================================================================
/// @brief Thrown by `operator new` when an allocation happens inside a `memory_guard` scope.
///
/// It carries a literal message and nothing else: building the exception must not allocate, or
/// reporting the problem would trigger the very thing being reported.
/// ===============================================================================================
class unexpected_allocation : public std::bad_alloc {
 public:
  [[nodiscard]] auto what() const noexcept -> const char* override {
    return "heap allocation inside a memory_guard scope";
  }
};

/// ===============================================================================================
/// @brief Forbids heap allocation for as long as it is in scope.
///
///     {
///       const memory_guard guard;
///       do_work();          // throws unexpected_allocation on the first `new`
///     }
///
/// The throw happens at the allocation itself, so a debugger or a stack trace points straight at
/// the offending line rather than at a counter checked afterwards.
///
/// The test binary replaces every form of the global `operator new`, so this covers allocations
/// from any source that goes through them: raw `new`, standard containers, and every `std::pmr`
/// resource built on `std::pmr::new_delete_resource` -- including the default resource. Memory
/// served from a resource that owns its storage, such as `arena_memory_resource`, does not touch
/// `operator new` and is therefore allowed, which is the point: the work under test can still get
/// memory, just not from the heap.
///
/// @note How much of `blaze::AlignedAllocator` is covered depends on the element type, and on the
/// build. It splits on `AlignmentOf_v<T>`: at 8 or above it calls `alignedAllocate`, which goes
/// straight to `posix_memalign` and is invisible here; below 8 it calls `operator new[]`, which
/// this sees. With SSE2 every vectorizable type reports 16, so all of them slip past. An
/// unvectorized build reports plain `alignof` instead, leaving only `double` and other 8-aligned
/// types outside -- and since blaze 3.8.2 has no NEON, that is what aarch64 gets, so `ipiv` and
/// friends are caught there but not on x86. Closing the gap for good would mean interposing a
/// libc symbol or patching blaze; neither is worth it for a test fixture.
///
/// Only the first allocation is reported: rejecting throws, and any allocation attempted while
/// that exception unwinds is let through, because throwing again mid-unwind is `std::terminate`
/// rather than a test failure. Guards stay stood down until the outermost one leaves scope, so
/// nesting is safe too. Arming is per-thread, so a guard on one thread does not constrain another.
///
/// The `nothrow` forms of `operator new` cannot throw, so under a guard they return `nullptr`
/// instead -- an allocation failure the caller is already required to handle.
/// ===============================================================================================
class memory_guard {
 public:
  memory_guard() noexcept;
  ~memory_guard() noexcept;

  memory_guard(const memory_guard&) = delete;
  memory_guard(memory_guard&&) = delete;
  auto operator=(const memory_guard&) -> memory_guard& = delete;
  auto operator=(memory_guard&&) -> memory_guard& = delete;

 private:
  /// @brief Guard depth in effect when this guard was constructed, reinstated on destruction.
  std::size_t previous_depth_;
};

}  // namespace vortex::test

#endif  // VORTEX_TESTS_FIXTURES_MEMORY_GUARD_HPP
