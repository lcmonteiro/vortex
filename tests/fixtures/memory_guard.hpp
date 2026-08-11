/// ===============================================================================================
/// @file
/// @brief RAII counter for global heap allocations, for allocation-behaviour tests.
/// ===============================================================================================
#ifndef VORTEX_TESTS_FIXTURES_MEMORY_GUARD_HPP
#define VORTEX_TESTS_FIXTURES_MEMORY_GUARD_HPP

#include <cstddef>

namespace vortex::test {

/// @brief Running totals of the replaced global allocation functions. Counting is always on, so
/// there is no state to enable and nothing to reset -- `memory_guard` just takes snapshots.
[[nodiscard]] auto heap_allocation_count() noexcept -> std::size_t;
[[nodiscard]] auto heap_deallocation_count() noexcept -> std::size_t;

/// ===============================================================================================
/// @brief Counts the heap traffic that happens during its lifetime.
///
///     const memory_guard guard;
///     do_work();
///     EXPECT_EQ(guard.allocations(), 0U);
///
/// The test binary replaces the global `operator new`/`operator delete` (every form, including the
/// sized, aligned and nothrow ones), so this sees allocations from any source that goes through
/// them: raw `new`, standard containers, and every `std::pmr` resource built on
/// `std::pmr::new_delete_resource` -- which includes the default resource and the upstream of the
/// graph's arena.
///
/// @note It does not see `blaze::AlignedAllocator`. That calls `posix_memalign` directly
/// (blaze/util/Memory.h), bypassing `operator new` entirely, so anything declared as a plain
/// `blaze::DynamicVector`/`DynamicMatrix` is invisible here. Observing it would mean interposing a
/// libc symbol in the test binary, which is not worth the portability risk; it is measured out of
/// band instead and is currently zero for a warmed-up solve.
///
/// Nesting works, and the counters are atomic, so a guard also stays correct if the code under
/// test allocates on another thread.
/// ===============================================================================================
class memory_guard {
 public:
  memory_guard() noexcept
      : allocations_{heap_allocation_count()}, deallocations_{heap_deallocation_count()} {}

  memory_guard(const memory_guard&) = delete;
  memory_guard(memory_guard&&) = delete;
  auto operator=(const memory_guard&) -> memory_guard& = delete;
  auto operator=(memory_guard&&) -> memory_guard& = delete;

  /// @brief Allocations made since the guard was created.
  [[nodiscard]] auto allocations() const noexcept -> std::size_t {
    return heap_allocation_count() - allocations_;
  }

  /// @brief Deallocations made since the guard was created.
  [[nodiscard]] auto deallocations() const noexcept -> std::size_t {
    return heap_deallocation_count() - deallocations_;
  }

 private:
  std::size_t allocations_;
  std::size_t deallocations_;
};

}  // namespace vortex::test

#endif  // VORTEX_TESTS_FIXTURES_MEMORY_GUARD_HPP
