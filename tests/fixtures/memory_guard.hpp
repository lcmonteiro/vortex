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
/// @note `blaze::AlignedAllocator` is not covered. It calls `posix_memalign` directly
/// (blaze/util/Memory.h) and never reaches `operator new`, so anything declared as a plain
/// `blaze::DynamicVector`/`DynamicMatrix` slips past. Catching it would mean interposing a libc
/// symbol in the test binary, which is not worth the portability risk.
///
/// The guard disarms itself the moment it throws, so an allocation attempted while the stack is
/// unwinding cannot throw a second time and call `std::terminate`. It also means only the first
/// allocation in a scope is reported, which is all a test needs. Nesting is supported, and arming
/// is per-thread, so a guard on one thread does not constrain another.
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
  std::size_t previous_;
};

}  // namespace vortex::test

#endif  // VORTEX_TESTS_FIXTURES_MEMORY_GUARD_HPP
