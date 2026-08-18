/// ===============================================================================================
/// @file
/// @brief Test memory resource that owns a fixed-capacity buffer and validates what it hands back.
/// ===============================================================================================
#ifndef VORTEX_TESTS_HELPERS_BOUNDED_ARENA_RESOURCE_HPP
#define VORTEX_TESTS_HELPERS_BOUNDED_ARENA_RESOURCE_HPP
#include <cstddef>
#include <iterator>
#include <memory_resource>
#include <new>
#include <vector>

namespace vortex::test {

/// ===============================================================================================
/// @brief Thrown on either violation this resource watches for: exhausting its buffer in a debug
/// build, or a block released through it that it never handed out. Carries a literal, so reporting
/// one cannot itself allocate.
/// ===============================================================================================
class memory_violation : public std::bad_alloc {
 public:
  explicit memory_violation(const char* const what) noexcept : what_{what} {}
  [[nodiscard]] auto what() const noexcept -> const char* override { return what_; }

 private:
  const char* what_;
};

/// ===============================================================================================
/// @brief A fixed-capacity memory resource that owns its storage and validates both ends of it.
///
/// One `std::vector` buffer from the heap up front, handed to a
/// `std::pmr::monotonic_buffer_resource`. In debug builds that resource's upstream is
/// `std::pmr::null_memory_resource()`, so exhausting @p capacity throws `std::bad_alloc` instead of
/// silently growing onto the heap; release builds fall back to @p upstream instead.
///
/// Its blocks are a contiguous range it owns, so a container releasing through the wrong resource
/// is caught by address rather than counted for someone to assert on afterwards.
///
/// @note Monotonic, so `do_deallocate` reclaims nothing -- it is here only to reject a foreign
/// block.
/// ===============================================================================================
class bounded_arena_resource : public std::pmr::memory_resource {
 public:
  static constexpr std::size_t default_capacity = std::size_t{1} << 20U;

  /// @param capacity Bytes to reserve from the heap. Exhausting it throws `std::bad_alloc` in
  /// debug builds, or forwards to @p upstream in release builds.
  /// @param upstream Resource used once @p capacity is exhausted. Only reachable in release
  /// builds.
  explicit bounded_arena_resource(std::size_t capacity, std::pmr::memory_resource* const upstream)
      : buffer_(capacity),
        arena_(std::data(buffer_), std::size(buffer_), std::pmr::null_memory_resource()) {}

  /// @brief Whether @p p points into this arena's buffer.
  [[nodiscard]] auto owns(const void* const p) const noexcept -> bool {
    const auto* const address = static_cast<const std::byte*>(p);
    return address >= buffer_.data() && address < std::next(buffer_.data(), buffer_.size());
  }

  /// =============================================================================================
  /// @brief Forbids heap allocation while in scope, throwing at the offending allocation so a
  /// stack trace points at it. The other half of this resource's validation: what it serves is
  /// allowed, everything else is not.
  ///
  /// The test binary replaces every form of the global `operator new`, so this covers raw `new`,
  /// standard containers, and any `std::pmr` resource over `new_delete_resource`. A
  /// bounded_arena_resource serves from its own buffer and never reaches `operator new`, so it
  /// stays usable here.
  ///
  /// Only the first allocation is reported: the guard disarms before throwing, since a destructor
  /// allocating during the unwind would throw into an exception in flight -- `std::terminate`.
  ///
  /// @note Not nestable. The flag is plain on/off, so an inner guard's destructor disarms an
  /// enclosing one. Arming is per-thread. The `nothrow` forms return `nullptr` instead of
  /// throwing.
  ///
  /// @note `blaze::AlignedAllocator` is only partly covered: it reaches `posix_memalign` when
  /// `AlignmentOf_v<T>` is 8 or more and `operator new[]` below that. Under SSE2 every
  /// vectorizable type reports 16 and slips past; unvectorized builds (aarch64, blaze 3.8.2 having
  /// no NEON) report plain `alignof`, leaving only `double` outside.
  /// =============================================================================================
  class guard {
   public:
    guard() noexcept;
    ~guard() noexcept;
    guard(const guard&) = delete;
    guard(guard&&) = delete;
    auto operator=(const guard&) -> guard& = delete;
    auto operator=(guard&&) -> guard& = delete;
  };

 protected:
  auto do_allocate(std::size_t bytes, std::size_t alignment) -> void* override {
    return arena_.allocate(bytes, alignment);
  }

  auto do_deallocate(void* const p, std::size_t bytes, std::size_t alignment) -> void override {
    if (not owns(p)) {
      throw memory_violation{
          "block released through a bounded_arena_resource that did not allocate it"};
    }
    arena_.deallocate(p, bytes, alignment);
  }

  auto do_is_equal(const std::pmr::memory_resource& other) const noexcept -> bool override {
    return this == &other;
  }

 private:
  std::vector<std::byte> buffer_;
  std::pmr::monotonic_buffer_resource arena_;  // after buffer_: constructed pointing into it
};

}  // namespace vortex::test

#endif  // VORTEX_TESTS_HELPERS_BOUNDED_ARENA_RESOURCE_HPP
