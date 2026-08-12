/// ===============================================================================================
/// @file
/// @brief Self-contained monotonic memory resource backed by one heap buffer.
/// ===============================================================================================
#ifndef VORTEX_TESTS_FIXTURES_ARENA_MEMORY_RESOURCE_HPP
#define VORTEX_TESTS_FIXTURES_ARENA_MEMORY_RESOURCE_HPP

#include <cstddef>
#include <exception>
#include <iterator>
#include <memory_resource>
#include <vector>

namespace vortex::test {

/// ===============================================================================================
/// @brief Thrown when a block is released through an arena that never produced it.
/// Carries a literal and nothing else, so reporting it cannot allocate.
/// ===============================================================================================
class foreign_deallocation : public std::exception {
 public:
  [[nodiscard]] auto what() const noexcept -> const char* override {
    return "block released through an arena_memory_resource that did not allocate it";
  }
};

/// ===============================================================================================
/// @brief A monotonic memory resource that owns its storage.
///
/// One `std::vector` buffer taken from the heap up front, handed to a
/// `std::pmr::monotonic_buffer_resource` that bumps a pointer through it. Upstream is
/// `null_memory_resource`, so the arena never reaches the heap again -- it serves from the buffer
/// and throws `std::bad_alloc` once that is spent. Which is what makes it usable under
/// `memory_guard`.
///
/// The buffer being a contiguous range the arena owns, its blocks are distinguishable by address
/// alone, so a container releasing through the wrong resource is caught outright rather than
/// counted for someone to assert on afterwards.
///
/// @note That check throws from `deallocate`, which is reached from destructors -- blaze's
/// container destructors, and `memory_scope_allocator::deallocate`, are `noexcept`. So a genuine
/// mismatch terminates the process with the message above rather than unwinding. That is the loud
/// failure it is meant to be, but it aborts the test binary instead of failing one assertion.
/// ===============================================================================================
class arena_memory_resource : public std::pmr::memory_resource {
 public:
  static constexpr std::size_t default_capacity = std::size_t{1} << 20U;

  /// @param capacity Bytes to reserve from the heap. Exhausting it throws `std::bad_alloc`.
  explicit arena_memory_resource(std::size_t capacity = default_capacity)
      : buffer_(capacity),
        arena_{buffer_.data(), buffer_.size(), std::pmr::null_memory_resource()} {}

  /// @brief Whether @p p points into this arena's buffer.
  [[nodiscard]] auto owns(const void* const p) const noexcept -> bool {
    const auto* const address = static_cast<const std::byte*>(p);
    return address >= buffer_.data() && address < std::next(buffer_.data(), buffer_.size());
  }

 protected:
  auto do_allocate(std::size_t bytes, std::size_t alignment) -> void* override {
    return arena_.allocate(bytes, alignment);
  }

  /// @brief Monotonic, so this reclaims nothing; it is here to reject a foreign block.
  auto do_deallocate(void* const p, std::size_t bytes, std::size_t alignment) -> void override {
    if (not owns(p)) {
      throw foreign_deallocation{};
    }
    arena_.deallocate(p, bytes, alignment);
  }

  auto do_is_equal(const std::pmr::memory_resource& other) const noexcept -> bool override {
    return this == &other;
  }

 private:
  std::vector<std::byte> buffer_;
  /// @brief Declared after the buffer, since it is constructed pointing into it.
  std::pmr::monotonic_buffer_resource arena_;
};

}  // namespace vortex::test

#endif  // VORTEX_TESTS_FIXTURES_ARENA_MEMORY_RESOURCE_HPP
