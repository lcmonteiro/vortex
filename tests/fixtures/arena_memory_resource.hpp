/// ===============================================================================================
/// @file
/// @brief Self-contained monotonic memory resource backed by one heap buffer.
/// ===============================================================================================
#ifndef VORTEX_TESTS_FIXTURES_ARENA_MEMORY_RESOURCE_HPP
#define VORTEX_TESTS_FIXTURES_ARENA_MEMORY_RESOURCE_HPP

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory_resource>
#include <new>
#include <vector>

namespace vortex::test {

/// ===============================================================================================
/// @brief A monotonic memory resource that owns its storage.
///
/// It takes one buffer from the heap up front -- like a `std::vector`, and by way of one -- then
/// serves every request by bumping a pointer through it. Nothing is reclaimed until the arena
/// itself is destroyed, so once constructed it hands out memory without touching the heap again.
/// That is what makes it usable under `memory_guard`: the heap allocation happens when the arena
/// is built, outside the guarded scope, and the work under test then draws from the buffer.
///
/// Because the buffer is a contiguous range the arena owns, it can tell its own blocks from
/// anything else by address alone. `foreign_frees()` counts pointers handed to `deallocate()` that
/// this arena never produced, which is how a container releasing a buffer through the wrong
/// resource shows up -- no bookkeeping side table needed.
/// ===============================================================================================
class arena_memory_resource : public std::pmr::memory_resource {
 public:
  static constexpr std::size_t default_capacity = std::size_t{1} << 20U;

  /// @param capacity Bytes to reserve from the heap. Exhausting it throws `std::bad_alloc`.
  explicit arena_memory_resource(std::size_t capacity = default_capacity) : buffer_(capacity) {}

  /// @brief Total bytes reserved.
  [[nodiscard]] auto capacity() const noexcept -> std::size_t { return buffer_.size(); }

  /// @brief Bytes handed out so far, padding included.
  [[nodiscard]] auto used() const noexcept -> std::size_t { return offset_; }

  /// @brief Blocks served.
  [[nodiscard]] auto allocations() const noexcept -> std::size_t { return allocations_; }

  /// @brief Blocks served and not yet handed back.
  [[nodiscard]] auto outstanding() const noexcept -> std::size_t { return allocations_ - releases_; }

  /// @brief Pointers released through this arena that it never produced.
  [[nodiscard]] auto foreign_frees() const noexcept -> std::size_t { return foreign_frees_; }

  /// @brief Whether @p p points into this arena's buffer.
  [[nodiscard]] auto owns(const void* const p) const noexcept -> bool {
    const auto* const address = static_cast<const std::byte*>(p);
    return address >= buffer_.data() && address < std::next(buffer_.data(), buffer_.size());
  }

 protected:
  auto do_allocate(std::size_t bytes, std::size_t alignment) -> void* override {
    // The buffer itself is only as aligned as `operator new` made it, so each block is aligned
    // within it. Callers can and do ask for more than that (blaze wants SIMD alignment).
    const auto base = reinterpret_cast<std::uintptr_t>(buffer_.data()) + offset_;
    const auto aligned = (base + alignment - 1U) & ~(alignment - 1U);
    const auto padding = static_cast<std::size_t>(aligned - base);
    if (offset_ + padding + bytes > buffer_.size()) {
      throw std::bad_alloc{};
    }
    offset_ += padding + bytes;
    ++allocations_;
    return reinterpret_cast<void*>(aligned);
  }

  /// @brief Monotonic: storage returns to the heap when the arena dies, not here. The pointer is
  /// still checked, so releasing a block through the wrong resource is visible.
  auto do_deallocate(void* const p, std::size_t /*bytes*/, std::size_t /*alignment*/)
      -> void override {
    if (owns(p)) {
      ++releases_;
    } else {
      ++foreign_frees_;
    }
  }

  auto do_is_equal(const std::pmr::memory_resource& other) const noexcept -> bool override {
    return this == &other;
  }

 private:
  std::vector<std::byte> buffer_;
  std::size_t offset_{0};
  std::size_t allocations_{0};
  std::size_t releases_{0};
  std::size_t foreign_frees_{0};
};

}  // namespace vortex::test

#endif  // VORTEX_TESTS_FIXTURES_ARENA_MEMORY_RESOURCE_HPP
