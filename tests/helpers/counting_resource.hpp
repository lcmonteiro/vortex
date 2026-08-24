/// ===============================================================================================
/// @file
/// @brief Test memory resource that counts what passes through it and forwards the rest upstream.
/// ===============================================================================================
#ifndef VORTEX_TESTS_HELPERS_COUNTING_RESOURCE_HPP
#define VORTEX_TESTS_HELPERS_COUNTING_RESOURCE_HPP
#include <cstddef>
#include <memory_resource>

namespace vortex::test {

/// ===============================================================================================
/// @brief Pass-through memory resource that tallies allocation count and bytes.
///
/// Allocation traffic is the dominant cost of dual-number evaluation, so a benchmark that reports
/// only wall time hides why one representation beats another. Placed at the top of an arena stack,
/// this records both without changing where the memory comes from.
/// ===============================================================================================
class counting_resource : public std::pmr::memory_resource {
 public:
  explicit counting_resource(std::pmr::memory_resource* const upstream) noexcept
      : upstream_{upstream} {}

  /// @brief Number of allocations served since the last reset.
  [[nodiscard]] auto allocations() const noexcept -> std::size_t { return allocations_; }

  /// @brief Bytes requested since the last reset.
  [[nodiscard]] auto bytes() const noexcept -> std::size_t { return bytes_; }

  /// @brief Clears both tallies, so a warm-up pass does not count towards the measurement.
  auto reset() noexcept -> void {
    allocations_ = 0;
    bytes_ = 0;
  }

 protected:
  auto do_allocate(std::size_t bytes, std::size_t alignment) -> void* override {
    ++allocations_;
    bytes_ += bytes;
    return upstream_->allocate(bytes, alignment);
  }

  auto do_deallocate(void* const p, std::size_t bytes, std::size_t alignment) -> void override {
    upstream_->deallocate(p, bytes, alignment);
  }

  auto do_is_equal(const std::pmr::memory_resource& other) const noexcept -> bool override {
    return this == &other;
  }

 private:
  std::pmr::memory_resource* upstream_;
  std::size_t allocations_{0};
  std::size_t bytes_{0};
};

}  // namespace vortex::test

#endif  // VORTEX_TESTS_HELPERS_COUNTING_RESOURCE_HPP
