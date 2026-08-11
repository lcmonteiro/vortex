/// ===============================================================================================
/// @file
/// @brief Memory resource that records the blocks it hands out, for allocation-behaviour tests.
/// ===============================================================================================
#ifndef VORTEX_TESTS_FIXTURES_TRACKING_RESOURCE_HPP
#define VORTEX_TESTS_FIXTURES_TRACKING_RESOURCE_HPP

#include <cstddef>
#include <memory_resource>
#include <set>

namespace vortex::test {

/// ===============================================================================================
/// @brief A `std::pmr::memory_resource` that counts its allocations and remembers which blocks it
/// produced, so a test can assert both how much was allocated and that every block came home.
///
/// Blaze exchanges element buffers between containers without exchanging their allocators, so a
/// buffer can end up released through the wrong resource. A plain allocation counter cannot see
/// that; it only shows up as a block being freed by a resource that never allocated it, which is
/// what `foreign_frees` counts.
/// ===============================================================================================
class tracking_resource : public std::pmr::memory_resource {
 public:
  explicit tracking_resource(std::pmr::memory_resource* const upstream) : upstream_{upstream} {}

  /// @brief Blocks handed out by this resource and not yet returned to it.
  [[nodiscard]] auto live() const -> std::size_t { return live_.size(); }

  std::size_t allocations{0};
  std::size_t foreign_frees{0};

 protected:
  auto do_allocate(std::size_t bytes, std::size_t alignment) -> void* override {
    ++allocations;
    auto* const p = upstream_->allocate(bytes, alignment);
    live_.insert(p);
    return p;
  }

  auto do_deallocate(void* p, std::size_t bytes, std::size_t alignment) -> void override {
    if (0 == live_.erase(p)) {
      // A block this resource never produced. Still forward it: tests share one upstream, so the
      // release itself stays valid and the process stays healthy.
      ++foreign_frees;
    }
    upstream_->deallocate(p, bytes, alignment);
  }

  auto do_is_equal(const std::pmr::memory_resource& other) const noexcept -> bool override {
    return this == &other;
  }

 private:
  std::pmr::memory_resource* upstream_;
  std::set<void*> live_;
};

}  // namespace vortex::test

#endif  // VORTEX_TESTS_FIXTURES_TRACKING_RESOURCE_HPP
