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

  /// @brief Forgets the counters, keeping the set of outstanding blocks intact. Used to measure a
  /// steady-state operation after warm-up allocations have already happened.
  auto reset_counters() -> void {
    allocations = 0;
    foreign_frees = 0;
  }

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

/// ===============================================================================================
/// @brief RAII guard installing a `std::pmr` default resource for the duration of a test and
/// restoring the previous one afterwards, so one test cannot leak its resource into the next.
/// ===============================================================================================
class default_resource_guard {
 public:
  explicit default_resource_guard(std::pmr::memory_resource* const resource)
      : previous_{std::pmr::set_default_resource(resource)} {}

  ~default_resource_guard() { std::pmr::set_default_resource(previous_); }

  default_resource_guard(const default_resource_guard&) = delete;
  default_resource_guard(default_resource_guard&&) = delete;
  auto operator=(const default_resource_guard&) -> default_resource_guard& = delete;
  auto operator=(default_resource_guard&&) -> default_resource_guard& = delete;

 private:
  std::pmr::memory_resource* previous_;
};

}  // namespace vortex::test

#endif  // VORTEX_TESTS_FIXTURES_TRACKING_RESOURCE_HPP
