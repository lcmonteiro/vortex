/// ===========================================================================
/// @file
///
/// @brief vortex.helper.memory component
/// ===========================================================================
#ifndef VORTEX_HELPERS_MEMORY_HPP
#define VORTEX_HELPERS_MEMORY_HPP
#include <memory>
#include <memory_resource>

#include "helpers/contracts.hpp"

namespace vortex::helpers {

/// @brief A memory resource wrapper that enforces a maximum allocation size
/// using assert.
///
/// This class inherits from std::pmr::memory_resource and adds a compile-time
/// bound on individual allocation sizes. If an allocation exceeds the specified
/// MaxAllocSize, an assertion failure occurs (in debug builds).
///
/// @tparam MaxAllocSize The maximum number of bytes allowed per allocation.
template <size_t MaxAllocSize>
class MemoryBoundedResource : public std::pmr::memory_resource {
 public:
  /// @brief Constructs the bounded memory resource.
  /// @param upstream The upstream memory resource to forward allocations to.
  explicit MemoryBoundedResource(memory_resource* const upstream) : upstream_(upstream) {}

 protected:
  /// @brief Allocates memory, checking against MaxAllocSize.
  /// @param bytes Number of bytes to allocate.
  /// @param alignment Required alignment.
  /// @return Pointer to allocated memory.
  ///
  /// Triggers VORTEX_PRECONDITION if bytes > MaxAllocSize.
  void* do_allocate(size_t bytes, size_t alignment) override {
    VORTEX_PRECONDITION(bytes <= MaxAllocSize,
                          "Allocation exceeds maximum allowed size (check CacheBlockMaxSize)");
    return upstream_->allocate(bytes, alignment);
  }

  /// @brief Deallocates memory.
  /// @param p Pointer to memory to deallocate.
  /// @param bytes Size of the allocation (must match original size).
  /// @param alignment Alignment of the original allocation.
  void do_deallocate(void* p, size_t bytes, size_t alignment) override {
    upstream_->deallocate(p, bytes, alignment);
  }

  /// @brief Compares this memory resource with another.
  /// @param other The other memory resource.
  /// @return true if the resources are the same object.
  bool do_is_equal(const memory_resource& other) const noexcept override { return this == &other; }

 private:
  std::pmr::memory_resource* upstream_;
};

/// @brief RAII guard that temporarily installs a thread-local
///        std::pmr::memory_resource, restoring the previous one on
///        destruction. Supports nesting (LIFO).
///
/// @note  Not thread-safe to share a single MemoryScope instance across
///        threads — each thread has its own independent active resource,
///        and each MemoryScope must be constructed/destroyed on the same
///        thread.
/// @note  The caller is responsible for ensuring the pointed-to resource
///        outlives the MemoryScope.
class MemoryScope final {
 public:
  using Resource = std::pmr::memory_resource;

  explicit MemoryScope(Resource* const resource) noexcept : previous_(Current()) {
    assert(resource != nullptr && "MemoryScope requires a non-null resource");
    Current() = resource;
  }

  ~MemoryScope() noexcept { Current() = previous_; }

  MemoryScope(const MemoryScope&) = delete;
  MemoryScope& operator=(const MemoryScope&) = delete;
  MemoryScope(MemoryScope&&) = delete;
  MemoryScope& operator=(MemoryScope&&) = delete;

  /// Returns the resource currently active on this thread.
  /// Never null (falls back to std::pmr::get_default_resource()).
  static auto GetResource() noexcept -> Resource* { return Current(); }

 private:
  static auto Current() noexcept -> Resource*& {
    thread_local Resource* res = std::pmr::get_default_resource();
    return res;
  }

  Resource* const previous_;
};

}  // namespace vortex::helpers

#endif  // VORTEX_HELPERS_MEMORY_HPP
