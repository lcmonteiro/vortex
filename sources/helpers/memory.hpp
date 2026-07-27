/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.helper.memory component
/// ===========================================================================
#ifndef VORTEX_HELPERS_MEMORY_HPP
#define VORTEX_HELPERS_MEMORY_HPP
#include <memory>
#include <memory_resource>

#include "sources/helpers/compat.hpp"

namespace vortex::graph {
namespace detail {

/// @brief A memory resource wrapper that enforces a maximum allocation size
/// using assert.
///
/// This class inherits from std::pmr::memory_resource and adds a compile-time
/// bound on individual allocation sizes. If an allocation exceeds the specified
/// MaxAllocSize, an assertion failure occurs (in debug builds).
///
/// @tparam MaxAllocSize The maximum number of bytes allowed per allocation.
template <size_t MaxAllocSize>
class BoundedMemoryResource : public std::pmr::memory_resource {
 public:
  /// @brief Constructs the bounded memory resource.
  /// @param upstream The upstream memory resource to forward allocations to.
  explicit BoundedMemoryResource(memory_resource* const upstream) : upstream_(upstream) {}

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

}  // namespace detail
}  // namespace vortex::graph

#endif  // VORTEX_HELPERS_MEMORY_HPP
