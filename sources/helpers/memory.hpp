/// ===============================================================================================
/// @file
///
/// @brief vortex.helpers.memory component
/// ===============================================================================================
#ifndef VORTEX_HELPERS_MEMORY_HPP
#define VORTEX_HELPERS_MEMORY_HPP
#include <cstddef>
#include <memory_resource>

#include "helpers/contracts.hpp"

namespace vortex::helpers {
/// ===============================================================================================
/// @brief A memory resource wrapper that enforces a maximum allocation size
/// using assert.
///
/// This class inherits from std::pmr::memory_resource and adds a compile-time
/// bound on individual allocation sizes. If an allocation exceeds the specified
/// MaxAllocSize, an assertion failure occurs (in debug builds).
///
/// @tparam MaxAllocSize The maximum number of bytes allowed per allocation.
/// ================================================================================================
template <std::size_t MaxAllocSize>
class bounded_memory_resource : public std::pmr::memory_resource {
 public:
  /// @brief Constructs the bounded memory resource.
  /// @param upstream The upstream memory resource to forward allocations to.
  explicit bounded_memory_resource(std::pmr::memory_resource* const upstream)
      : upstream_(upstream) {}

 protected:
  /// @brief Allocates memory, checking against MaxAllocSize.
  /// @param bytes Number of bytes to allocate.
  /// @param alignment Required alignment.
  /// @return Pointer to allocated memory.
  ///
  /// Triggers VORTEX_PRECONDITION if bytes > MaxAllocSize.
  auto do_allocate(std::size_t bytes, std::size_t alignment) -> void* override {
    VORTEX_PRECONDITION(bytes <= MaxAllocSize, "Allocation exceeds maximum allowed");
    return upstream_->allocate(bytes, alignment);
  }

  /// @brief Deallocates memory.
  /// @param p Pointer to memory to deallocate.
  /// @param bytes Size of the allocation (must match original size).
  /// @param alignment Alignment of the original allocation.
  auto do_deallocate(void* p, std::size_t bytes, std::size_t alignment) -> void override {
    upstream_->deallocate(p, bytes, alignment);
  }

  /// @brief Compares this memory resource with another.
  /// @param other The other memory resource.
  /// @return true if the resources are the same object.
  auto do_is_equal(const std::pmr::memory_resource& other) const noexcept -> bool override {
    return this == &other;
  }

 private:
  std::pmr::memory_resource* upstream_;
};

/// ===============================================================================================
/// @brief RAII guard that temporarily installs a thread-local std::pmr::memory_resource, restoring
/// the previous one on destruction. Supports nesting (LIFO).
///
/// Not thread-safe to share a single memory_scope instance across threads — each thread has its own
/// independent active resource, and each memory_scope must be constructed/destroyed on the same
/// thread.
///
/// The caller is responsible for ensuring the pointed-to resource
/// outlives the memory_scope.
/// ================================================================================================
class memory_scope final {
 public:
  using memory = std::pmr::memory_resource;

  /// @brief Constructs a memory_scope, setting the active resource for the current thread.
  /// @param resource The memory resource to set as active.
  explicit memory_scope(memory* const resource) noexcept : previous_(current()) {
    VORTEX_PRECONDITION(resource != nullptr, "memory_scope requires a non-null resource");
    current() = resource;
  }

  /// @brief Destroys the memory_scope, restoring the previous active resource.
  ~memory_scope() noexcept { current() = previous_; }

  /// @brief Deleted copy and move constructors and assignment operators to prevent copying/moving.
  memory_scope(const memory_scope&) = delete;
  memory_scope(memory_scope&&) = delete;
  auto operator=(const memory_scope&) -> memory_scope& = delete;
  auto operator=(memory_scope&&) -> memory_scope& = delete;

  /// @brief Returns the resource currently active on this thread.
  /// @return The active resource. Never null (falls back to
  /// std::pmr::get_default_resource()).
  static auto get_resource() noexcept -> memory* { return current(); }

 private:
  /// @brief Returns a reference to the thread-local pointer to the current resource.
  /// @return Reference to the thread-local pointer.
  static auto current() noexcept -> memory*& {
    thread_local memory* res = std::pmr::get_default_resource();
    return res;
  }

  memory* const previous_;
};

}  // namespace vortex::helpers

#endif  // VORTEX_HELPERS_MEMORY_HPP
