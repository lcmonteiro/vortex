/// ===============================================================================================
/// @file
///
/// @brief vortex.math.memory component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_MATH_MATH_MEMORY_HPP
#define VORTEX_FOUNDATION_MATH_MATH_MEMORY_HPP
#include <blaze/util/typetraits/AlignmentOf.h>

#include <cstddef>
#include <memory_resource>

#include "helpers/memory.hpp"

namespace vortex::math {
/// ===============================================================================================
/// @brief Allocator that draws from the currently active `helpers::memory_scope` resource, so that
/// dynamically-sized blaze matrices/vectors are allocated from the same scoped resource as the
/// rest of vortex (e.g. `vortex::dual::number`), instead of blaze's default global allocator.
/// ================================================================================================
template <class Type>
class memory_scope_allocator {
 public:
  using value_type = Type;

  /// @brief Constructs the allocator, capturing the resource active on this thread right now.
  memory_scope_allocator() noexcept : resource_{helpers::memory_scope::get_resource()} {}

  /// @brief Rebind-style converting constructor, required by std::allocator_traits.
  /// @param other The allocator to copy the captured resource from.
  template <class Other>
  explicit memory_scope_allocator(const memory_scope_allocator<Other>& other) noexcept
      : resource_{other.resource()} {}

  /// @brief Allocates storage for n objects of Type from the captured resource.
  /// @param n Number of objects to allocate storage for.
  /// @return Pointer to the allocated (uninitialized) storage.
  [[nodiscard]]
  auto allocate(std::size_t n) -> Type* {
    return static_cast<Type*>(resource_->allocate(n * sizeof(Type), blaze::AlignmentOf_v<Type>));
  }

  /// @brief Deallocates storage previously obtained from `allocate`.
  /// @param p Pointer previously returned by `allocate`.
  /// @param n Number of objects originally requested (must match the `allocate` call).
  auto deallocate(Type* p, std::size_t n) noexcept -> void {
    resource_->deallocate(p, n * sizeof(Type), blaze::AlignmentOf_v<Type>);
  }

  /// @brief Returns the memory resource captured by this allocator.
  [[nodiscard]]
  auto resource() const noexcept -> std::pmr::memory_resource* {
    return resource_;
  }

  /// @brief Two allocators compare equal if they share the same underlying resource.
  friend auto operator==(const memory_scope_allocator& lhs,
                         const memory_scope_allocator& rhs) noexcept -> bool {
    return *lhs.resource_ == *rhs.resource_;
  }

 private:
  std::pmr::memory_resource* resource_;
};

}  // namespace vortex::math

#endif  // VORTEX_FOUNDATION_MATH_MATH_MEMORY_HPP
