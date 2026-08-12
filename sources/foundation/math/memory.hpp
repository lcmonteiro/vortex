/// ===============================================================================================
/// @file
///
/// @brief vortex.math.memory component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_MATH_MATH_MEMORY_HPP
#define VORTEX_FOUNDATION_MATH_MATH_MEMORY_HPP
#include <blaze/util/typetraits/AlignmentOf.h>

#include <cstddef>
#include <new>
#include <type_traits>

#include "foundation/math/scoped_memory.hpp"

namespace vortex::math {
/// ===============================================================================================
/// @brief Allocator drawing from the active `helpers::memory_scope`, so blaze's dynamic containers
/// share the scoped resource with the rest of vortex rather than using blaze's global allocator.
///
/// Two rules and no state: a block comes from the scope active when it is allocated, and is
/// released through the resource recorded with it.
///
/// @note The recording is what makes it correct. Blaze does not treat the allocator as part of a
/// container's value -- move leaves `alloc_` default-constructed, move assignment keeps the
/// destination's, and `resize()`/`reserve()`/`swap()` exchange the buffer while each object keeps
/// its own allocator -- so a buffer routinely ends up owned by a container whose allocator never
/// produced it.
///
/// @warning A buffer from a scoped arena must be released before that arena dies.
///
/// @warning This covers containers the project declares, not blaze's expression temporaries, whose
/// result types always come back on `AlignedAllocator` (see math_types_test.cpp). Building with
/// VORTEX_BLAZE_SCOPED_ALLOCATOR closes that gap by patching `AlignedAllocator` itself.
/// ================================================================================================
template <class Type>
class memory_scope_allocator {
 public:
  using value_type = Type;

  /// @brief Ownership travels with the buffer, so any instance can release any block.
  using is_always_equal = std::true_type;

  memory_scope_allocator() noexcept = default;

  /// @brief Rebind-style converting constructor, required by std::allocator_traits.
  template <class Other>
  memory_scope_allocator(const memory_scope_allocator<Other>& /*other*/) noexcept {}  // NOLINT

  [[nodiscard]]
  auto allocate(std::size_t n) -> Type* {
    return static_cast<Type*>(scoped_allocate(n * sizeof(Type), kAlignment));
  }

  /// @brief Blaze deallocates a null buffer when destroying a default-constructed container.
  auto deallocate(Type* p, std::size_t n) noexcept -> void {
    scoped_deallocate(p, n * sizeof(Type), kAlignment);
  }

  friend auto operator==(const memory_scope_allocator&, const memory_scope_allocator&) noexcept
      -> bool {
    return true;
  }

 private:
  /// @brief Blaze rejects a buffer not aligned to `AlignmentOf_v`, the SIMD alignment in force.
  static constexpr std::size_t kAlignment = blaze::AlignmentOf_v<Type>;
};

/// ===============================================================================================
/// @brief Allocator that always takes from the process heap, whatever scope is active.
///
/// For storage outliving the scope it was created in -- the solvers' `thread_local` LAPACK
/// workspaces -- where an arena block would be released long after the arena died. Blaze's default
/// allocator does this too, but only while unpatched; naming it makes the choice deliberate.
/// ================================================================================================
template <class Type>
class heap_allocator {
 public:
  using value_type = Type;
  using is_always_equal = std::true_type;

  heap_allocator() noexcept = default;

  template <class Other>
  heap_allocator(const heap_allocator<Other>& /*other*/) noexcept {}  // NOLINT

  [[nodiscard]]
  auto allocate(std::size_t n) -> Type* {
    return static_cast<Type*>(::operator new(n * sizeof(Type), kAlignment));
  }

  auto deallocate(Type* p, std::size_t n) noexcept -> void {
    ::operator delete(p, n * sizeof(Type), kAlignment);
  }

  friend auto operator==(const heap_allocator&, const heap_allocator&) noexcept -> bool {
    return true;
  }

 private:
  static constexpr auto kAlignment = std::align_val_t{blaze::AlignmentOf_v<Type>};
};

}  // namespace vortex::math

#endif  // VORTEX_FOUNDATION_MATH_MATH_MEMORY_HPP
