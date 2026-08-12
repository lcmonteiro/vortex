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
/// @brief Allocator that draws from the currently active `helpers::memory_scope` resource, so that
/// dynamically-sized blaze matrices/vectors are allocated from the same scoped resource as the
/// rest of vortex (e.g. `vortex::dual::number`), instead of blaze's default global allocator.
///
/// Two rules, and no state: a block is taken from the scope active at the moment it is allocated,
/// and released through the resource recorded with it.
///
/// @note The recording is what makes this correct. Blaze does not treat the allocator as part of a
/// container's value -- move construction leaves `alloc_` default-constructed, move assignment
/// keeps the destination's, and `resize()`/`reserve()`/`swap()` exchange the buffer via
/// `std::swap` while each object keeps its own -- so a buffer routinely ends up owned by a
/// container whose allocator never produced it. Storing the resource in a header just before the
/// block sidesteps all of that: any instance can release any block, hence `is_always_equal`.
///
/// @warning A buffer from a scoped arena must be released before that arena dies, as for any
/// arena-allocated object.
///
/// @warning By itself this covers containers the project declares, not blaze's expression
/// temporaries. `DynamicMatrix`/`DynamicVector` hardwire `AllocatorType` to `AlignedAllocator`
/// rather than reporting their own `Alloc` (still so on blaze 3.9.0), and `GetAllocator` -- which
/// every arithmetic trait consults -- is an alias template and cannot be specialised, so result
/// types always come back allocated by `AlignedAllocator`. math_types_test.cpp pins this.
///
/// Building with VORTEX_BLAZE_SCOPED_ALLOCATOR closes that gap from the other side, patching
/// `AlignedAllocator` itself to draw from the scope. The result types are unchanged -- what
/// changes is where `AlignedAllocator` gets its memory.
/// ================================================================================================
template <class Type>
class memory_scope_allocator {
 public:
  using value_type = Type;

  /// @brief Stateless, and ownership travels with the buffer, so any instance can release any
  /// block.
  using is_always_equal = std::true_type;

  memory_scope_allocator() noexcept = default;

  /// @brief Rebind-style converting constructor, required by std::allocator_traits.
  template <class Other>
  memory_scope_allocator(const memory_scope_allocator<Other>& /*other*/) noexcept {}  // NOLINT

  /// @brief Allocates storage for n objects of Type from the scope active right now.
  [[nodiscard]]
  auto allocate(std::size_t n) -> Type* {
    return static_cast<Type*>(scoped_allocate(n * sizeof(Type), kAlignment));
  }

  /// @brief Releases storage through the resource recorded with the block. Blaze destroys
  /// default-constructed containers by deallocating a null buffer, which is a no-op here.
  auto deallocate(Type* p, std::size_t n) noexcept -> void {
    scoped_deallocate(p, n * sizeof(Type), kAlignment);
  }

  friend auto operator==(const memory_scope_allocator&, const memory_scope_allocator&) noexcept
      -> bool {
    return true;
  }

 private:
  /// @brief Blaze rejects a buffer whose address is not a multiple of `AlignmentOf_v`, the SIMD
  /// alignment for the active instruction set.
  static constexpr std::size_t kAlignment = blaze::AlignmentOf_v<Type>;
};

/// ===============================================================================================
/// @brief Allocator that always takes from the process heap, whatever scope is active.
///
/// For storage that outlives the scope it was created in -- the solvers' `thread_local` LAPACK
/// workspaces -- where drawing from an arena would mean releasing through it long after it died.
/// Blaze's own default allocator does this too, but only while it is unpatched; naming it here
/// says the choice is deliberate rather than inherited.
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
