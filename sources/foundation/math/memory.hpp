/// ===============================================================================================
/// @file
///
/// @brief vortex.math.memory component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_MATH_MATH_MEMORY_HPP
#define VORTEX_FOUNDATION_MATH_MATH_MEMORY_HPP
#include <blaze/util/typetraits/AlignmentOf.h>

#include <cstddef>
#include <cstring>
#include <memory_resource>
#include <type_traits>

#include "helpers/memory.hpp"

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
/// @warning This covers containers the project declares, not blaze's expression temporaries.
/// `DynamicMatrix`/`DynamicVector` hardwire `AllocatorType` to `AlignedAllocator` rather than
/// reporting their own `Alloc` (still so on blaze 3.9.0), and `GetAllocator` -- which every
/// arithmetic trait consults -- is an alias template and cannot be specialised, so result types
/// always come back allocated by `AlignedAllocator`. math_types_test.cpp pins this.
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
  /// @param n Number of objects to allocate storage for.
  /// @return Pointer to the allocated (uninitialized) storage, aligned as blaze requires.
  [[nodiscard]]
  auto allocate(std::size_t n) -> Type* {
    auto* const resource = helpers::memory_scope::get_resource();
    auto* const base = static_cast<std::byte*>(resource->allocate(bytes(n), kAlignment));
    auto* const block = base + kOffset;
    // Record the owner, so `deallocate` releases through it whichever allocator instance -- and
    // whichever scope -- happens to be current by then.
    std::memcpy(block - sizeof(owner_type), &resource, sizeof(owner_type));
    return reinterpret_cast<Type*>(block);
  }

  /// @brief Deallocates storage previously obtained from `allocate`, through the resource recorded
  /// with the block.
  /// @param p Pointer previously returned by `allocate` (may be null).
  /// @param n Number of objects originally requested (must match the `allocate` call).
  auto deallocate(Type* p, std::size_t n) noexcept -> void {
    // Blaze destroys default-constructed containers by deallocating a null buffer.
    if (nullptr == p) {
      return;
    }
    auto* const block = reinterpret_cast<std::byte*>(p);
    auto owner = owner_type{};
    std::memcpy(&owner, block - sizeof(owner_type), sizeof(owner_type));
    owner->deallocate(block - kOffset, bytes(n), kAlignment);
  }

  friend auto operator==(const memory_scope_allocator&, const memory_scope_allocator&) noexcept
      -> bool {
    return true;
  }

 private:
  using owner_type = std::pmr::memory_resource*;

  /// @brief Blaze rejects a buffer whose address is not a multiple of `AlignmentOf_v` -- the SIMD
  /// alignment for the active instruction set -- and the header must itself land on a suitably
  /// aligned address, so honour whichever is stricter. Both are powers of two, so the larger is a
  /// multiple of the smaller and satisfies each.
  static constexpr std::size_t kAlignment = blaze::AlignmentOf_v<Type> > alignof(owner_type)
                                                ? blaze::AlignmentOf_v<Type>
                                                : alignof(owner_type);

  static_assert(kAlignment % blaze::AlignmentOf_v<Type> == 0,
                "blocks must satisfy blaze's SIMD alignment requirement");

  /// @brief Distance from the base of the allocation to the block handed out. Being a multiple of
  /// kAlignment it keeps the returned pointer aligned, while leaving room for the header.
  static constexpr std::size_t kOffset = kAlignment;

  static_assert(kOffset >= sizeof(owner_type), "header must fit between the base and the block");

  /// @brief Total bytes backing n objects, header included. Recomputed identically on release so
  /// the resource always sees the size/alignment pair it was given.
  static constexpr auto bytes(std::size_t n) noexcept -> std::size_t {
    return kOffset + (n * sizeof(Type));
  }
};

}  // namespace vortex::math

#endif  // VORTEX_FOUNDATION_MATH_MATH_MEMORY_HPP
