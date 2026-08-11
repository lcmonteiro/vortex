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
/// @note Blaze's dynamic containers do not treat the allocator as part of the value: the move
/// constructor leaves `alloc_` default-constructed, move assignment keeps the destination's
/// `alloc_`, and `resize()`/`reserve()`/`swap()` exchange the element buffer via `std::swap`
/// while each object keeps its own allocator. A naive stateful allocator therefore ends up
/// releasing a buffer through a different `std::pmr::memory_resource` than the one that
/// produced it whenever such an operation straddles a `memory_scope` boundary.
///
/// To stay correct under those semantics, ownership is recorded with the buffer rather than in
/// the allocator: `allocate` stores the owning resource in a header placed immediately before
/// the returned block, and `deallocate` releases through that recorded resource. Deallocation is
/// consequently independent of which allocator instance performs it, which is also why every
/// instance compares equal (`is_always_equal`) -- any of them can free any block.
///
/// The captured resource still decides where *new* allocations come from, so a container grown
/// inside a `memory_scope` draws from that scope's arena as intended.
///
/// @warning A buffer taken from a scoped arena must be released before that arena is destroyed,
/// exactly as for any other arena-allocated object.
/// ================================================================================================
template <class Type>
class memory_scope_allocator {
 public:
  using value_type = Type;

  /// @brief Ownership travels with the buffer, so any instance can release any block.
  using is_always_equal = std::true_type;

  /// @brief Constructs the allocator, capturing the resource active on this thread right now.
  memory_scope_allocator() noexcept : resource_{helpers::memory_scope::get_resource()} {}

  /// @brief Rebind-style converting constructor, required by std::allocator_traits.
  /// @param other The allocator to copy the captured resource from.
  template <class Other>
  memory_scope_allocator(const memory_scope_allocator<Other>& other) noexcept  // NOLINT
      : resource_{other.resource()} {}

  /// @brief Allocates storage for n objects of Type from the captured resource.
  /// @param n Number of objects to allocate storage for.
  /// @return Pointer to the allocated (uninitialized) storage, aligned as blaze requires.
  [[nodiscard]]
  auto allocate(std::size_t n) -> Type* {
    auto* const base = static_cast<std::byte*>(resource_->allocate(bytes(n), kAlignment));
    auto* const block = base + kOffset;
    // Record the owning resource so `deallocate` can release the block through it, whichever
    // allocator instance happens to own the container by then.
    std::memcpy(block - sizeof(owner_type), &resource_, sizeof(owner_type));
    return reinterpret_cast<Type*>(block);
  }

  /// @brief Deallocates storage previously obtained from `allocate`, using the resource recorded
  /// with the block rather than the one captured by this allocator.
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

  /// @brief Returns the memory resource captured by this allocator, i.e. the one new allocations
  /// are taken from. It is not necessarily the resource that owns an already allocated block.
  [[nodiscard]]
  auto resource() const noexcept -> std::pmr::memory_resource* {
    return resource_;
  }

  /// @brief Every allocator can release every block, so all instances are interchangeable.
  friend auto operator==(const memory_scope_allocator&, const memory_scope_allocator&) noexcept
      -> bool {
    return true;
  }

 private:
  using owner_type = std::pmr::memory_resource*;

  /// @brief Blaze verifies the returned address against `AlignmentOf_v`, and the header must be
  /// stored at a suitably aligned address, so honour whichever requirement is stricter.
  static constexpr std::size_t kAlignment = blaze::AlignmentOf_v<Type> > alignof(owner_type)
                                                ? blaze::AlignmentOf_v<Type>
                                                : alignof(owner_type);

  /// @brief Distance from the base of the allocation to the block handed out. Being a multiple of
  /// kAlignment it keeps the returned pointer aligned, while leaving room for the header.
  static constexpr std::size_t kOffset = kAlignment;

  static_assert(kOffset >= sizeof(owner_type), "header must fit between the base and the block");

  /// @brief Total bytes backing n objects, header included. Recomputed identically on release so
  /// the resource always sees the size/alignment pair it was given.
  static constexpr auto bytes(std::size_t n) noexcept -> std::size_t {
    return kOffset + (n * sizeof(Type));
  }

  std::pmr::memory_resource* resource_;
};

}  // namespace vortex::math

#endif  // VORTEX_FOUNDATION_MATH_MATH_MEMORY_HPP
