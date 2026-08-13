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

namespace vortex::math {
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
