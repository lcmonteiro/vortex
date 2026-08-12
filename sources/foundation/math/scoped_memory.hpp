/// ===============================================================================================
/// @file
///
/// @brief vortex.math.scoped_memory component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_MATH_SCOPED_MEMORY_HPP
#define VORTEX_FOUNDATION_MATH_SCOPED_MEMORY_HPP

#include <cstddef>

namespace vortex::math {

/// ===============================================================================================
/// @brief Byte-level allocation from the active `helpers::memory_scope`, recording the owning
/// resource with the block so it is released through the same one.
///
/// Deliberately free of any blaze include: the patched `blaze::AlignedAllocator` calls these, and
/// a blaze header pulling a vortex header that pulls blaze back would be an include cycle.
/// ===============================================================================================

/// @param bytes Payload size.
/// @param alignment Required alignment of the returned pointer; raised to hold the header if the
/// caller asks for less than a pointer's alignment.
/// @return Storage aligned as requested, never null (allocation failure throws).
[[nodiscard]] auto scoped_allocate(std::size_t bytes, std::size_t alignment) -> void*;

/// @param p Pointer from `scoped_allocate`, or null.
/// @param bytes Payload size given to `scoped_allocate`.
/// @param alignment Alignment given to `scoped_allocate`.
auto scoped_deallocate(void* p, std::size_t bytes, std::size_t alignment) noexcept -> void;

}  // namespace vortex::math

#endif  // VORTEX_FOUNDATION_MATH_SCOPED_MEMORY_HPP
