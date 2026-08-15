#pragma once

#include "helpers/memory.hpp"

namespace vortex::math {

namespace detail {

/// @brief Returns an alignment suitable for both the requested storage and the allocation prefix.
constexpr auto align_of(std::size_t alignment) noexcept -> std::size_t {
  return alignment < alignof(std::max_align_t) ? alignof(std::max_align_t) : alignment;
}

}  // namespace detail

/// @brief Allocates aligned storage from the currently active memory scope.
/// @param bytes Number of bytes to allocate for the caller.
/// @param alignment Required alignment of the returned storage.
/// @return Pointer to aligned, uninitialized storage.
inline auto scoped_allocate(std::size_t bytes, std::size_t alignment) -> void* {
  const auto align = detail::align_of(alignment);
  auto* const resource = helpers::memory_scope::get_resource();
  auto* const base = static_cast<std::byte*>(resource->allocate(align + bytes, align));
  return base + align;
}

/// @brief Deallocates storage previously allocated by `scoped_allocate`.
/// @param p Pointer previously returned by `scoped_allocate`.
/// @param bytes Number of bytes originally allocated.
/// @param alignment Alignment originally requested.
inline auto scoped_deallocate(void* p, std::size_t bytes, std::size_t alignment) noexcept -> void {
  const auto align = detail::align_of(alignment);
  auto* const resource = helpers::memory_scope::get_resource();
  resource->deallocate(static_cast<std::byte*>(p) - align, align + bytes, align);
}

}  // namespace vortex::math
