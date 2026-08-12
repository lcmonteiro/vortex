/// ===============================================================================================
/// @file
///
/// @brief vortex.math.scoped_memory component
/// ===============================================================================================
#include "foundation/math/scoped_memory.hpp"

#include <cstddef>
#include <cstring>
#include <iterator>
#include <memory_resource>

#include "helpers/memory.hpp"

namespace {

using owner_type = std::pmr::memory_resource*;

/// @brief The header sits immediately before the block, so the block must be at least
/// pointer-aligned even when the caller asks for less.
constexpr auto width_of(std::size_t alignment) noexcept -> std::size_t {
  return alignment < alignof(owner_type) ? alignof(owner_type) : alignment;
}

/// @brief Smallest multiple of the width leaving room for the header, so the block stays aligned.
constexpr auto offset_of(std::size_t width) noexcept -> std::size_t {
  return ((sizeof(owner_type) + width - 1U) / width) * width;
}

}  // namespace

namespace vortex::math {

auto scoped_allocate(std::size_t bytes, std::size_t alignment) -> void* {
  const auto width = width_of(alignment);
  const auto offset = offset_of(width);

  auto* const resource = helpers::memory_scope::get_resource();
  auto* const base = static_cast<std::byte*>(resource->allocate(offset + bytes, width));
  auto* const block = std::next(base, static_cast<std::ptrdiff_t>(offset));

  std::memcpy(std::prev(block, sizeof(owner_type)), &resource, sizeof(owner_type));
  return block;
}

auto scoped_deallocate(void* p, std::size_t bytes, std::size_t alignment) noexcept -> void {
  if (nullptr == p) {
    return;
  }
  const auto width = width_of(alignment);
  const auto offset = offset_of(width);

  auto* const block = static_cast<std::byte*>(p);
  auto owner = owner_type{};
  std::memcpy(&owner, std::prev(block, sizeof(owner_type)), sizeof(owner_type));

  owner->deallocate(std::prev(block, static_cast<std::ptrdiff_t>(offset)), offset + bytes, width);
}

}  // namespace vortex::math
