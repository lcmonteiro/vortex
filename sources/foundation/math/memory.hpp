#pragma once

#include <cstddef>
#include <cstring>
#include <memory_resource>

#include "helpers/memory.hpp"

namespace vortex::math {

namespace detail {

using owner_type = std::pmr::memory_resource*;

constexpr auto width_of(std::size_t alignment) noexcept -> std::size_t {
  return alignment < alignof(owner_type) ? alignof(owner_type) : alignment;
}

constexpr auto offset_of(std::size_t width) noexcept -> std::size_t {
  return ((sizeof(owner_type) + width - 1U) / width) * width;
}

}  // namespace detail

inline auto scoped_allocate(std::size_t bytes, std::size_t alignment) -> void* {
  const auto width = detail::width_of(alignment);
  const auto offset = detail::offset_of(width);

  auto* const resource = helpers::memory_scope::get_resource();
  auto* const base = static_cast<std::byte*>(resource->allocate(offset + bytes, width));
  auto* const block = base + offset;

  std::memcpy(block - sizeof(detail::owner_type), &resource, sizeof(resource));
  return block;
}

inline auto scoped_deallocate(void* p, std::size_t bytes, std::size_t alignment) noexcept -> void {
  if (p == nullptr) {
    return;
  }

  const auto width = detail::width_of(alignment);
  const auto offset = detail::offset_of(width);

  auto* const block = static_cast<std::byte*>(p);
  auto owner = detail::owner_type{};

  std::memcpy(&owner, block - sizeof(owner), sizeof(owner));

  owner->deallocate(block - offset, offset + bytes, width);
}

}  // namespace vortex::math
