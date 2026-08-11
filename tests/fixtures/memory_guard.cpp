/// ===============================================================================================
/// @file
/// @brief Replacements for the global allocation functions, counting for `memory_guard`.
///
/// These are the standard replaceable operators, so defining them here redirects every `new` and
/// `delete` in the test binary -- including the ones inside libstdc++, which is what lets a guard
/// observe `std::pmr::new_delete_resource` and the standard containers.
/// ===============================================================================================
#include "tests/fixtures/memory_guard.hpp"

#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <new>

namespace {

// Constant-initialised, so these are already usable by allocations made during static
// initialisation, before any dynamic initialiser has run.
std::atomic<std::size_t> g_allocations{0};
std::atomic<std::size_t> g_deallocations{0};

auto count_allocation() noexcept -> void {
  g_allocations.fetch_add(1, std::memory_order_relaxed);
}

auto count_deallocation() noexcept -> void {
  g_deallocations.fetch_add(1, std::memory_order_relaxed);
}

/// @brief Zero-sized requests must still return distinct pointers, hence the bump to 1.
auto allocate(std::size_t bytes) -> void* {
  auto* const p = std::malloc(0 == bytes ? 1 : bytes);
  if (nullptr == p) {
    throw std::bad_alloc{};
  }
  count_allocation();
  return p;
}

auto allocate(std::size_t bytes, std::size_t alignment) -> void* {
  // posix_memalign requires an alignment that is both a power of two and a multiple of
  // sizeof(void*); the over-aligned new path can ask for less than the latter.
  const auto width = alignment < sizeof(void*) ? sizeof(void*) : alignment;
  void* p = nullptr;
  if (0 != ::posix_memalign(&p, width, 0 == bytes ? 1 : bytes)) {
    throw std::bad_alloc{};
  }
  count_allocation();
  return p;
}

auto release(void* p) noexcept -> void {
  // Deleting a null pointer is well-defined and allocates nothing, so it is not counted.
  if (nullptr != p) {
    count_deallocation();
    std::free(p);
  }
}

}  // namespace

namespace vortex::test {

auto heap_allocation_count() noexcept -> std::size_t {
  return g_allocations.load(std::memory_order_relaxed);
}

auto heap_deallocation_count() noexcept -> std::size_t {
  return g_deallocations.load(std::memory_order_relaxed);
}

}  // namespace vortex::test

/// ===============================================================================================
/// Allocation
/// ===============================================================================================
auto operator new(std::size_t bytes) -> void* { return allocate(bytes); }
auto operator new[](std::size_t bytes) -> void* { return allocate(bytes); }

auto operator new(std::size_t bytes, std::align_val_t alignment) -> void* {
  return allocate(bytes, static_cast<std::size_t>(alignment));
}
auto operator new[](std::size_t bytes, std::align_val_t alignment) -> void* {
  return allocate(bytes, static_cast<std::size_t>(alignment));
}

auto operator new(std::size_t bytes, const std::nothrow_t& /*tag*/) noexcept -> void* {
  try {
    return allocate(bytes);
  } catch (...) {
    return nullptr;
  }
}
auto operator new[](std::size_t bytes, const std::nothrow_t& /*tag*/) noexcept -> void* {
  try {
    return allocate(bytes);
  } catch (...) {
    return nullptr;
  }
}
auto operator new(std::size_t bytes, std::align_val_t alignment,
                  const std::nothrow_t& /*tag*/) noexcept -> void* {
  try {
    return allocate(bytes, static_cast<std::size_t>(alignment));
  } catch (...) {
    return nullptr;
  }
}
auto operator new[](std::size_t bytes, std::align_val_t alignment,
                    const std::nothrow_t& /*tag*/) noexcept -> void* {
  try {
    return allocate(bytes, static_cast<std::size_t>(alignment));
  } catch (...) {
    return nullptr;
  }
}

/// ===============================================================================================
/// Deallocation
///
/// Every form funnels to `free`: both `malloc` and `posix_memalign` hand back memory that `free`
/// releases, so the aligned and sized overloads need no special handling.
/// ===============================================================================================
auto operator delete(void* p) noexcept -> void { release(p); }
auto operator delete[](void* p) noexcept -> void { release(p); }
auto operator delete(void* p, std::size_t /*bytes*/) noexcept -> void { release(p); }
auto operator delete[](void* p, std::size_t /*bytes*/) noexcept -> void { release(p); }
auto operator delete(void* p, std::align_val_t /*alignment*/) noexcept -> void { release(p); }
auto operator delete[](void* p, std::align_val_t /*alignment*/) noexcept -> void { release(p); }
auto operator delete(void* p, std::size_t /*bytes*/, std::align_val_t /*alignment*/) noexcept
    -> void {
  release(p);
}
auto operator delete[](void* p, std::size_t /*bytes*/, std::align_val_t /*alignment*/) noexcept
    -> void {
  release(p);
}
auto operator delete(void* p, const std::nothrow_t& /*tag*/) noexcept -> void { release(p); }
auto operator delete[](void* p, const std::nothrow_t& /*tag*/) noexcept -> void { release(p); }
auto operator delete(void* p, std::align_val_t /*alignment*/,
                     const std::nothrow_t& /*tag*/) noexcept -> void {
  release(p);
}
auto operator delete[](void* p, std::align_val_t /*alignment*/,
                       const std::nothrow_t& /*tag*/) noexcept -> void {
  release(p);
}
