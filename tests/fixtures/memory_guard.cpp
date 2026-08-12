/// ===============================================================================================
/// @file
/// @brief Replacements for the global allocation functions, enforcing `memory_guard`.
///
/// These are the standard replaceable operators, so defining them here redirects every `new` and
/// `delete` in the test binary -- including the ones inside libstdc++, which is what lets a guard
/// see through `std::pmr::new_delete_resource` and the standard containers.
/// ===============================================================================================
#include "tests/fixtures/memory_guard.hpp"

#include <cstddef>
#include <cstdlib>
#include <new>

namespace {

/// @brief Nesting depth of the active guards on this thread; non-zero means allocation is
/// forbidden. Per-thread so a guard constrains only the thread that installed it.
thread_local std::size_t g_forbidden_depth = 0;

/// @brief Rejects the allocation if a guard is active.
///
/// Arming is cleared before throwing. Unwinding runs destructors, and one of those attempting an
/// allocation would otherwise throw a second exception while the first is in flight, which ends in
/// `std::terminate` rather than a test failure. Each `memory_guard` restores the depth it saved
/// rather than decrementing, so clearing it here cannot corrupt an enclosing guard.
auto check() -> void {
  if (0 != g_forbidden_depth) {
    g_forbidden_depth = 0;
    throw vortex::test::unexpected_allocation{};
  }
}

/// @brief Zero-sized requests must still return distinct pointers, hence the bump to 1.
auto allocate(std::size_t bytes) -> void* {
  check();
  auto* const p = std::malloc(0 == bytes ? 1 : bytes);
  if (nullptr == p) {
    throw std::bad_alloc{};
  }
  return p;
}

auto allocate(std::size_t bytes, std::size_t alignment) -> void* {
  check();
  // posix_memalign wants an alignment that is a power of two *and* a multiple of sizeof(void*);
  // the over-aligned new path can ask for less than the latter.
  const auto width = alignment < sizeof(void*) ? sizeof(void*) : alignment;
  void* p = nullptr;
  if (0 != ::posix_memalign(&p, width, 0 == bytes ? 1 : bytes)) {
    throw std::bad_alloc{};
  }
  return p;
}

/// @brief Deallocation is always permitted: releasing memory is not what a guard forbids, and
/// blocking it would break the unwinding that a rejected allocation sets off.
auto release(void* const p) noexcept -> void { std::free(p); }

}  // namespace

namespace vortex::test {

memory_guard::memory_guard() noexcept : previous_{g_forbidden_depth} {
  g_forbidden_depth = previous_ + 1;
}

memory_guard::~memory_guard() noexcept { g_forbidden_depth = previous_; }

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
