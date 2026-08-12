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
/// forbidden. Per-thread, so a guard constrains only the thread that installed it.
thread_local std::size_t g_alloc_forbidden = 0;

/// @brief Set once an allocation has been rejected, and cleared only when the outermost guard
/// leaves scope.
///
/// Rejecting throws, and the throw unwinds. Destructors run during that unwind, and one of them
/// allocating would throw a second exception while the first is still in flight -- which is
/// `std::terminate`, not a test failure. So the first rejection stands down every later one until
/// the stack has fully unwound past all the guards. Tracking this separately from the depth is
/// what makes that work when guards nest: an inner guard's destructor restores the enclosing
/// depth, which would otherwise re-arm the outer guard part-way through the unwind.
thread_local bool g_alloc_rejected = false;

/// @brief Rejects the allocation if a guard is active and nothing has been rejected yet.
auto check() -> void {
  if (0 != g_alloc_forbidden && not g_alloc_rejected) {
    g_alloc_rejected = true;
    throw vortex::test::unexpected_allocation{};
  }
}

/// @brief Allocates, rounding a zero-sized request up to one byte.
///
/// `::operator new(0)` has to hand back a non-null pointer distinct from every other live object,
/// but `malloc(0)` is allowed to return null. Without the bump, a zero-sized `new` on such a libc
/// would trip the null check below and report an allocation failure that never happened.
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

memory_guard::memory_guard() noexcept : previous_depth_{g_alloc_forbidden} {
  g_alloc_forbidden = previous_depth_ + 1;
}

memory_guard::~memory_guard() noexcept {
  // Restored rather than decremented: a rejection may have cleared the depth, and decrementing
  // from there would wrap.
  g_alloc_forbidden = previous_depth_;
  if (0 == g_alloc_forbidden) {
    g_alloc_rejected = false;
  }
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
