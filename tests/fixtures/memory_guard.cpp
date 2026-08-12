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

/// @brief Guard nesting depth on this thread; non-zero forbids allocation.
thread_local std::size_t g_alloc_forbidden = 0;

/// @brief Set on the first rejection, cleared when the outermost guard exits. It suppresses later
/// rejections, because an allocation during unwinding would otherwise throw into an exception
/// already in flight -- `std::terminate`, not a test failure. Kept apart from the depth because an
/// inner guard's destructor reinstates the enclosing depth, re-arming mid-unwind.
thread_local bool g_alloc_rejected = false;

auto check() -> void {
  if (0 != g_alloc_forbidden && not g_alloc_rejected) {
    g_alloc_rejected = true;
    throw vortex::test::unexpected_allocation{};
  }
}

/// @brief Allocates, rounding a zero-sized request up to one byte: `::operator new(0)` must return
/// a non-null pointer distinct from every live object, while `malloc(0)` may return null and would
/// trip the check below as a failure that never happened.
auto allocate(std::size_t bytes) -> void* {
  check();
  auto* const p = std::malloc(0 == bytes ? 1 : bytes);
  if (nullptr == p) {
    throw std::bad_alloc{};
  }
  return p;
}

/// @brief As above. posix_memalign wants an alignment that is a power of two *and* a multiple of
/// sizeof(void*); the over-aligned new path can ask for less than the latter.
auto allocate(std::size_t bytes, std::size_t alignment) -> void* {
  check();
  const auto width = alignment < sizeof(void*) ? sizeof(void*) : alignment;
  void* p = nullptr;
  if (0 != ::posix_memalign(&p, width, 0 == bytes ? 1 : bytes)) {
    throw std::bad_alloc{};
  }
  return p;
}

/// @brief Always permitted: a guard forbids taking memory, not returning it, and blocking this
/// would break the unwinding a rejection sets off.
auto release(void* const p) noexcept -> void { std::free(p); }

}  // namespace

namespace vortex::test {

memory_guard::memory_guard() noexcept : previous_depth_{g_alloc_forbidden} {
  g_alloc_forbidden = previous_depth_ + 1;
}

memory_guard::~memory_guard() noexcept {
  // Restored, not decremented: a rejection may have cleared the depth, and decrementing would wrap.
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
/// Deallocation -- every form funnels to `free`, which releases both `malloc` and
/// `posix_memalign` memory, so the sized and aligned overloads need no special handling.
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
