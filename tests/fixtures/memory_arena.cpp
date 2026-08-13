/// ===============================================================================================
/// @file
/// @brief Replacements for the global allocation functions, enforcing `memory_arena::guard`.
///
/// Being the standard replaceable operators, defining them here redirects every `new` and `delete`
/// in the test binary, libstdc++'s included -- which is what lets a guard see through
/// `std::pmr::new_delete_resource` and the standard containers.
/// ===============================================================================================
#include "tests/fixtures/memory_arena.hpp"

#include <cstddef>
#include <cstdlib>
#include <new>

namespace {

/// @brief Whether a guard on this thread currently forbids allocation.
thread_local bool g_alloc_forbidden = false;

/// @brief Disarms before throwing: the throw unwinds, and a destructor allocating on the way out
/// would otherwise throw into an exception already in flight, which is `std::terminate`.
auto check() -> void {
  if (g_alloc_forbidden) {
    g_alloc_forbidden = false;
    throw vortex::test::memory_violation{"heap allocation inside a memory_arena::guard scope"};
  }
}

/// @brief Rounds a zero-sized request up to one byte: `::operator new(0)` must return a non-null
/// pointer distinct from every live object, while `malloc(0)` may return null.
auto allocate(std::size_t bytes) -> void* {
  check();
  auto* const p = std::malloc(0 == bytes ? 1 : bytes);
  if (nullptr == p) {
    throw std::bad_alloc{};
  }
  return p;
}

/// @brief As above. posix_memalign wants an alignment that is a multiple of sizeof(void*), which
/// the over-aligned new path can ask for less than.
auto allocate(std::size_t bytes, std::size_t alignment) -> void* {
  check();
  const auto width = alignment < sizeof(void*) ? sizeof(void*) : alignment;
  void* p = nullptr;
  if (0 != ::posix_memalign(&p, width, 0 == bytes ? 1 : bytes)) {
    throw std::bad_alloc{};
  }
  return p;
}

/// @brief Always permitted: a guard forbids taking memory, not returning it.
auto release(void* const p) noexcept -> void { std::free(p); }

}  // namespace

namespace vortex::test {

memory_arena::guard::guard() noexcept { g_alloc_forbidden = true; }

memory_arena::guard::~guard() noexcept { g_alloc_forbidden = false; }

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
/// Deallocation -- `free` releases both `malloc` and `posix_memalign` memory, so the sized and
/// aligned overloads need no special handling.
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
