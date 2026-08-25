/// ===============================================================================================
/// @file
///
/// @brief vortex.helpers.handle component
/// ===============================================================================================
#ifndef VORTEX_HELPERS_HANDLE_HPP
#define VORTEX_HELPERS_HANDLE_HPP
#include <functional>
#include <memory>
#include <memory_resource>
#include <optional>
#include <tuple>
#include <type_traits>

namespace vortex::helpers {

/// @brief A lightweight, non-null, shared-ownership handle to an object, with
/// custom memory resource support.
/// @tparam T The type of object to manage.
template <class T>
class handle {
  /// @brief Type alias for the allocator type used with the handle.
  using allocator_type = std::pmr::polymorphic_allocator<T>;

 public:
  /// @brief Constructs a handle managing a new instance of T.
  /// @param memory The memory resource to use for allocation.
  template <class... Args>
  explicit handle(std::pmr::memory_resource* const memory, Args&&... args)
      : ptr_{std::allocate_shared<T>(allocator_type{memory}, std::forward<Args>(args)...)} {}

  /// @brief Copy and move constructors and assignment operators.
  handle(const handle&) = default;
  handle(handle&&) noexcept = default;
  ~handle() = default;
  auto operator=(const handle&) -> handle& = default;
  auto operator=(handle&&) noexcept -> handle& = default;

  /// @brief Dereference operator for the managed object.
  [[nodiscard]]
  auto operator*() const noexcept -> T& {
    return *ptr_;
  }

  /// @brief Member-access operator for the managed object.
  [[nodiscard]]
  auto operator->() const noexcept -> T* {
    return ptr_.get();
  }

  /// @brief Comparison operators, based on the identity of the managed object.
  friend auto operator<(const handle& lhs, const handle& rhs) -> bool {
    return lhs.ptr_ < rhs.ptr_;
  }
  friend auto operator==(const handle& lhs, const handle& rhs) -> bool {
    return lhs.ptr_ == rhs.ptr_;
  }

 private:
  std::shared_ptr<T> ptr_;
};

/// @brief Checks equality between two handles of possibly different types.
/// @param a First handle.
/// @param b Second handle.
/// @return True if both handles manage the same object, false otherwise.
template <class T1, class T2>
inline auto equal(const handle<T1>& a, const handle<T2>& b) -> bool {
  if constexpr (std::is_same_v<T1, T2>) {
    return a == b;
  } else {
    std::ignore = a;
    std::ignore = b;
    return false;
  }
}

/// @brief An optional `handle<T>`, representing the possible absence of a
/// handle. Internally backed by `std::optional<handle<T>>`.
/// @tparam T The type of object managed by the wrapped handle.
template <class T>
class option : public std::optional<handle<T>> {
  using base = std::optional<handle<T>>;

 public:
  using base::base;
  using base::operator=;

  /// @brief Exposes `T*` directly, instead of `handle<T>*`.
  [[nodiscard]]
  auto operator->() const noexcept -> T* {
    return (**this).operator->();
  }

  /// @brief Equality, based on the identity of the managed object (if any).
  friend auto operator==(const option& lhs, const option& rhs) -> bool {
    return static_cast<const base&>(lhs) == static_cast<const base&>(rhs);
  }
};

}  // namespace vortex::helpers

// NOLINTBEGIN(cert-dcl58-cpp)
namespace std {

template <class T>
struct hash<vortex::helpers::handle<T>> {
  [[nodiscard]]
  auto operator()(const vortex::helpers::handle<T>& value) const noexcept -> size_t {
    return std::hash<const T*>{}(value.operator->());
  }
};

}  // namespace std
// NOLINTEND(cert-dcl58-cpp)

#endif  // VORTEX_HELPERS_HANDLE_HPP
