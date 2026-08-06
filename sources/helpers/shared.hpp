/// ===========================================================================
/// @file
///
/// @brief vortex.helpers.shared component
/// ===========================================================================
#ifndef VORTEX_HELPERS_SHARED_HPP
#define VORTEX_HELPERS_SHARED_HPP
#include <functional>
#include <memory>
#include <memory_resource>
#include <tuple>

namespace vortex::helpers {

/// @brief A lightweight shared object implementation with custom memory
/// resource support.
/// @tparam Type The type of object to manage.
template <typename Type>
class shared {
  /// @brief Type alias for the allocator type used with the shared object.
  using allocator_type = std::pmr::polymorphic_allocator<Type>;

 public:
  /// @brief Type alias to allow access to the Type being pointed to,
  /// similarly to std::shared_ptr::element_type.
  using element_type = Type;

  /// @brief Constructs a shared object managing a new instance of Type.
  /// @param memory The memory resource to use for allocation.
  template <class... Args>
  explicit shared(std::pmr::memory_resource* const memory, Args&&... args)
      : ptr_{std::allocate_shared<Type>(allocator_type{memory}, std::forward<Args>(args)...)} {}

  /// @brief Copy and move constructors and assignment operators.
  shared(const shared& other) = default;
  shared(shared&& other) = default;
  ~shared() = default;
  auto operator=(shared&& other) -> shared& = default;
  auto operator=(const shared& other) -> shared& = default;

  /// @brief Accessor for the managed object.
  auto get() const -> Type* { return ptr_.get(); }
  auto value() const -> Type& { return *ptr_; }

  /// @brief Dereference operators for the managed object.
  auto operator*() const -> Type& { return *ptr_; }
  auto operator->() const -> Type* { return ptr_.get(); }

  /// @brief Comparison operators for shared objects.
  friend auto operator<(const shared& lhs, const shared& rhs) -> bool {
    return lhs.ptr_ < rhs.ptr_;
  }
  friend auto operator==(const shared& lhs, const shared& rhs) -> bool {
    return lhs.ptr_ == rhs.ptr_;
  }

 private:
  std::shared_ptr<Type> ptr_;
};

/// @brief Checks equality between two shared objects of possibly different
/// types.
/// @param a First shared object.
/// @param b Second shared object.
/// @return True if both shared objects manage the same object, false otherwise.
template <class T1, class T2>
inline auto equal(const shared<T1>& a, const shared<T2>& b) -> bool {
  if constexpr (std::is_same_v<T1, T2>) {
    return a == b;
  } else {
    std::ignore = a;
    std::ignore = b;
    return false;
  }
}

}  // namespace vortex::helpers

// NOLINTBEGIN(cert-dcl58-cpp)
namespace std {
template <class Type>
struct hash<vortex::helpers::shared<Type>> {
  auto operator()(const vortex::helpers::shared<Type>& shared) const {
    return std::hash<Type*>()(shared.get());
  }
};

}  // namespace std
// NOLINTEND(cert-dcl58-cpp)

#endif  // VORTEX_HELPERS_SHARED_HPP
