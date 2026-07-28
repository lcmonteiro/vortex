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
class Shared {
 public:
  /// @brief Type alias to allow access to the Type being pointed to,
  /// similarly to std::shared_ptr::element_type.
  using element_type = Type;

  /// @brief Constructors
  template <class... Args>
  explicit Shared(std::pmr::memory_resource* const memory, Args&&... args)
      : ptr_{std::allocate_shared<Type>(std::pmr::polymorphic_allocator<Type>{memory},
                                        std::forward<Args>(args)...)} {}
  Shared(const Shared& other) = default;
  auto operator=(const Shared& other) -> Shared& = default;
  Shared(Shared&& other) = default;
  auto operator=(Shared&& other) -> Shared& = default;
  ~Shared() = default;

  /// @brief Accessor for the managed object.
  auto get() const -> Type* { return ptr_.get(); }
  auto value() const -> Type& { return *ptr_; }

  /// @brief Operators.
  auto operator*() const -> Type& { return *ptr_; }
  auto operator->() const -> Type* { return ptr_.get(); }

  friend auto operator<(const Shared& lhs, const Shared& rhs) -> bool { return lhs.ptr_ < rhs.ptr_; }
  friend auto operator==(const Shared& lhs, const Shared& rhs) -> bool { return lhs.ptr_ == rhs.ptr_; }

 private:
  std::shared_ptr<Type> ptr_;
};

/// @brief Checks equality between two Shared objects of possibly different
/// types.
/// @param a First Shared object.
/// @param b Second Shared object.
/// @return True if both Shared objects manage the same object, false otherwise.
template <class T1, class T2>
inline auto equal(const Shared<T1>& a, const Shared<T2>& b) -> bool {
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
struct hash<vortex::helpers::Shared<Type>> {
  auto operator()(const vortex::helpers::Shared<Type>& shared) const {
    return std::hash<Type*>()(shared.get());
  }
};

}  // namespace std
// NOLINTEND(cert-dcl58-cpp)

#endif  // VORTEX_HELPERS_SHARED_HPP
