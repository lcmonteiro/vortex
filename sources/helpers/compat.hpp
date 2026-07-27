/// ===========================================================================
/// @file
/// @brief vortex.helpers.compat component
///
/// Small standard-library based replacements for the utilities that were
/// previously provided by the internal `amp` library.
/// ===========================================================================
#ifndef VORTEX_HELPERS_COMPAT_HPP
#define VORTEX_HELPERS_COMPAT_HPP

#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>

/// @brief Runtime precondition check.
/// Replaces the former `VORTEX_PRECONDITION` macro. In debug builds it
/// behaves like an assertion; the message is kept for readability.
#define VORTEX_PRECONDITION(condition, message) assert((condition) && (message))

namespace vortex::graph {
namespace detail {

/// @brief Narrowing cast helper (replacement for `graph::detail::narrow_cast`).
/// @tparam To   Target type.
/// @tparam From Source type.
/// @param value The value to convert.
/// @return The value converted to the target type.
template <class To, class From>
constexpr To narrow_cast(From&& value) noexcept {
  return static_cast<To>(std::forward<From>(value));
}

/// @brief Compile-time integer power (replacement for `graph::detail::int_pow`).
/// @tparam Exponent The integer exponent.
/// @tparam T        The base type.
/// @param base The base value.
/// @return base raised to the given exponent.
template <unsigned Exponent, class T>
constexpr T int_pow(T base) {
  T result{1};
  for (unsigned i = 0; i < Exponent; ++i) {
    result *= base;
  }
  return result;
}

/// @brief Fixed-capacity FIFO history buffer.
/// Drop-in replacement for `graph::detail::CircularBuffer`. When the buffer reaches its
/// capacity a new push evicts the oldest element. Backed by a contiguous
/// std::vector so that iterator based erase / back access keep working.
/// @tparam Type     Element type.
/// @tparam Capacity Maximum number of retained elements.
template <class Type, std::size_t Capacity>
class CircularBuffer {
 public:
  using container_type = std::vector<Type>;
  using iterator = typename container_type::iterator;
  using const_iterator = typename container_type::const_iterator;

  CircularBuffer() { data_.reserve(Capacity + 1); }

  template <class... Args>
  Type& emplace_back(Args&&... args) {
    if (data_.size() >= Capacity) {
      data_.erase(data_.begin());
    }
    return data_.emplace_back(std::forward<Args>(args)...);
  }

  Type& back() { return data_.back(); }
  const Type& back() const { return data_.back(); }

  iterator begin() { return data_.begin(); }
  iterator end() { return data_.end(); }
  const_iterator begin() const { return data_.begin(); }
  const_iterator end() const { return data_.end(); }

  iterator erase(const_iterator first, const_iterator last) { return data_.erase(first, last); }

  std::size_t size() const { return data_.size(); }
  bool empty() const { return data_.empty(); }

 private:
  container_type data_;
};

}  // namespace detail
}  // namespace vortex::graph

#endif  // VORTEX_HELPERS_COMPAT_HPP
