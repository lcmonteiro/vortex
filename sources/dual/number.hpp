#pragma once

#include <array>
#include <cassert>
#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

namespace g2o_dual::dual {

template <std::size_t N>
using make_sequence = std::make_index_sequence<N>;
template <std::size_t... I>
using sequence = std::index_sequence<I...>;

template <class T>
struct number {
  using index_t = std::size_t;
  using value_t = T;
  using dindex_t = std::vector<index_t>;
  using dvalue_t = std::vector<value_t>;

  static constexpr auto kMaxIndex = 100000;

  number() = default;
  number(const number&) = default;
  number(number&&) = default;

  explicit number(const value_t& value)
      : value_{value},  //
        dindex_{},      //
        dvalue_{} {}

  explicit number(const value_t& value, index_t index)
      : value_{value}, dindex_{index}, dvalue_(index + 1, value_t{0}) {
    assert(index < kMaxIndex);
    dvalue_.back() = value_t{1};
  }

  auto operator=(const number&) -> number<T>& = default;
  auto operator=(number&&) -> number<T>& = default;

  operator const value_t&() const { return value_; }

  auto value(const value_t& v) -> void { value_ = v; }

  auto value() const -> const value_t& { return value_; }

  auto dindex() const -> const dindex_t& { return dindex_; }

  auto dvalue() const -> const dvalue_t& { return dvalue_; }

  auto dvalue(index_t i) const -> const value_t& {
    assert(i < dvalue_.size());
    return dvalue_[i];
  }

  auto size() const -> std::size_t { return dvalue_.size(); }

 protected:
  number(const value_t& value,    //
         const dindex_t& dindex,  //
         const dvalue_t& dvalue)
      : value_{value}, dindex_{dindex}, dvalue_{dvalue} {}
  template <class Derived>
  friend struct unary_operation;
  template <class Derived>
  friend struct binary_operation;

 private:
  value_t value_{};
  dindex_t dindex_{};
  dvalue_t dvalue_{};
};

template <class T>
inline auto operator<(const number<T>& n1, const number<T>& n2) -> bool {
  return n1.value() < n2.value();
}
template <class T>
inline auto operator<(const number<T>& n1, const T& n2) -> bool {
  return n1.value() < n2;
}
template <class T>
inline auto operator<(const T& n1, const number<T>& n2) -> bool {
  return n1 < n2.value();
}

template <class U>
inline auto make_number(const U& init, std::size_t i = 0) {
  return number{init, i};
}

template <class U>
inline auto make_vector(std::size_t n, const U& value) {
  auto vector = std::vector<number<U>>{};
  vector.reserve(n);
  for (std::size_t idx = 0; idx < n; ++idx) {
    vector.emplace_back(value, idx);
  }
  return vector;
}

template <size_t N, class U, std::size_t... I>
inline auto make_array(const U& init, sequence<I...> = {}) {
  if constexpr (sizeof...(I) != N) {
    return make_array<N>(init, make_sequence<N>{});
  } else {
    return std::array{number{init, I}...};
  }
}
template <class U, size_t N>
inline auto make_array(const std::array<U, N>& container) {
  auto array = std::array<number<U>, N>{};
  for (std::size_t idx = 0; idx < N; ++idx) {
    array[idx] = number{container[idx], idx};
  }
  return array;
}

template <class T>
struct is_number : std::false_type {};
template <class T>
struct is_number<number<T>> : std::true_type {};
template <class T>
constexpr bool is_number_v = is_number<T>::value;

template <class T>
struct is_number_like : std::is_arithmetic<T> {};
template <class T>
struct is_number_like<number<T>> : std::true_type {};
template <class T>
constexpr bool is_number_like_v = is_number_like<T>::value;

}  // namespace g2o_dual::dual
