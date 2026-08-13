/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.base component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_BASE_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_BASE_HPP

#include <tuple>
#include <utility>

#include "foundation/dual/number.hpp"

namespace vortex::dual {

/// @brief Value/derivative pair for a single active index of a dual number.
/// @tparam T Underlying scalar type.
template <class T>
struct duo {
  duo(const number<T>& n, std::size_t i) : v{n.value()}, d{n.dvalue(i)} {}
  const T& v;
  const T& d;
};

/// @brief CRTP base for unary dual-number operations.
///
/// The derived type provides `value` and `dvalue`; this base evaluates the
/// operation on the scalar value and propagates the operand's derivatives via
/// the chain rule.
/// @tparam Derived The concrete unary operation type.
template <class Derived>
struct unary_operation {
  template <class T>
  static constexpr auto enable = is_number_v<T>;

  template <class T>
  auto operator()(const number<T>& n) const {
    return number<T>{value(n), n.dindex(), dvalue(n)};
  }

 protected:
  auto self() const noexcept { return static_cast<const Derived*>(this); }

  template <class T>
  auto value(const number<T>& n) const {
    return self()->value(n.value());
  }

  template <class T>
  auto dvalue(const number<T>& n) const {
    using dvalue_t = typename number<T>::dvalue_t;
    auto dv = dvalue_t(std::size(n), T{0}, memory());
    for (const auto i : n.dindex()) {
      dv[i] = self()->dvalue(duo(n, i));
    }
    return dv;
  }
};

/// @brief CRTP base for binary dual-number operations.
///
/// The derived type provides `value` and `dvalue` overloads; this base merges
/// the active derivative indices of the two operands and propagates their
/// derivatives via the chain rule. Overloads accept a dual number combined
/// with either another dual number or a plain scalar.
/// @tparam Derived The concrete binary operation type.
template <class Derived>
struct binary_operation {
  template <class T1, class T2>
  static constexpr auto enable = (                   //
      (is_number_v<T1> and is_number_like_v<T2>) or  //
      (is_number_v<T2> and is_number_like_v<T1>)     //
  );

  template <class T>
  auto operator()(const number<T>& n1, const T& v2) const {
    const auto value = this->value(n1.value(), v2);
    auto [dindex, dvalue] = this->dvalues(n1, v2);
    return number<T>{value, std::move(dindex), std::move(dvalue)};
  }

  template <class T>
  auto operator()(const T& v1, const number<T>& n2) const {
    const auto value = this->value(v1, n2.value());
    auto [dindex, dvalue] = this->dvalues(v1, n2);
    return number<T>{value, std::move(dindex), std::move(dvalue)};
  }

  template <class T>
  auto operator()(const number<T>& n1, const number<T>& n2) const {
    const auto value = this->value(n1.value(), n2.value());
    auto [dindex, dvalue] = this->dvalues(n1, n2);
    return number<T>{value, std::move(dindex), std::move(dvalue)};
  }

 protected:
  auto self() const noexcept { return static_cast<const Derived*>(this); }

  template <class T>
  auto value(const T& v1, const T& v2) const {
    return self()->value(v1, v2);
  }

  template <class T>
  auto dvalues(const number<T>& n, const T& v) const {
    using dindex_t = typename number<T>::dindex_t;
    using dvalue_t = typename number<T>::dvalue_t;
    auto dv = dvalue_t(std::size(n), T{0}, memory());
    for (auto i : n.dindex()) {
      dv[i] = self()->dvalue(duo(n, i), v);
    }
    return std::tuple{dindex_t{n.dindex(), memory()}, std::move(dv)};
  }

  template <class T>
  auto dvalues(const T& v, const number<T>& n) const {
    using dindex_t = typename number<T>::dindex_t;
    using dvalue_t = typename number<T>::dvalue_t;
    auto dv = dvalue_t(std::size(n), T{0}, memory());
    for (auto i : n.dindex()) {
      dv[i] = self()->dvalue(v, duo(n, i));
    }
    return std::tuple{dindex_t{n.dindex(), memory()}, std::move(dv)};
  }

  template <class T>
  auto dvalues(const number<T>& n1, const number<T>& n2) const {
    using dindex_t = typename number<T>::dindex_t;
    using dvalue_t = typename number<T>::dvalue_t;
    const auto ds = std::max(std::size(n1), std::size(n2));
    auto dv = dvalue_t(ds, T{0}, memory());
    auto di = dindex_t(memory());
    di.reserve(ds);
    merge_index(
        n1.dindex(), n2.dindex(),
        [&](auto i) {
          dv[i] = self()->dvalue(duo(n1, i), n2.value());
          di.emplace_back(i);
        },
        [&](auto i) {
          dv[i] = self()->dvalue(n1.value(), duo(n2, i));
          di.emplace_back(i);
        },
        [&](auto i) {
          dv[i] = self()->dvalue(duo(n1, i), duo(n2, i));
          di.emplace_back(i);
        });
    return std::tuple{std::move(di), std::move(dv)};
  }

 private:
  template <class I, class OnI1, class OnI2, class OnIx>
  auto merge_index(const I& i1, const I& i2, OnI1 on_i1, OnI2 on_i2, OnIx on_ix) const -> void {
    auto it1 = std::cbegin(i1);
    auto it2 = std::cbegin(i2);
    auto end1 = std::cend(i1);
    auto end2 = std::cend(i2);
    while (it1 != end1 && it2 != end2) {
      const auto& v1 = *it1;
      const auto& v2 = *it2;
      if (v1 < v2) {
        on_i1(v1), ++it1;
      } else if (v2 < v1) {
        on_i2(v2), ++it2;
      } else {
        on_ix(v1), ++it1, ++it2;
      }
    }
    while (it1 != end1) {
      on_i1(*it1++);
    }
    while (it2 != end2) {
      on_i2(*it2++);
    }
  }
};
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_BASE_HPP
