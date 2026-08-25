/// ===============================================================================================
/// @file
///
/// @brief vortex.dual.operations.base component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_DUAL_OPERATIONS_BASE_HPP
#define VORTEX_FOUNDATION_DUAL_OPERATIONS_BASE_HPP

#include "foundation/dual/number.hpp"
#include "helpers/contracts.hpp"

namespace vortex::dual {

/// @brief Value/derivative pair for a single active index of a dual number.
/// @tparam T Underlying scalar type.
template <class T>
struct duo {
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
    return number{value(n), dvalues(n)};
  }

 protected:
  auto self() const noexcept { return static_cast<const Derived*>(this); }

  template <class T>
  auto value(const number<T>& n) const {
    return self()->value(n.value());
  }

  template <class T>
  auto dvalues(const number<T>& n) const {
    using dvalues_t = typename number<T>::dvalues_t;
    auto out = dvalues_t(memory());
    out.reserve(std::size(n.dvalues()));
    for (const auto& [index, derivative] : n.dvalues()) {
      out.emplace_back(index, self()->dvalue(duo<T>{n.value(), derivative}));
    }
    return out;
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
    return number{this->value(n1.value(), v2), this->dvalues(n1, v2)};
  }

  template <class T>
  auto operator()(const T& v1, const number<T>& n2) const {
    return number{this->value(v1, n2.value()), this->dvalues(v1, n2)};
  }

  template <class T>
  auto operator()(const number<T>& n1, const number<T>& n2) const {
    return number{this->value(n1.value(), n2.value()), this->dvalues(n1, n2)};
  }

 protected:
  auto self() const noexcept { return static_cast<const Derived*>(this); }

  template <class T>
  auto value(const T& v1, const T& v2) const {
    return self()->value(v1, v2);
  }

  template <class T>
  auto dvalues(const number<T>& n, const T& v) const {
    using dvalues_t = typename number<T>::dvalues_t;
    auto out = dvalues_t(memory());
    out.reserve(std::size(n.dvalues()));
    for (const auto& [index, derivative] : n.dvalues()) {
      out.emplace_back(index, self()->dvalue(duo{n.value(), derivative}, v));
    }
    return out;
  }

  template <class T>
  auto dvalues(const T& v, const number<T>& n) const {
    using dvalues_t = typename number<T>::dvalues_t;
    auto out = dvalues_t(memory());
    out.reserve(std::size(n.dvalues()));
    for (const auto& [index, derivative] : n.dvalues()) {
      out.emplace_back(index, self()->dvalue(v, duo{n.value(), derivative}));
    }
    return out;
  }

  template <class T>
  auto dvalues(const number<T>& n1, const number<T>& n2) const {
    using dvalues_t = typename number<T>::dvalues_t;
    auto out = dvalues_t(memory());
    out.reserve(merged_capacity(n1.dvalues(), n2.dvalues()));
    merge_dvalues(
        n1.dvalues(), n2.dvalues(),
        [&](const auto& d1) {
          out.emplace_back(d1.index, self()->dvalue(duo{n1.value(), d1.value}, n2.value()));
        },
        [&](const auto& d2) {
          out.emplace_back(d2.index, self()->dvalue(n1.value(), duo{n2.value(), d2.value}));
        },
        [&](const auto& d1, const auto& d2) {
          out.emplace_back(  //
              d1.index, self()->dvalue(duo{n1.value(), d1.value}, duo{n2.value(), d2.value}));
        });
    return out;
  }

 private:
  template <class D>
  static auto merged_capacity(const D& d1, const D& d2) -> std::size_t {
    VORTEX_ASSERT(not d1.empty() and not d2.empty(), "derivative vectors must be non-empty");
    const auto lowest = std::min(d1.front().index, d2.front().index);
    const auto highest = std::max(d1.back().index, d2.back().index);
    return std::min(highest - lowest + 1, std::size(d1) + std::size(d2));
  }

  template <class D, class OnD1, class OnD2, class OnDx>
  auto merge_dvalues(const D& d1, const D& d2, OnD1 on_d1, OnD2 on_d2, OnDx on_dx) const -> void {
    auto it1 = std::cbegin(d1);
    auto it2 = std::cbegin(d2);
    auto end1 = std::cend(d1);
    auto end2 = std::cend(d2);
    while (it1 != end1 && it2 != end2) {
      if (it1->index < it2->index) {
        on_d1(*it1), ++it1;
      } else if (it2->index < it1->index) {
        on_d2(*it2), ++it2;
      } else {
        on_dx(*it1, *it2), ++it1, ++it2;
      }
    }
    while (it1 != end1) {
      on_d1(*it1++);
    }
    while (it2 != end2) {
      on_d2(*it2++);
    }
  }
};
}  // namespace vortex::dual

#endif  // VORTEX_FOUNDATION_DUAL_OPERATIONS_BASE_HPP
