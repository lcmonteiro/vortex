#pragma once
#include <cmath>

#include "sources/dual/operations/base.hpp"

namespace g2o_dual::dual {
struct erf : unary_operation<erf> {
  template <class T>
  auto value(const T& v) const {
    return std::erf(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    return two_over_sqrt_pi<T> * std::exp(-n.v * n.v) * n.d;
  }

 private:
  template <class T>
  static constexpr T two_over_sqrt_pi = T{1.12837916709551257389615890312154517L};
};

struct erfc : unary_operation<erfc> {
  template <class T>
  auto value(const T& v) const {
    return std::erfc(v);
  }
  template <class T>
  auto dvalue(const duo<T>& n) const {
    return -two_over_sqrt_pi<T> * std::exp(-n.v * n.v) * n.d;
  }

 private:
  template <class T>
  static constexpr T two_over_sqrt_pi = T{1.12837916709551257389615890312154517L};
};

}  // namespace g2o_dual::dual

namespace std {
template <class T, g2o_dual::dual::erf::enable_t<T> = 0>
inline auto erf(const T& n) {
  return std::invoke(g2o_dual::dual::erf{}, n);
}
template <class T, g2o_dual::dual::erfc::enable_t<T> = 0>
inline auto erfc(const T& n) {
  return std::invoke(g2o_dual::dual::erfc{}, n);
}
}  // namespace std
