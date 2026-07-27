/// ===========================================================================
/// @file
/// @brief Unit tests for the dual-number forward-mode AD engine.
/// ===========================================================================
#include <gtest/gtest.h>

#include <cmath>

#include "sources/dual/dual.hpp"

namespace {

using vortex::dual::number;
using Dual = number<double>;

TEST(DualNumber, ProductRule) {
  const auto x = Dual{3.0, 0};
  const auto y = Dual{4.0, 1};
  const auto z = x * y;  // z = 12, dz/dx = y = 4, dz/dy = x = 3
  EXPECT_DOUBLE_EQ(z.value(), 12.0);
  EXPECT_DOUBLE_EQ(z.dvalue(0), 4.0);
  EXPECT_DOUBLE_EQ(z.dvalue(1), 3.0);
}

TEST(DualNumber, QuotientRule) {
  const auto x = Dual{6.0, 0};
  const auto y = Dual{2.0, 1};
  const auto z = x / y;  // z = 3, dz/dx = 1/y = 0.5, dz/dy = -x/y^2 = -1.5
  EXPECT_DOUBLE_EQ(z.value(), 3.0);
  EXPECT_DOUBLE_EQ(z.dvalue(0), 0.5);
  EXPECT_DOUBLE_EQ(z.dvalue(1), -1.5);
}

TEST(DualNumber, Trigonometric) {
  const auto x = Dual{0.5, 0};
  const auto s = std::sin(x);
  EXPECT_NEAR(s.value(), std::sin(0.5), 1e-12);
  EXPECT_NEAR(s.dvalue(0), std::cos(0.5), 1e-12);

  const auto c = std::cos(x);
  EXPECT_NEAR(c.value(), std::cos(0.5), 1e-12);
  EXPECT_NEAR(c.dvalue(0), -std::sin(0.5), 1e-12);
}

TEST(DualNumber, Atan2) {
  const auto y = Dual{1.0, 0};
  const auto x = Dual{2.0, 1};
  const auto a = std::atan2(y, x);
  const double denom = 1.0 * 1.0 + 2.0 * 2.0;
  EXPECT_NEAR(a.value(), std::atan2(1.0, 2.0), 1e-12);
  EXPECT_NEAR(a.dvalue(0), 2.0 / denom, 1e-12);   // d/dy =  x / (x^2 + y^2)
  EXPECT_NEAR(a.dvalue(1), -1.0 / denom, 1e-12);  // d/dx = -y / (x^2 + y^2)
}

TEST(DualNumber, ExpSqrtChainRule) {
  const auto x = Dual{2.0, 0};
  const auto f = std::sqrt(std::exp(x));  // e^{x/2}, df/dx = 0.5 e^{x/2}
  EXPECT_NEAR(f.value(), std::exp(1.0), 1e-12);
  EXPECT_NEAR(f.dvalue(0), 0.5 * std::exp(1.0), 1e-12);
}

// Verifies the AD Jacobian matches a central-difference reference for a
// nonlinear vector-valued function of two variables.
TEST(DualNumber, JacobianMatchesNumeric) {
  const auto cost = [](const auto& p) {
    // f0 = sin(x) * y ; f1 = x^2 + cos(y)
    using std::cos;
    using std::sin;
    return std::array{sin(p[0]) * p[1], p[0] * p[0] + cos(p[1])};
  };

  const double x0 = 0.7;
  const double y0 = -1.3;

  const std::array<Dual, 2> dual_input{Dual{x0, 0}, Dual{y0, 1}};
  const auto dual_out = cost(dual_input);

  const double eps = 1e-6;
  const std::array<double, 2> base{x0, y0};
  for (int col = 0; col < 2; ++col) {
    auto plus = base;
    auto minus = base;
    plus[col] += eps;
    minus[col] -= eps;
    const auto fp = cost(plus);
    const auto fm = cost(minus);
    for (int row = 0; row < 2; ++row) {
      const double numeric = (fp[row] - fm[row]) / (2 * eps);
      EXPECT_NEAR(dual_out[row].dvalue(col), numeric, 1e-6) << "row=" << row << " col=" << col;
    }
  }
}

}  // namespace
