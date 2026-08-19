/// ===============================================================================================
/// @file
/// @brief Unit tests for the dual-number forward-mode AD engine.
/// ===============================================================================================
#include "foundation/dual.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <numbers>

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

/// ===============================================================================================
/// Unary rules
///
/// Each rule is checked on both its value and its derivative. Where a rule is smooth the
/// derivative is cross-checked against a central difference; where it has a kink or a domain
/// edge the expected value is spelled out, since finite differences cannot see either.
/// ===============================================================================================

/// @brief Central-difference derivative of a real-valued function, for cross-checking.
template <class Fn>
auto NumericDerivative(Fn&& fn, double x, double eps = 1e-6) -> double {
  return (fn(x + eps) - fn(x - eps)) / (2 * eps);
}

TEST(DualOperations, Negate) {
  const auto x = Dual{3.0, 0};
  const auto y = -x;
  EXPECT_DOUBLE_EQ(y.value(), -3.0);
  EXPECT_DOUBLE_EQ(y.dvalue(0), -1.0);
}

TEST(DualOperations, AbsAwayFromTheKink) {
  const auto positive = std::abs(Dual{2.0, 0});
  EXPECT_DOUBLE_EQ(positive.value(), 2.0);
  EXPECT_DOUBLE_EQ(positive.dvalue(0), 1.0);

  const auto negative = std::abs(Dual{-2.0, 0});
  EXPECT_DOUBLE_EQ(negative.value(), 2.0);
  EXPECT_DOUBLE_EQ(negative.dvalue(0), -1.0);
}

TEST(DualOperations, AbsAtTheKinkPicksZeroSubgradient) {
  // abs() is not differentiable at 0; the implementation selects 0 from the subgradient
  // [-1, 1]. Pinned because a finite difference cannot distinguish the choices.
  const auto at_zero = std::abs(Dual{0.0, 0});
  EXPECT_DOUBLE_EQ(at_zero.value(), 0.0);
  EXPECT_DOUBLE_EQ(at_zero.dvalue(0), 0.0);
}

TEST(DualOperations, Log) {
  const auto y = std::log(Dual{2.0, 0});
  EXPECT_NEAR(y.value(), std::log(2.0), 1e-12);
  EXPECT_NEAR(y.dvalue(0), 0.5, 1e-12);  // d/dx log(x) = 1/x
  EXPECT_NEAR(y.dvalue(0), NumericDerivative([](double v) { return std::log(v); }, 2.0), 1e-6);
}

TEST(DualOperations, Log1p) {
  const auto y = std::log1p(Dual{0.5, 0});
  EXPECT_NEAR(y.value(), std::log1p(0.5), 1e-12);
  EXPECT_NEAR(y.dvalue(0), 1.0 / 1.5, 1e-12);  // d/dx log(1+x) = 1/(1+x)
  EXPECT_NEAR(y.dvalue(0), NumericDerivative([](double v) { return std::log1p(v); }, 0.5), 1e-6);
}

TEST(DualOperations, Erf) {
  constexpr auto x0 = 0.3;
  const auto y = std::erf(Dual{x0, 0});
  const auto expected = 2.0 / std::sqrt(std::numbers::pi) * std::exp(-x0 * x0);
  EXPECT_NEAR(y.value(), std::erf(x0), 1e-12);
  EXPECT_NEAR(y.dvalue(0), expected, 1e-12);
  EXPECT_NEAR(y.dvalue(0), NumericDerivative([](double v) { return std::erf(v); }, x0), 1e-6);
}

TEST(DualOperations, Erfc) {
  constexpr auto x0 = 0.3;
  const auto y = std::erfc(Dual{x0, 0});
  const auto expected = -2.0 / std::sqrt(std::numbers::pi) * std::exp(-x0 * x0);
  EXPECT_NEAR(y.value(), std::erfc(x0), 1e-12);
  EXPECT_NEAR(y.dvalue(0), expected, 1e-12);
  EXPECT_NEAR(y.dvalue(0), NumericDerivative([](double v) { return std::erfc(v); }, x0), 1e-6);
}

TEST(DualOperations, ErfAndErfcDerivativesAreOpposite) {
  // erf(x) + erfc(x) == 1, so their derivatives must cancel exactly.
  const auto x = Dual{0.7, 0};
  EXPECT_NEAR(std::erf(x).dvalue(0) + std::erfc(x).dvalue(0), 0.0, 1e-15);
}

/// ===============================================================================================
/// Binary rules
/// ===============================================================================================

TEST(DualOperations, MinRoutesTheDerivativeThroughTheSmallerOperand) {
  const auto x = Dual{1.0, 0};
  const auto y = Dual{2.0, 1};

  // Invoked directly rather than through std::min: for two duals the standard two-argument
  // std::min is the better overload and shadows this one. See StdMinOnTwoDuals... below.
  const auto m = std::invoke(vortex::dual::min{}, x, y);

  EXPECT_DOUBLE_EQ(m.value(), 1.0);
  EXPECT_EQ(m.size(), 2U);             // indices of both operands are merged in
  EXPECT_DOUBLE_EQ(m.dvalue(0), 1.0);  // flows from x
  EXPECT_DOUBLE_EQ(m.dvalue(1), 0.0);  // y is inactive
}

TEST(DualOperations, MaxRoutesTheDerivativeThroughTheLargerOperand) {
  const auto x = Dual{1.0, 0};
  const auto y = Dual{2.0, 1};
  const auto m = std::invoke(vortex::dual::max{}, x, y);
  EXPECT_DOUBLE_EQ(m.value(), 2.0);
  EXPECT_EQ(m.size(), 2U);
  EXPECT_DOUBLE_EQ(m.dvalue(0), 0.0);  // x is inactive
  EXPECT_DOUBLE_EQ(m.dvalue(1), 1.0);  // flows from y
}

TEST(DualOperations, MinMaxAgainstAScalarBound) {
  const auto x = Dual{2.0, 0};

  const auto clamped_high = std::min(x, 5.0);
  EXPECT_DOUBLE_EQ(clamped_high.value(), 2.0);
  EXPECT_DOUBLE_EQ(clamped_high.dvalue(0), 1.0);  // x is selected, so its derivative survives

  const auto clamped_low = std::min(1.0, x);
  EXPECT_DOUBLE_EQ(clamped_low.value(), 1.0);
  EXPECT_DOUBLE_EQ(clamped_low.dvalue(0), 0.0);  // the constant wins, derivative is cut

  const auto floored = std::max(x, 5.0);
  EXPECT_DOUBLE_EQ(floored.value(), 5.0);
  EXPECT_DOUBLE_EQ(floored.dvalue(0), 0.0);
}

TEST(DualOperations, MinMaxTiesResolveTowardsTheFirstOperand) {
  // Both comparisons are non-strict (<= and >=), so an exact tie keeps the left operand's
  // derivative. Worth pinning: the opposite convention is equally defensible.
  const auto x = Dual{1.0, 0};
  const auto y = Dual{1.0, 1};

  const auto smallest = std::invoke(vortex::dual::min{}, x, y);
  EXPECT_DOUBLE_EQ(smallest.dvalue(0), 1.0);
  EXPECT_DOUBLE_EQ(smallest.dvalue(1), 0.0);

  const auto largest = std::invoke(vortex::dual::max{}, x, y);
  EXPECT_DOUBLE_EQ(largest.dvalue(0), 1.0);
  EXPECT_DOUBLE_EQ(largest.dvalue(1), 0.0);
}

/// @brief Records that std::min/std::max do not reach the dual/dual overloads.
///
/// vortex::dual::min is `template <class T, class U> requires enable<T, U>`, but for two operands
/// of the same type the standard `template <class T> const T& min(const T&, const T&)` is the
/// better match, so it wins and returns a reference to one of the operands. The value and its
/// derivatives are still right, but the result is *narrower*: it carries only the winning
/// operand's indices, so reading a partial belonging to the loser runs off the end of the
/// derivative vector.
TEST(DualOperations, StdMinOnTwoDualsSelectsTheStandardOverload) {
  const auto x = Dual{1.0, 0};  // one active index
  const auto y = Dual{2.0, 1};  // two active indices

  const auto& selected = std::min(x, y);

  EXPECT_EQ(&selected, &x) << "std::min returned a new object, so the overload set changed";
  EXPECT_EQ(selected.size(), x.size());

  // The vortex operation would have merged both operands' indices instead.
  EXPECT_EQ(std::invoke(vortex::dual::min{}, x, y).size(), 2U);
  EXPECT_LT(selected.size(), std::invoke(vortex::dual::min{}, x, y).size());
}

TEST(DualOperations, PowWithDualBaseAndConstantExponent) {
  const auto x = Dual{2.0, 0};
  const auto y = std::pow(x, 3.0);  // x^3, dy/dx = 3x^2
  EXPECT_NEAR(y.value(), 8.0, 1e-12);
  EXPECT_NEAR(y.dvalue(0), 12.0, 1e-12);
  EXPECT_NEAR(y.dvalue(0), NumericDerivative([](double v) { return std::pow(v, 3.0); }, 2.0), 1e-5);
}

TEST(DualOperations, PowWithConstantBaseAndDualExponent) {
  const auto n = Dual{3.0, 0};
  const auto y = std::pow(2.0, n);  // 2^n, dy/dn = 2^n * ln(2)
  EXPECT_NEAR(y.value(), 8.0, 1e-12);
  EXPECT_NEAR(y.dvalue(0), 8.0 * std::log(2.0), 1e-12);
  EXPECT_NEAR(y.dvalue(0), NumericDerivative([](double v) { return std::pow(2.0, v); }, 3.0), 1e-5);
}

TEST(DualOperations, PowWithBothOperandsDual) {
  const auto base = Dual{2.0, 0};
  const auto exponent = Dual{3.0, 1};
  const auto y = std::pow(base, exponent);

  // Both partials at once: d/dbase = e*b^(e-1), d/dexponent = b^e * ln(b).
  EXPECT_NEAR(y.value(), 8.0, 1e-12);
  EXPECT_NEAR(y.dvalue(0), 12.0, 1e-12);
  EXPECT_NEAR(y.dvalue(1), 8.0 * std::log(2.0), 1e-12);
}

/// ===============================================================================================
/// Shared derivative indices
///
/// Every binary rule has three dvalue overloads: (dual, scalar), (scalar, dual) and
/// (dual, dual). The last one is only reached when both operands are active at the *same*
/// derivative index, which the tests above never produce because they seed each operand at an
/// index of its own. Composing both operands from one variable takes that path.
/// ===============================================================================================

TEST(DualSharedIndex, Plus) {
  const auto t = Dual{3.0, 0};
  const auto y = t + t;  // 2t
  EXPECT_DOUBLE_EQ(y.value(), 6.0);
  EXPECT_DOUBLE_EQ(y.dvalue(0), 2.0);
}

TEST(DualSharedIndex, Minus) {
  const auto t = Dual{3.0, 0};
  const auto y = t - t;  // identically 0, so the derivative cancels
  EXPECT_DOUBLE_EQ(y.value(), 0.0);
  EXPECT_DOUBLE_EQ(y.dvalue(0), 0.0);
}

TEST(DualSharedIndex, MultipliesAndDivides) {
  const auto t = Dual{3.0, 0};

  const auto square = t * t;  // t^2, d/dt = 2t
  EXPECT_DOUBLE_EQ(square.value(), 9.0);
  EXPECT_DOUBLE_EQ(square.dvalue(0), 6.0);

  const auto unity = t / t;  // identically 1
  EXPECT_DOUBLE_EQ(unity.value(), 1.0);
  EXPECT_NEAR(unity.dvalue(0), 0.0, 1e-15);
}

TEST(DualSharedIndex, Atan2) {
  const auto t = Dual{2.0, 0};
  const auto y = std::atan2(t, t);  // constant pi/4 along the diagonal
  EXPECT_NEAR(y.value(), std::numbers::pi / 4, 1e-12);
  EXPECT_NEAR(y.dvalue(0), 0.0, 1e-15);
}

TEST(DualSharedIndex, Pow) {
  const auto t = Dual{2.0, 0};
  const auto y = std::pow(t, t);  // t^t, d/dt = t^t (ln t + 1)
  EXPECT_NEAR(y.value(), 4.0, 1e-12);
  EXPECT_NEAR(y.dvalue(0), 4.0 * (std::log(2.0) + 1.0), 1e-12);
  EXPECT_NEAR(y.dvalue(0), NumericDerivative([](double v) { return std::pow(v, v); }, 2.0), 1e-5);
}

TEST(DualSharedIndex, MinMax) {
  const auto t = Dual{2.0, 0};
  const auto doubled = t * 2.0;  // active at the same index as t

  const auto smallest = std::invoke(vortex::dual::min{}, t, doubled);
  EXPECT_DOUBLE_EQ(smallest.value(), 2.0);
  EXPECT_DOUBLE_EQ(smallest.dvalue(0), 1.0);  // t wins, carrying d/dt = 1

  const auto largest = std::invoke(vortex::dual::max{}, t, doubled);
  EXPECT_DOUBLE_EQ(largest.value(), 4.0);
  EXPECT_DOUBLE_EQ(largest.dvalue(0), 2.0);  // 2t wins, carrying d/dt = 2
}

}  // namespace
