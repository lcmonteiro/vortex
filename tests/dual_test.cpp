/// ===============================================================================================
/// @file
/// @brief Unit tests for the dual-number forward-mode AD engine.
/// ===============================================================================================
#include "vortex/foundation/dual.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <numbers>
#include <tuple>
#include <type_traits>

namespace {

using vortex::dual::number;
using Dual = number<double>;

/// @brief Gets the dvalue derivative of a dual number with respect to a given index.
auto dvalue(const Dual& n, std::size_t index) -> double {
  const auto it = std::lower_bound(  //
      std::cbegin(n.dvalues()),      //
      std::cend(n.dvalues()),        //
      index,                         //
      [](const auto& d, std::size_t i) { return d.index < i; });
  return (it == std::cend(n.dvalues()) or it->index != index) ? 0.0 : it->value;
}

TEST(DualNumber, ProductRule) {
  const auto x = Dual{3.0, 0};
  const auto y = Dual{4.0, 1};
  const auto z = x * y;  // z = 12, dz/dx = y = 4, dz/dy = x = 3
  EXPECT_DOUBLE_EQ(z.value(), 12.0);
  EXPECT_DOUBLE_EQ(dvalue(z, 0), 4.0);
  EXPECT_DOUBLE_EQ(dvalue(z, 1), 3.0);
}

TEST(DualNumber, QuotientRule) {
  const auto x = Dual{6.0, 0};
  const auto y = Dual{2.0, 1};
  const auto z = x / y;  // z = 3, dz/dx = 1/y = 0.5, dz/dy = -x/y^2 = -1.5
  EXPECT_DOUBLE_EQ(z.value(), 3.0);
  EXPECT_DOUBLE_EQ(dvalue(z, 0), 0.5);
  EXPECT_DOUBLE_EQ(dvalue(z, 1), -1.5);
}

TEST(DualNumber, Trigonometric) {
  const auto x = Dual{0.5, 0};
  const auto s = sin(x);
  EXPECT_NEAR(s.value(), std::sin(0.5), 1e-12);
  EXPECT_NEAR(dvalue(s, 0), std::cos(0.5), 1e-12);

  const auto c = cos(x);
  EXPECT_NEAR(c.value(), std::cos(0.5), 1e-12);
  EXPECT_NEAR(dvalue(c, 0), -std::sin(0.5), 1e-12);
}

TEST(DualNumber, Atan2) {
  const auto y = Dual{1.0, 0};
  const auto x = Dual{2.0, 1};
  const auto a = atan2(y, x);
  const double denom = 1.0 * 1.0 + 2.0 * 2.0;
  EXPECT_NEAR(a.value(), std::atan2(1.0, 2.0), 1e-12);
  EXPECT_NEAR(dvalue(a, 0), 2.0 / denom, 1e-12);   // d/dy =  x / (x^2 + y^2)
  EXPECT_NEAR(dvalue(a, 1), -1.0 / denom, 1e-12);  // d/dx = -y / (x^2 + y^2)
}

TEST(DualNumber, ExpSqrtChainRule) {
  const auto x = Dual{2.0, 0};
  const auto f = sqrt(exp(x));  // e^{x/2}, df/dx = 0.5 e^{x/2}
  EXPECT_NEAR(f.value(), std::exp(1.0), 1e-12);
  EXPECT_NEAR(dvalue(f, 0), 0.5 * std::exp(1.0), 1e-12);
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
      EXPECT_NEAR(dvalue(dual_out[row], col), numeric, 1e-6) << "row=" << row << " col=" << col;
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
  EXPECT_DOUBLE_EQ(dvalue(y, 0), -1.0);
}

TEST(DualOperations, AbsAwayFromTheKink) {
  const auto positive = abs(Dual{2.0, 0});
  EXPECT_DOUBLE_EQ(positive.value(), 2.0);
  EXPECT_DOUBLE_EQ(dvalue(positive, 0), 1.0);

  const auto negative = abs(Dual{-2.0, 0});
  EXPECT_DOUBLE_EQ(negative.value(), 2.0);
  EXPECT_DOUBLE_EQ(dvalue(negative, 0), -1.0);
}

TEST(DualOperations, AbsAtTheKinkPicksZeroSubgradient) {
  // abs() is not differentiable at 0; the implementation selects 0 from the subgradient
  // [-1, 1]. Pinned because a finite difference cannot distinguish the choices.
  const auto at_zero = abs(Dual{0.0, 0});
  EXPECT_DOUBLE_EQ(at_zero.value(), 0.0);
  EXPECT_DOUBLE_EQ(dvalue(at_zero, 0), 0.0);
}

TEST(DualOperations, Log) {
  const auto y = log(Dual{2.0, 0});
  EXPECT_NEAR(y.value(), std::log(2.0), 1e-12);
  EXPECT_NEAR(dvalue(y, 0), 0.5, 1e-12);  // d/dx log(x) = 1/x
  EXPECT_NEAR(dvalue(y, 0), NumericDerivative([](double v) { return std::log(v); }, 2.0), 1e-6);
}

TEST(DualOperations, Log1p) {
  const auto y = log1p(Dual{0.5, 0});
  EXPECT_NEAR(y.value(), std::log1p(0.5), 1e-12);
  EXPECT_NEAR(dvalue(y, 0), 1.0 / 1.5, 1e-12);  // d/dx log(1+x) = 1/(1+x)
  EXPECT_NEAR(dvalue(y, 0), NumericDerivative([](double v) { return std::log1p(v); }, 0.5), 1e-6);
}

TEST(DualOperations, Erf) {
  constexpr auto x0 = 0.3;
  const auto y = erf(Dual{x0, 0});
  const auto expected = 2.0 / std::sqrt(std::numbers::pi) * std::exp(-x0 * x0);
  EXPECT_NEAR(y.value(), std::erf(x0), 1e-12);
  EXPECT_NEAR(dvalue(y, 0), expected, 1e-12);
  EXPECT_NEAR(dvalue(y, 0), NumericDerivative([](double v) { return std::erf(v); }, x0), 1e-6);
}

TEST(DualOperations, Erfc) {
  constexpr auto x0 = 0.3;
  const auto y = erfc(Dual{x0, 0});
  const auto expected = -2.0 / std::sqrt(std::numbers::pi) * std::exp(-x0 * x0);
  EXPECT_NEAR(y.value(), std::erfc(x0), 1e-12);
  EXPECT_NEAR(dvalue(y, 0), expected, 1e-12);
  EXPECT_NEAR(dvalue(y, 0), NumericDerivative([](double v) { return std::erfc(v); }, x0), 1e-6);
}

TEST(DualOperations, ErfAndErfcDerivativesAreOpposite) {
  // erf(x) + erfc(x) == 1, so their derivatives must cancel exactly.
  const auto x = Dual{0.7, 0};
  EXPECT_NEAR(dvalue(erf(x), 0) + dvalue(erfc(x), 0), 0.0, 1e-15);
}

/// ===============================================================================================
/// Binary rules
/// ===============================================================================================

TEST(DualOperations, MinRoutesTheDerivativeThroughTheSmallerOperand) {
  const auto x = Dual{1.0, 0};
  const auto y = Dual{2.0, 1};

  const auto m = vortex::dual::min(x, y);

  EXPECT_DOUBLE_EQ(m.value(), 1.0);
  EXPECT_EQ(m.size(), 2U);              // indices of both operands are merged in
  EXPECT_DOUBLE_EQ(dvalue(m, 0), 1.0);  // flows from x
  EXPECT_DOUBLE_EQ(dvalue(m, 1), 0.0);  // y is inactive
}

TEST(DualOperations, MaxRoutesTheDerivativeThroughTheLargerOperand) {
  const auto x = Dual{1.0, 0};
  const auto y = Dual{2.0, 1};
  const auto m = vortex::dual::max(x, y);
  EXPECT_DOUBLE_EQ(m.value(), 2.0);
  EXPECT_EQ(m.size(), 2U);
  EXPECT_DOUBLE_EQ(dvalue(m, 0), 0.0);  // x is inactive
  EXPECT_DOUBLE_EQ(dvalue(m, 1), 1.0);  // flows from y
}

TEST(DualOperations, MinMaxAgainstAScalarBound) {
  const auto x = Dual{2.0, 0};

  const auto clamped_high = min(x, 5.0);
  EXPECT_DOUBLE_EQ(clamped_high.value(), 2.0);
  EXPECT_DOUBLE_EQ(dvalue(clamped_high, 0), 1.0);  // x is selected, so its derivative survives

  const auto clamped_low = min(1.0, x);
  EXPECT_DOUBLE_EQ(clamped_low.value(), 1.0);
  EXPECT_DOUBLE_EQ(dvalue(clamped_low, 0), 0.0);  // the constant wins, derivative is cut

  const auto floored = max(x, 5.0);
  EXPECT_DOUBLE_EQ(floored.value(), 5.0);
  EXPECT_DOUBLE_EQ(dvalue(floored, 0), 0.0);
}

TEST(DualOperations, MinMaxTiesResolveTowardsTheFirstOperand) {
  // Both comparisons are non-strict (<= and >=), so an exact tie keeps the left operand's
  // derivative. Worth pinning: the opposite convention is equally defensible.
  const auto x = Dual{1.0, 0};
  const auto y = Dual{1.0, 1};

  const auto smallest = vortex::dual::min(x, y);
  EXPECT_DOUBLE_EQ(dvalue(smallest, 0), 1.0);
  EXPECT_DOUBLE_EQ(dvalue(smallest, 1), 0.0);

  const auto largest = vortex::dual::max(x, y);
  EXPECT_DOUBLE_EQ(dvalue(largest, 0), 1.0);
  EXPECT_DOUBLE_EQ(dvalue(largest, 1), 0.0);
}

/// @brief Generic code reaches the dual overloads, and only std::min itself does not.
///
/// The dual functions live in vortex::dual and are found by argument-dependent lookup, the same
/// way a cost function written once for double and for dual numbers calls them: `using std::min;
/// min(a, b)`. For two duals vortex::dual::min takes `number<T>` and is the more specialised
/// candidate, so it beats the standard `const T& min(const T&, const T&)` and merges both
/// operands' indices. Spelling `std::min` asks for the standard one, which compares by value and
/// returns a reference to the winner, carrying only that operand's indices.
TEST(DualOperations, MinOnTwoDualsFoundByLookupMergesBothOperands) {
  const auto x = Dual{1.0, 0};
  const auto y = Dual{2.0, 1};

  using std::min;
  const auto merged = min(x, y);
  EXPECT_EQ(merged.size(), 2U);
  EXPECT_DOUBLE_EQ(dvalue(merged, 0), 1.0);
  EXPECT_DOUBLE_EQ(dvalue(merged, 1), 0.0);

  const auto& selected = std::min(x, y);
  EXPECT_EQ(&selected, &x);
  EXPECT_EQ(selected.size(), 1U);
}

TEST(DualOperations, PowWithDualBaseAndConstantExponent) {
  const auto x = Dual{2.0, 0};
  const auto y = pow(x, 3.0);  // x^3, dy/dx = 3x^2
  EXPECT_NEAR(y.value(), 8.0, 1e-12);
  EXPECT_NEAR(dvalue(y, 0), 12.0, 1e-12);
  EXPECT_NEAR(dvalue(y, 0), NumericDerivative([](double v) { return std::pow(v, 3.0); }, 2.0),
              1e-5);
}

TEST(DualOperations, PowWithConstantBaseAndDualExponent) {
  const auto n = Dual{3.0, 0};
  const auto y = pow(2.0, n);  // 2^n, dy/dn = 2^n * ln(2)
  EXPECT_NEAR(y.value(), 8.0, 1e-12);
  EXPECT_NEAR(dvalue(y, 0), 8.0 * std::log(2.0), 1e-12);
  EXPECT_NEAR(dvalue(y, 0), NumericDerivative([](double v) { return std::pow(2.0, v); }, 3.0),
              1e-5);
}

TEST(DualOperations, PowWithBothOperandsDual) {
  const auto base = Dual{2.0, 0};
  const auto exponent = Dual{3.0, 1};
  const auto y = pow(base, exponent);

  // Both dvalues at once: d/dbase = e*b^(e-1), d/dexponent = b^e * ln(b).
  EXPECT_NEAR(y.value(), 8.0, 1e-12);
  EXPECT_NEAR(dvalue(y, 0), 12.0, 1e-12);
  EXPECT_NEAR(dvalue(y, 1), 8.0 * std::log(2.0), 1e-12);
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
  EXPECT_DOUBLE_EQ(dvalue(y, 0), 2.0);
}

TEST(DualSharedIndex, Minus) {
  const auto t = Dual{3.0, 0};
  const auto y = t - t;  // identically 0, so the derivative cancels
  EXPECT_DOUBLE_EQ(y.value(), 0.0);
  EXPECT_DOUBLE_EQ(dvalue(y, 0), 0.0);
}

TEST(DualSharedIndex, MultipliesAndDivides) {
  const auto t = Dual{3.0, 0};

  const auto square = t * t;  // t^2, d/dt = 2t
  EXPECT_DOUBLE_EQ(square.value(), 9.0);
  EXPECT_DOUBLE_EQ(dvalue(square, 0), 6.0);

  const auto unity = t / t;  // identically 1
  EXPECT_DOUBLE_EQ(unity.value(), 1.0);
  EXPECT_NEAR(dvalue(unity, 0), 0.0, 1e-15);
}

TEST(DualSharedIndex, Atan2) {
  const auto t = Dual{2.0, 0};
  const auto y = atan2(t, t);  // constant pi/4 along the diagonal
  EXPECT_NEAR(y.value(), std::numbers::pi / 4, 1e-12);
  EXPECT_NEAR(dvalue(y, 0), 0.0, 1e-15);
}

TEST(DualSharedIndex, Pow) {
  const auto t = Dual{2.0, 0};
  const auto y = pow(t, t);  // t^t, d/dt = t^t (ln t + 1)
  EXPECT_NEAR(y.value(), 4.0, 1e-12);
  EXPECT_NEAR(dvalue(y, 0), 4.0 * (std::log(2.0) + 1.0), 1e-12);
  EXPECT_NEAR(dvalue(y, 0), NumericDerivative([](double v) { return std::pow(v, v); }, 2.0), 1e-5);
}

TEST(DualSharedIndex, MinMax) {
  const auto t = Dual{2.0, 0};
  const auto doubled = t * 2.0;  // active at the same index as t

  const auto smallest = vortex::dual::min(t, doubled);
  EXPECT_DOUBLE_EQ(smallest.value(), 2.0);
  EXPECT_DOUBLE_EQ(dvalue(smallest, 0), 1.0);  // t wins, carrying d/dt = 1

  const auto largest = vortex::dual::max(t, doubled);
  EXPECT_DOUBLE_EQ(largest.value(), 4.0);
  EXPECT_DOUBLE_EQ(dvalue(largest, 0), 2.0);  // 2t wins, carrying d/dt = 2
}

/// ===============================================================================================
/// Contracts. A dual reaching an operation must carry at least one derivative, and the derivatives
/// it carries must be strictly ascending by index -- the ordering `dvalue`'s binary search and the
/// merge both rely on. Both are checked with assertions, so these only run where assertions do.
/// ===============================================================================================

TEST(DualContract, SeededVariableCarriesExactlyOneDerivative) {
  const auto x = Dual{3.0, 5};
  ASSERT_EQ(x.size(), 1U);
  EXPECT_EQ(x.dvalues().front().index, 5U);
  EXPECT_DOUBLE_EQ(x.dvalues().front().value, 1.0);
}

/// @brief The invariant is structural: no constructor can produce a dual without derivatives.
/// A seeded variable carries one, copies and moves carry what they were given, and the operations
/// build results from operands that already carry some -- so there is no path to an empty one.
/// A constant belongs on the scalar side, and the type no longer offers a way to spell it
/// otherwise.
static_assert(not std::is_constructible_v<Dual, double>,
              "a constant dual would carry no derivatives; keep constants scalar");
static_assert(std::is_constructible_v<Dual, double, std::size_t>,
              "a variable is a value seeded at a tangent index");
static_assert(std::is_copy_constructible_v<Dual> and std::is_move_constructible_v<Dual>);

TEST(DualContract, ScalarConstantsGiveTheSameResultAsSeededArithmetic) {
  const auto x = Dual{3.0, 0};

  const auto scaled = x * 2.0;
  EXPECT_DOUBLE_EQ(scaled.value(), 6.0);
  EXPECT_DOUBLE_EQ(dvalue(scaled, 0), 2.0);

  const auto shifted = 2.0 + x;
  EXPECT_DOUBLE_EQ(shifted.value(), 5.0);
  EXPECT_DOUBLE_EQ(dvalue(shifted, 0), 1.0);
}

/// ===============================================================================================
/// Operand types
/// ===============================================================================================

TEST(DualOperands, AnyArithmeticScalarCombinesWithANumber) {
  const auto x = Dual{3.0, 0};

  const auto doubled = x * 2;  // int
  EXPECT_DOUBLE_EQ(doubled.value(), 6.0);
  EXPECT_DOUBLE_EQ(dvalue(doubled, 0), 2.0);

  const auto halved = x / 2.0f;  // float
  EXPECT_DOUBLE_EQ(halved.value(), 1.5);
  EXPECT_DOUBLE_EQ(dvalue(halved, 0), 0.5);

  const auto shifted = 1u - x;  // unsigned, scalar on the left
  EXPECT_DOUBLE_EQ(shifted.value(), -2.0);
  EXPECT_DOUBLE_EQ(dvalue(shifted, 0), -1.0);

  const auto squared = pow(x, 2);
  EXPECT_DOUBLE_EQ(squared.value(), 9.0);
  EXPECT_DOUBLE_EQ(dvalue(squared, 0), 6.0);

  const auto clamped = max(x, 5);
  EXPECT_DOUBLE_EQ(clamped.value(), 5.0);
  EXPECT_DOUBLE_EQ(dvalue(clamped, 0), 0.0);
}

template <class A, class B>
concept multipliable = requires(const A& a, const B& b) { a* b; };
template <class A>
concept has_sin = requires(const A& a) { std::sin(a); };
template <class A>
concept has_tanh = requires(const A& a) { std::tanh(a); };

static_assert(multipliable<Dual, int> and multipliable<long, Dual> and multipliable<Dual, float>);
static_assert(not multipliable<Dual, bool>, "n * true is a mistake, not a multiplication by one");
static_assert(not multipliable<Dual, number<float>>, "mixing scalar types must be explicit");

/// The math overloads are not added to namespace std (undefined behaviour), and a number does not
/// convert to its scalar implicitly, so a std:: call cannot quietly evaluate on the value alone and
/// drop the derivatives: it does not compile.
static_assert(not has_sin<Dual> and not has_tanh<Dual>);
static_assert(not std::is_convertible_v<Dual, double>);
static_assert(std::is_constructible_v<double, Dual>, "the value is still reachable explicitly");

}  // namespace
