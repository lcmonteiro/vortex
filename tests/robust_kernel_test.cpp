/// ===============================================================================================
/// @file
/// @brief Unit tests for the edge robust kernel variants (null, Huber) in
/// `optimization::variants::kernel_variant`, exercised both directly and
/// through an edge that selects them via `kernel_option`.
/// ===============================================================================================
#include "optimization/variants/robust_kernel.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <memory_resource>

#include "foundation/math.hpp"
#include "optimization/graph.hpp"

namespace {

namespace go = vortex::optimization;
namespace math = vortex::math;

using go::variants::huber_kernel_option;
using go::variants::null_kernel_option;

using Matrix = math::static_matrix<double, 2, 2>;

/// @brief Stand-in information matrix; the kernels only ever scale it.
const auto kInformation = Matrix{{1.0, 0.0}, {0.0, 1.0}};

/// @brief Recovers the scalar weight a kernel applies to the information matrix.
template <class Kernel>
auto weight_of(Kernel& kernel) -> double {
  const Matrix robustified = kernel.robustify(kInformation);
  return robustified(0, 0);
}

/// ===============================================================================================
/// Null kernel
/// ===============================================================================================

TEST(NullKernel, GivenChi2_ExpectItPassesThroughUnchanged) {
  // Given
  auto kernel = null_kernel_option<double>{};

  // When
  kernel.update(7.5);

  // Then
  EXPECT_DOUBLE_EQ(kernel.chi2(), 7.5);
}

TEST(NullKernel, GivenInformation_ExpectRobustifyReturnsTheSameObject) {
  // Given
  auto kernel = null_kernel_option<double>{};
  kernel.update(9.0);

  // When
  const auto& robustified = kernel.robustify(kInformation);

  // Then: the null kernel is the identity, not merely a value-equal copy.
  EXPECT_EQ(&robustified, &kInformation);
}

/// ===============================================================================================
/// Huber kernel
/// ===============================================================================================

TEST(HuberKernel, GivenDefaultConstruction_ExpectDeltaOfOne) {
  // Given / When
  auto kernel = huber_kernel_option<double>{};

  // Then
  EXPECT_DOUBLE_EQ(kernel.delta(), 1.0);
}

TEST(HuberKernel, GivenChi2BelowDeltaSquared_ExpectQuadraticRegion) {
  // Given
  auto kernel = huber_kernel_option<double>{};
  kernel.delta(2.0);  // threshold at chi2 == 4

  // When
  kernel.update(1.0);

  // Then: inside the quadratic region chi2 is untouched and nothing is down-weighted.
  EXPECT_DOUBLE_EQ(kernel.chi2(), 1.0);
  EXPECT_DOUBLE_EQ(weight_of(kernel), 1.0);
}

TEST(HuberKernel, GivenChi2ExactlyAtDeltaSquared_ExpectQuadraticRegion) {
  // Given
  auto kernel = huber_kernel_option<double>{};
  kernel.delta(2.0);

  // When: update() branches on `chi2 <= delta_sqr_`, so the boundary belongs to the
  // quadratic side.
  kernel.update(4.0);

  // Then
  EXPECT_DOUBLE_EQ(kernel.chi2(), 4.0);
  EXPECT_DOUBLE_EQ(weight_of(kernel), 1.0);
}

TEST(HuberKernel, GivenChi2AboveDeltaSquared_ExpectLinearRegion) {
  // Given
  auto kernel = huber_kernel_option<double>{};
  kernel.delta(1.0);

  // When
  kernel.update(4.0);

  // Then: rho = 2*sqrt(chi2)*delta - delta^2 = 2*2*1 - 1, weight = delta/sqrt(chi2) = 1/2.
  EXPECT_DOUBLE_EQ(kernel.chi2(), 3.0);
  EXPECT_DOUBLE_EQ(weight_of(kernel), 0.5);
}

TEST(HuberKernel, GivenGrowingChi2_ExpectRhoGrowsSublinearlyAndWeightDecays) {
  // Given
  auto small = huber_kernel_option<double>{};
  auto large = huber_kernel_option<double>{};

  // When
  small.update(4.0);
  large.update(400.0);

  // Then: a hundredfold jump in raw chi2 must not produce a hundredfold jump in cost,
  // which is the whole point of the kernel.
  EXPECT_LT(large.chi2(), 100.0 * small.chi2());
  EXPECT_LT(weight_of(large), weight_of(small));
}

TEST(HuberKernel, GivenRegionBoundary_ExpectRhoAndWeightAreContinuous) {
  // Given
  auto below = huber_kernel_option<double>{};
  auto above = huber_kernel_option<double>{};
  below.delta(2.0);
  above.delta(2.0);

  // When: straddle the threshold at chi2 == delta^2 == 4.
  below.update(4.0);
  above.update(4.0 + 1e-9);

  // Then: the two branches must agree in the limit, or the cost surface has a step in it.
  EXPECT_NEAR(below.chi2(), above.chi2(), 1e-6);
  EXPECT_NEAR(weight_of(below), weight_of(above), 1e-6);
}

/// @brief Regression test for the default-state bug fixed in 7354640.
///
/// `delta_` carried an in-class initializer of 1 while `delta_sqr_` carried none, so a
/// default-constructed kernel reported delta() == 1 against a squared threshold of 0. That
/// emptied the quadratic region: every residual took the outlier branch, and update(0.25)
/// yielded chi2() == 1 instead of 0.25. Setting delta to the value it already reported
/// changed the result, which is exactly what this pins down.
TEST(HuberKernel, GivenDefaultDelta_ExpectSameResultAsExplicitlySettingIt) {
  // Given
  auto defaulted = huber_kernel_option<double>{};
  auto explicitly_set = huber_kernel_option<double>{};
  explicitly_set.delta(1.0);

  ASSERT_DOUBLE_EQ(defaulted.delta(), explicitly_set.delta());

  // When
  defaulted.update(0.25);
  explicitly_set.update(0.25);

  // Then
  EXPECT_DOUBLE_EQ(defaulted.chi2(), explicitly_set.chi2());
  EXPECT_DOUBLE_EQ(weight_of(defaulted), weight_of(explicitly_set));

  // And the shared value is the quadratic-region one, not the outlier-branch one.
  EXPECT_DOUBLE_EQ(defaulted.chi2(), 0.25);
  EXPECT_DOUBLE_EQ(weight_of(defaulted), 1.0);
}

TEST(HuberKernel, GivenLargerDelta_ExpectTheQuadraticRegionWidens) {
  // Given
  auto narrow = huber_kernel_option<double>{};
  auto wide = huber_kernel_option<double>{};
  narrow.delta(1.0);
  wide.delta(3.0);

  // When: chi2 == 4 sits outside delta == 1 but inside delta == 3.
  narrow.update(4.0);
  wide.update(4.0);

  // Then
  EXPECT_DOUBLE_EQ(narrow.chi2(), 3.0);
  EXPECT_DOUBLE_EQ(weight_of(narrow), 0.5);
  EXPECT_DOUBLE_EQ(wide.chi2(), 4.0);
  EXPECT_DOUBLE_EQ(weight_of(wide), 1.0);
}

/// @brief Characterization test, not an endorsement.
///
/// Both kernels are queryable before their first update(). The Huber kernel seeds rho_ to 1
/// while the null kernel's chi2_ has no in-class initializer and value-initializes to 0, so
/// graph::chi2() on a graph read before its first update_edges() starts at one per Huber edge
/// rather than zero. If that default is ever changed, this test is the one that should fail.
TEST(RobustKernel, GivenNoUpdateYet_ExpectHuberReportsOneAndNullReportsZero) {
  // Given / When
  auto huber = huber_kernel_option<double>{};
  auto null = null_kernel_option<double>{};

  // Then
  EXPECT_DOUBLE_EQ(huber.chi2(), 1.0);
  EXPECT_DOUBLE_EQ(null.chi2(), 0.0);
}

/// ===============================================================================================
/// Kernel selection through an edge
/// ===============================================================================================

/// @brief Scalar-generic 2D point, used as both estimation and measurement type.
template <class Number>
struct Point {
  Number x{};
  Number y{};
};

struct NullKernelEdge;
struct HuberKernelEdge;
using Edges = go::edges<NullKernelEdge, HuberKernelEdge>;

/// @brief Minimal 2D node with the identity retraction.
struct PointNode : go::node<PointNode, 2, Point<double>, Edges> {
  using Base = go::node<PointNode, 2, Point<double>, Edges>;
  using Base::Base;

  template <class Delta>
  auto plus(const Delta& delta) const {
    return Point{this->estimation().x + delta[0], this->estimation().y + delta[1]};
  }
};

/// @brief Absolute-position residual under the default (null) kernel.
struct NullKernelEdge : go::edge<NullKernelEdge, 2, Point<double>, go::nodes<PointNode>> {
  using Base = go::edge<NullKernelEdge, 2, Point<double>, go::nodes<PointNode>>;
  using Base::Base;

  static constexpr std::size_t kernel_option = go::variants::null_kernel;

  template <class T>
  auto error(const Point<T>& a) -> typename Base::template error_vector<T> {
    return {a.x - this->measurement().x, a.y - this->measurement().y};
  }
};

/// @brief The same residual, selecting the Huber kernel instead.
struct HuberKernelEdge : go::edge<HuberKernelEdge, 2, Point<double>, go::nodes<PointNode>> {
  using Base = go::edge<HuberKernelEdge, 2, Point<double>, go::nodes<PointNode>>;
  using Base::Base;

  static constexpr std::size_t kernel_option = go::variants::huber_kernel;

  template <class T>
  auto error(const Point<T>& a) -> typename Base::template error_vector<T> {
    return {a.x - this->measurement().x, a.y - this->measurement().y};
  }
};

struct Graph : go::graph<go::nodes<PointNode>, Edges> {
  using Base = go::graph<go::nodes<PointNode>, Edges>;
  using Base::Base;
};

struct RobustKernelEdgeTestFixture : public ::testing::Test {
 protected:
  void TearDown() override { g_.destroy(); }

  /// @brief Builds a node whose estimate sits (3, 4) away from the measurement, so the raw
  /// squared error is 3^2 + 4^2 == 25 under the default identity information matrix.
  auto BuildNode() {
    auto node = g_.build<PointNode>(Graph::key_type{1});
    node->estimation(Point<double>{3.0, 4.0});
    return node;
  }

  static constexpr auto kRawChi2 = 25.0;

  Graph g_{std::pmr::new_delete_resource()};
};

TEST_F(RobustKernelEdgeTestFixture, GivenNullKernelEdge_ExpectChi2IsTheRawSquaredError) {
  // Given
  auto node = BuildNode();
  auto edge = g_.build<NullKernelEdge>(node);
  edge->measurement(Point<double>{0.0, 0.0});

  // When
  edge->update();

  // Then
  EXPECT_DOUBLE_EQ(edge->chi2(), kRawChi2);
}

TEST_F(RobustKernelEdgeTestFixture, GivenHuberKernelEdge_ExpectLargeResidualIsDownWeighted) {
  // Given
  auto node = BuildNode();
  auto edge = g_.build<HuberKernelEdge>(node);
  edge->measurement(Point<double>{0.0, 0.0});

  // When: default delta == 1, so chi2 == 25 lands well outside the quadratic region.
  edge->update();

  // Then: rho = 2*sqrt(25)*1 - 1 == 9, materially below the raw 25.
  EXPECT_DOUBLE_EQ(edge->chi2(), 9.0);
  EXPECT_LT(edge->chi2(), kRawChi2);
}

TEST_F(RobustKernelEdgeTestFixture, GivenDeltaWiderThanTheResidual_ExpectHuberMatchesNullKernel) {
  // Given
  auto node = BuildNode();
  auto edge = g_.build<HuberKernelEdge>(node);
  edge->measurement(Point<double>{0.0, 0.0});

  // When: delta == 10 puts chi2 == 25 back inside the quadratic region.
  edge->kernel.delta(10.0);
  edge->update();

  // Then: a Huber kernel that never leaves its quadratic region is the null kernel.
  EXPECT_DOUBLE_EQ(edge->chi2(), kRawChi2);
}

TEST_F(RobustKernelEdgeTestFixture, GivenKernelAccessor_ExpectDeltaRoundTrips) {
  // Given
  auto node = BuildNode();
  auto edge = g_.build<HuberKernelEdge>(node);

  // When
  edge->kernel.delta(2.5);

  // Then
  EXPECT_DOUBLE_EQ(edge->kernel.delta(), 2.5);
}

}  // namespace
