/// ===============================================================================================
/// @file
/// @brief Tests for how `optimization::edge` extracts per-node Jacobian blocks out of the
/// combined dual residual.
///
/// The residual for one edge is evaluated once with every connected node seeded into its own
/// slice of a shared tangent space. Reading node I's block back means taking the partials in
/// [O, O + D) of each residual component -- and a component that does not depend on the whole
/// tangent space carries a shorter derivative vector than the space is wide.
/// ===============================================================================================
#include <gtest/gtest.h>

#include <cstddef>
#include <memory_resource>
#include <vector>

#include "foundation/math.hpp"
#include "optimization/graph.hpp"

namespace {

namespace go = vortex::optimization;
namespace math = vortex::math;

template <class Number>
struct Point {
  Number x{};
  Number y{};
};

struct NarrowEdge;
using Edges = go::edges<NarrowEdge>;

struct PointNode : go::node<PointNode, 2, Point<double>, Edges> {
  using Base = go::node<PointNode, 2, Point<double>, Edges>;
  using Base::Base;

  template <class Delta>
  auto plus(const Delta& delta) const {
    return Point{this->estimation().x + delta[0], this->estimation().y + delta[1]};
  }
};

/// @brief An edge whose first residual component depends only on the first node's first
/// coordinate.
///
/// `a.x` is seeded at derivative index 0, and a dual seeded at index i allocates i + 1
/// derivative slots, so that component's vector has length 1 -- shorter than the second node's
/// offset of 2. Nothing about this is exotic: a prior on one coordinate combined with a relative
/// term is an ordinary way to write a cost.
struct NarrowEdge : go::edge<NarrowEdge, 2, Point<double>, go::nodes<PointNode, PointNode>> {
  using Base = go::edge<NarrowEdge, 2, Point<double>, go::nodes<PointNode, PointNode>>;
  using Base::Base;

  template <class T>
  auto error(const Point<T>& a, const Point<T>& b) -> typename Base::template error_vector<T> {
    return {a.x - this->measurement().x, (b.y - a.y) - this->measurement().y};
  }
};

struct Graph : go::graph<go::nodes<PointNode>, Edges> {
  using Base = go::graph<go::nodes<PointNode>, Edges>;
  using Base::Base;
};

struct GraphEdgeJacobianTest : public ::testing::Test {
 protected:
  void SetUp() override {
    a_ = g_.build<PointNode>(Graph::key_type{1});
    b_ = g_.build<PointNode>(Graph::key_type{2});
    (*a_)->estimation(Point<double>{1.0, 2.0});
    (*b_)->estimation(Point<double>{3.0, 4.0});
    e_ = g_.build<NarrowEdge>(*a_, *b_);
    (*e_)->measurement(Point<double>{0.0, 0.0});
  }

  void TearDown() override { g_.destroy(); }

  Graph g_{std::pmr::new_delete_resource()};
  go::option<PointNode> a_;
  go::option<PointNode> b_;
  go::option<NarrowEdge> e_;
};

/// @brief error = [a.x, b.y - a.y] at a = (1,2), b = (3,4) gives [1, 2], so chi2 = 1 + 4.
TEST_F(GraphEdgeJacobianTest, GivenANarrowResidualComponent_ExpectTheChi2ToBeCorrect) {
  e_->update();
  EXPECT_DOUBLE_EQ(e_->chi2(), 5.0);
}

/// @brief Regression test: a residual component shorter than a node's tangent offset must
/// contribute a zero row to that node's block, not whatever lies past the end of its derivative
/// vector.
///
/// With the identity information matrix, foreach_b_block hands out J_I^T * error. Node b's
/// Jacobian is [[0, 0], [0, 1]] because component 0 does not mention b at all, so its gradient
/// block is [[0,0],[0,1]] * [1,2] = [0, 2]. A stale read past the end of component 0's partials
/// lands in b's first row and shows up as a non-zero leading entry here.
TEST_F(GraphEdgeJacobianTest, GivenANarrowResidualComponent_ExpectZeroGradientForTheAbsentNode) {
  e_->update();

  std::vector<std::pair<std::size_t, math::static_vector<double, 2>>> blocks;
  e_->foreach_b_block([&blocks](const auto& node, const auto& block) {
    blocks.emplace_back(node->key(), math::static_vector<double, 2>(block));
  });

  ASSERT_EQ(blocks.size(), 2U);

  // Node a: J_a = [[1, 0], [0, -1]], so J_a^T * [1, 2] = [1, -2].
  EXPECT_EQ(blocks[0].first, 1U);
  EXPECT_DOUBLE_EQ(blocks[0].second[0], 1.0);
  EXPECT_DOUBLE_EQ(blocks[0].second[1], -2.0);

  // Node b: component 0 is independent of b, so the leading entry must be exactly zero.
  EXPECT_EQ(blocks[1].first, 2U);
  EXPECT_DOUBLE_EQ(blocks[1].second[0], 0.0);
  EXPECT_DOUBLE_EQ(blocks[1].second[1], 2.0);
}

}  // namespace
