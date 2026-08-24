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
#include <utility>
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
struct SwitchingEdge;
struct PartialEdge;
using Edges = go::edges<NarrowEdge, SwitchingEdge, PartialEdge>;

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

/// @brief An edge whose first residual component stops depending on the second node once that
/// node's x goes negative -- a switching constraint, written the way one ordinarily is.
///
/// Its point is that the *structure* of the residual changes between updates, not just its
/// values. An edge whose dependencies are fixed can never show whether a Jacobian block is
/// rebuilt or merely written over.
struct SwitchingEdge : go::edge<SwitchingEdge, 2, Point<double>, go::nodes<PointNode, PointNode>> {
  using Base = go::edge<SwitchingEdge, 2, Point<double>, go::nodes<PointNode, PointNode>>;
  using Base::Base;

  template <class T>
  auto error(const Point<T>& a, const Point<T>& b) -> typename Base::template error_vector<T> {
    if (0.0 < b.x) {
      return {a.x + b.x, b.y - a.y};
    }
    return {a.x, b.y - a.y};
  }
};

/// @brief An edge that fills its residual one component at a time and leaves the last alone.
///
/// Writing a residual incrementally is ordinary, and it is the one way a default-constructed dual
/// reaches the jacobian scatter: component 1 is never assigned, so it carries whatever the default
/// carries.
struct PartialEdge : go::edge<PartialEdge, 2, Point<double>, go::nodes<PointNode, PointNode>> {
  using Base = go::edge<PartialEdge, 2, Point<double>, go::nodes<PointNode, PointNode>>;
  using Base::Base;

  template <class T>
  auto error(const Point<T>& a, const Point<T>& /*b*/) -> typename Base::template error_vector<T> {
    auto residual = typename Base::template error_vector<T>{};
    residual[0] = a.x - this->measurement().x;
    return residual;
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

/// @brief A Jacobian block must be rebuilt on every update, not written over.
///
/// The blocks are zero-initialised, so a single update cannot tell the difference -- every entry
/// the scatter skips still happens to hold zero. It is the *second* update that can: with b.x
/// negative the first component no longer mentions b, so b's block must lose the entry the first
/// update put there. Scattering alone would leave it behind, and the stale derivative would then
/// steer the solver with a dependency the residual no longer has.
TEST(GraphEdgeStaleJacobian, GivenADependencyThatDisappears_ExpectTheBlockToBeRebuilt) {
  Graph graph{std::pmr::new_delete_resource()};
  const auto a = graph.build<PointNode>(Graph::key_type{1});
  const auto b = graph.build<PointNode>(Graph::key_type{2});
  const auto edge = graph.build<SwitchingEdge>(a, b);
  edge->measurement(Point<double>{0.0, 0.0});

  a->estimation(Point<double>{1.0, 2.0});
  b->estimation(Point<double>{3.0, 4.0});
  edge->update();

  // While b.x is positive, component 0 is a.x + b.x: b's block carries d/db.x = 1 in row 0.
  auto blocks = std::vector<std::pair<std::size_t, math::static_vector<double, 2>>>{};
  edge->foreach_b_block([&blocks](const auto& node, const auto& block) {
    blocks.emplace_back(node->key(), math::static_vector<double, 2>(block));
  });
  ASSERT_EQ(blocks.size(), 2U);
  // error = [1 + 3, 4 - 2] = [4, 2]; J_b = [[1, 0], [0, 1]] so J_b^T * error = [4, 2].
  EXPECT_DOUBLE_EQ(blocks[1].second[0], 4.0);

  // Flip the switch: component 0 becomes a.x alone and must stop mentioning b entirely.
  b->estimation(Point<double>{-3.0, 4.0});
  edge->update();

  blocks.clear();
  edge->foreach_b_block([&blocks](const auto& node, const auto& block) {
    blocks.emplace_back(node->key(), math::static_vector<double, 2>(block));
  });
  ASSERT_EQ(blocks.size(), 2U);
  // error = [1, 2] now; J_b = [[0, 0], [0, 1]] so J_b^T * error = [0, 2]. A leading entry of 1
  // here is the first update's derivative surviving into the second.
  EXPECT_DOUBLE_EQ(blocks[1].second[0], 0.0)
      << "node b's block kept a derivative from the previous update";
  EXPECT_DOUBLE_EQ(blocks[1].second[1], 2.0);
}

/// @brief A residual component left unassigned must contribute nothing.
///
/// It holds a default-constructed dual, which carries one derivative so that every number does.
/// That derivative is zero, so the component depends on no direction and its jacobian row is zero.
///
/// The H block is what shows this, not the gradient: the unassigned component's *value* is zero
/// either way, so J^T * error cannot tell a zero row from a spurious one. H = J_a^T * J_a can --
/// row 0 contributes 1, and a second row claiming the same dependency would make it 2.
TEST(GraphEdgeDefaultResidual, GivenAnUnassignedComponent_ExpectAZeroJacobianRow) {
  Graph graph{std::pmr::new_delete_resource()};
  const auto a = graph.build<PointNode>(Graph::key_type{1});
  const auto b = graph.build<PointNode>(Graph::key_type{2});
  const auto edge = graph.build<PartialEdge>(a, b);
  edge->measurement(Point<double>{0.0, 0.0});
  a->estimation(Point<double>{5.0, 2.0});
  b->estimation(Point<double>{3.0, 4.0});
  edge->update();

  // residual = [a.x - 0, unassigned], so only component 0 depends on anything: a.x at index 0.
  auto diagonal = std::vector<std::pair<std::size_t, double>>{};
  edge->foreach_h_block([&diagonal](const auto& node_i, const auto& node_j, const auto& block) {
    if (node_i->key() == node_j->key()) {
      diagonal.emplace_back(node_i->key(), block(0, 0));
    }
  });
  ASSERT_EQ(diagonal.size(), 2U);

  // J_a = [[1, 0], [0, 0]] so H_aa(0,0) = 1. A default seeded at one would make the second row
  // claim d/da.x as well, and this would read 2.
  EXPECT_EQ(diagonal[0].first, 1U);
  EXPECT_DOUBLE_EQ(diagonal[0].second, 1.0)
      << "an unassigned residual component contributed a dependency it never expressed";

  // The residual never mentions b at all.
  EXPECT_EQ(diagonal[1].first, 2U);
  EXPECT_DOUBLE_EQ(diagonal[1].second, 0.0);
}

}  // namespace
