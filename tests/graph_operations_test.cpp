/// ===============================================================================================
/// @file
/// @brief Unit tests for SLAM graph optimization operations: measurements,
/// estimations, estimation backlog (push/pull/revert), chi2, node/edge
/// enable-disable, and the `graph_operations.hpp` free-function API
/// (ForEach/FindIf/RemoveIf/Enable/Disable/EnableIf/DisableIf).
/// ===============================================================================================
#include <gtest/gtest.h>

#include <cstddef>
#include <memory_resource>

#include "tests/fixtures/simple_slam_graph.hpp"

namespace {

using namespace vortex::test;

void ExpectPositionEq(const Position& actual, const Position& expected) {
  EXPECT_DOUBLE_EQ(actual.x, expected.x);
  EXPECT_DOUBLE_EQ(actual.y, expected.y);
}

class SlamGraphOperationsTest : public ::testing::Test {
 protected:
  using Key = SlamGraph::key_type;

  auto SetUp() -> void override {
    p1_ = g_.build<PositionNode>(Key{1});
    p2_ = g_.build<PositionNode>(Key{2});
    p3_ = g_.build<PositionNode>(Key{3});
    d1_ = g_.build<PositionDistanceEdge>(*p1_, *p2_);
    d2_ = g_.build<PositionDistanceEdge>(*p2_, *p3_);
    l1_ = g_.build<PositionLocationEdge>(*p1_);
  }

  auto TearDown() -> void override { g_.destroy(); }

  SlamGraph g_{std::pmr::new_delete_resource()};
  go::optional<PositionNode> p1_;
  go::optional<PositionNode> p2_;
  go::optional<PositionNode> p3_;
  go::optional<PositionDistanceEdge> d1_;
  go::optional<PositionDistanceEdge> d2_;
  go::optional<PositionLocationEdge> l1_;
};

/// @brief Verifies that edge measurements can be set and retrieved.
TEST_F(SlamGraphOperationsTest, CheckMeasurements) {
  auto d1 = Position{1, 0};
  auto d2 = Position{1, 2};
  auto l1 = Position{0.5, 0};
  (*d1_)->measurement(d1);
  (*d2_)->measurement(d2);
  (*l1_)->measurement(l1);

  ExpectPositionEq((*d1_)->measurement(), d1);
  ExpectPositionEq((*d2_)->measurement(), d2);
  ExpectPositionEq((*l1_)->measurement(), l1);
}

/// @brief Verifies that node estimations can be set and retrieved.
TEST_F(SlamGraphOperationsTest, CheckEstimations) {
  auto p1 = Position{0, 0};
  auto p2 = Position{1, 1};
  auto p3 = Position{2, 3};
  (*p1_)->estimation(p1);
  (*p2_)->estimation(p2);
  (*p3_)->estimation(p3);

  ExpectPositionEq((*p1_)->estimation(), p1);
  ExpectPositionEq((*p2_)->estimation(), p2);
  ExpectPositionEq((*p3_)->estimation(), p3);
}

/// @brief Verifies push/pull/revert backlog for node estimations.
TEST_F(SlamGraphOperationsTest, CheckEstimationsBacklog) {
  auto p1 = Position{0, 0};
  auto p2 = Position{1, 1};
  auto p3 = Position{2, 3};
  (*p1_)->estimation(p1);
  (*p2_)->estimation(p2);
  (*p3_)->estimation(p3);
  g_.push();
  (*p1_)->estimation(p2);
  g_.push();
  (*p1_)->estimation(p3);
  g_.push();
  (*p1_)->estimation(p1);

  g_.pull();
  ExpectPositionEq((*p1_)->estimation(), p3);
  g_.revert();
  ExpectPositionEq((*p1_)->estimation(), p2);
  g_.revert();
  ExpectPositionEq((*p1_)->estimation(), p1);
}

/// @brief Verifies full optimization converges to expected estimations.
TEST_F(SlamGraphOperationsTest, CheckOptimizeProcess) {
  auto iterations = std::size_t{0};
  auto result = g_.optimize(iterations);
  ASSERT_TRUE(result.has_value());
  iterations = 1;
  result = g_.optimize(iterations);
  ASSERT_TRUE(result.has_value());

  auto p1 = Position{0, 0};
  auto p2 = Position{2, 2};
  auto d1 = Position{1, 1};
  auto l1 = Position{1, 1};
  (*p1_)->estimation(p1);
  (*p2_)->estimation(p2);
  (*d1_)->measurement(d1);
  (*l1_)->measurement(l1);

  iterations = 10;
  result = g_.optimize(iterations);
  ASSERT_TRUE(result.has_value());

  auto p1_expect = Position{1, 1};
  auto p2_expect = Position{2, 2};
  EXPECT_NEAR((*p1_)->estimation().x, p1_expect.x, 1e-9);
  EXPECT_NEAR((*p1_)->estimation().y, p1_expect.y, 1e-9);
  EXPECT_NEAR((*p2_)->estimation().x, p2_expect.x, 1e-9);
  EXPECT_NEAR((*p2_)->estimation().y, p2_expect.y, 1e-9);
}

/// @brief Verifies optimization fails when all nodes and edges are disabled.
TEST_F(SlamGraphOperationsTest, GivenDisabledGraph_ExpectLevenbergReset) {
  g_.toggle<PositionNode>(SlamGraph::enabled_type{});
  g_.toggle<PositionDistanceEdge>(SlamGraph::enabled_type{});

  const auto iterations = std::size_t{10};
  auto result = g_.optimize(iterations);
  ASSERT_FALSE(result.has_value());
}

/// @brief Verifies bulk measurement assignment via ForEach on edge types.
TEST_F(SlamGraphOperationsTest, GivenMeasurementForEachEdgeTypeGivenCondition_ExpectCorrectStatus) {
  auto d1 = Position{1, 0};
  auto l1 = Position{0.5, 0};
  go::ForEach<PositionDistanceEdge>(g_, [&](const auto& edge) { edge->measurement(d1); });
  go::ForEach<PositionLocationEdge>(g_, [&](const auto& edge) { edge->measurement(l1); });
  ExpectPositionEq((*d1_)->measurement(), d1);
  ExpectPositionEq((*d2_)->measurement(), d1);
  ExpectPositionEq((*l1_)->measurement(), l1);
}

/// @brief Verifies FindIf retrieves the correct node by key.
TEST_F(SlamGraphOperationsTest, GivenKey_ExpectCorrectFind) {
  constexpr auto key_that_exists = Key{2};
  const auto& node_found = go::FindIf<PositionNode>(g_, key_that_exists);
  EXPECT_EQ(node_found, p2_);
}

/// @brief Verifies FindIf on a const graph retrieves the correct node by key.
TEST_F(SlamGraphOperationsTest, GivenKey_ExpectCorrectFind_Const) {
  constexpr auto key_that_exists = Key{2};
  const auto& node_found =
      go::FindIf<PositionNode>(const_cast<const SlamGraph&>(g_), key_that_exists);
  EXPECT_EQ(node_found, p2_);
}

/// @brief Verifies FindIf with a predicate lambda finds the correct node.
TEST_F(SlamGraphOperationsTest, GivenKeyCondition_ExpectCorrectFind) {
  constexpr auto key_that_exists = Key{2};
  const auto& node_found = go::FindIf<PositionNode>(
      g_, [](const auto& node) { return (node->key() == key_that_exists) ? true : false; });
  EXPECT_EQ(node_found, p2_);
}

/// @brief Verifies FindIf with a predicate on a const graph.
TEST_F(SlamGraphOperationsTest, GivenKeyCondition_ExpectCorrectFind_Const) {
  constexpr auto key_that_exists = Key{2};
  const auto& node_found = go::FindIf<PositionNode>(
      const_cast<const SlamGraph&>(g_),
      [](const auto& node) { return (node->key() == key_that_exists) ? true : false; });
  EXPECT_EQ(node_found, p2_);
}

/// @brief Verifies FindIf returns empty when key does not exist.
TEST_F(SlamGraphOperationsTest, GivenKey_ExpectNoFind) {
  constexpr auto key_that_does_not_exist = Key{100};
  const auto& node_found = go::FindIf<PositionNode>(g_, key_that_does_not_exist);
  EXPECT_FALSE(node_found.has_value());
}

/// @brief Verifies FindIf on a const graph returns empty for missing key.
TEST_F(SlamGraphOperationsTest, GivenKey_ExpectNoFind_Const) {
  constexpr auto key_that_does_not_exist = Key{100};
  const auto& node_found =
      go::FindIf<PositionNode>(const_cast<const SlamGraph&>(g_), key_that_does_not_exist);
  EXPECT_FALSE(node_found.has_value());
}

/// @brief Verifies FindIf with a predicate returns empty when no match.
TEST_F(SlamGraphOperationsTest, GivenKeyCondition_ExpectNoFind) {
  constexpr auto key_that_does_not_exist = Key{100};
  const auto& node_found = go::FindIf<PositionNode>(
      g_, [](const auto& node) { return (node->key() == key_that_does_not_exist) ? true : false; });
  EXPECT_FALSE(node_found.has_value());
}

/// @brief Verifies FindIf with a predicate on a const graph returns empty.
TEST_F(SlamGraphOperationsTest, GivenKeyCondition_ExpectNoFind_Const) {
  constexpr auto key_that_does_not_exist = Key{100};
  const auto& node_found = go::FindIf<PositionNode>(
      const_cast<const SlamGraph&>(g_),
      [](const auto& node) { return (node->key() == key_that_does_not_exist) ? true : false; });
  EXPECT_FALSE(node_found.has_value());
}

/// @brief Verifies RemoveIf with a false predicate removes nothing.
TEST_F(SlamGraphOperationsTest, GivenFalseCondition_ExpectNonRemoval) {
  ASSERT_EQ(g_.size<PositionNode>(), 3U);
  go::RemoveIf<PositionNode>(g_, [](const auto&) { return false; });
  EXPECT_EQ(g_.size<PositionNode>(), 3U);
}

/// @brief Verifies RemoveIf with a true predicate removes all nodes.
TEST_F(SlamGraphOperationsTest, GivenTrueCondition_ExpectRemoval) {
  ASSERT_EQ(g_.size<PositionNode>(), 3U);
  go::RemoveIf<PositionNode>(g_, [](const auto&) { return true; });
  EXPECT_EQ(g_.size<PositionNode>(), 0U);
}

/// @brief Verifies RemoveIf with a key predicate removes only the matching node.
TEST_F(SlamGraphOperationsTest, GivenKeyCondition_ExpectCorrectRemoval) {
  ASSERT_EQ(g_.size<PositionNode>(), 3U);
  go::RemoveIf<PositionNode>(g_,
                             [](const auto& node) { return (node->key() == 1) ? true : false; });
  ASSERT_EQ(g_.size<PositionNode>(), 2U);
  EXPECT_FALSE(g_.find<PositionNode>(1).has_value());
}

/// @brief Verifies Disable/Enable by type toggles nodes and edges correctly.
TEST_F(SlamGraphOperationsTest, GivenDisableAndEnableByType_ExpectCorrectStatus) {
  go::Disable<PositionNode>(g_);
  go::Disable<PositionDistanceEdge>(g_);
  EXPECT_TRUE((*p1_)->disable());
  EXPECT_TRUE((*p2_)->disable());
  EXPECT_TRUE((*d1_)->disable());
  EXPECT_FALSE((*l1_)->disable());

  go::Enable<PositionNode>(g_);
  go::Enable<PositionDistanceEdge>(g_);
  EXPECT_FALSE((*p1_)->disable());
  EXPECT_FALSE((*p2_)->disable());
  EXPECT_FALSE((*p3_)->disable());
  EXPECT_FALSE((*d1_)->disable());
  EXPECT_FALSE((*l1_)->disable());

  go::Disable<go::Edges<PositionDistanceEdge, PositionLocationEdge>>(g_);
  EXPECT_TRUE((*d1_)->disable());
  EXPECT_TRUE((*l1_)->disable());

  go::Enable<go::Edges<PositionDistanceEdge, PositionLocationEdge>>(g_);
  EXPECT_FALSE((*d1_)->disable());
  EXPECT_FALSE((*l1_)->disable());
}

/// @brief Verifies that disabling twice is idempotent.
TEST_F(SlamGraphOperationsTest, GivenDisableTwice_ExpectCorrectStatus) {
  go::Disable<PositionNode>(g_);
  go::Disable<PositionDistanceEdge>(g_);
  EXPECT_TRUE((*p1_)->disable());
  EXPECT_TRUE((*p2_)->disable());
  EXPECT_TRUE((*d1_)->disable());
  EXPECT_FALSE((*l1_)->disable());

  go::Disable<PositionNode>(g_);
  go::Disable<PositionDistanceEdge>(g_);
  EXPECT_TRUE((*p1_)->disable());
  EXPECT_TRUE((*p2_)->disable());
  EXPECT_TRUE((*d1_)->disable());
  EXPECT_FALSE((*l1_)->disable());
}

/// @brief Verifies Disable/Enable by key or edge reference targets correctly.
TEST_F(SlamGraphOperationsTest, GivenDisableAndEnableByKey_ExpectCorrectStatus) {
  go::Disable<PositionNode>(g_, Key{1});
  go::Disable<PositionDistanceEdge>(g_, *d1_);
  EXPECT_TRUE((*p1_)->disable());
  EXPECT_FALSE((*p2_)->disable());
  EXPECT_FALSE((*p3_)->disable());
  EXPECT_TRUE((*d1_)->disable());
  EXPECT_FALSE((*l1_)->disable());

  go::Enable<PositionNode>(g_, Key{1});
  go::Enable<PositionDistanceEdge>(g_, *d1_);
  EXPECT_FALSE((*p1_)->disable());
  EXPECT_FALSE((*p2_)->disable());
  EXPECT_FALSE((*p3_)->disable());
  EXPECT_FALSE((*d1_)->disable());
  EXPECT_FALSE((*l1_)->disable());

  go::Disable<PositionDistanceEdge>(g_, *p1_);
  EXPECT_TRUE((*d1_)->disable());
  EXPECT_FALSE((*l1_)->disable());

  go::Enable<PositionDistanceEdge>(g_, *p1_);
  EXPECT_FALSE((*d1_)->disable());
  EXPECT_FALSE((*l1_)->disable());
}

/// @brief Verifies Disable/Enable of multiple edge types by node reference.
TEST_F(SlamGraphOperationsTest, GivenDisableAndEnableByKeyList_ExpectCorrectStatus) {
  go::Disable<go::Edges<PositionDistanceEdge, PositionLocationEdge>>(g_, *p1_);
  EXPECT_TRUE((*d1_)->disable());
  EXPECT_TRUE((*l1_)->disable());
  go::Enable<go::Edges<PositionDistanceEdge, PositionLocationEdge>>(g_, *p1_);
  EXPECT_FALSE((*d1_)->disable());
  EXPECT_FALSE((*l1_)->disable());
}

/// @brief Verifies DisableIf/EnableIf with predicates on nodes and edges.
TEST_F(SlamGraphOperationsTest, GivenDisableAndEnableByCondition_ExpectCorrectStatus) {
  go::DisableIf<PositionNode>(g_,
                              [](const auto& node) { return (node->key() == 1) ? true : false; });
  go::DisableIf<PositionDistanceEdge>(
      g_, [](const auto& edge) { return (edge->n_nodes == 2) ? true : false; });
  EXPECT_TRUE((*p1_)->disable());
  EXPECT_FALSE((*p2_)->disable());
  EXPECT_FALSE((*p3_)->disable());
  EXPECT_TRUE((*d1_)->disable());
  EXPECT_FALSE((*l1_)->disable());

  go::EnableIf<PositionNode>(g_,
                             [](const auto& node) { return (node->key() == 1) ? true : false; });
  go::EnableIf<PositionDistanceEdge>(
      g_, [](const auto& edge) { return (edge->n_nodes == 2) ? true : false; });
  EXPECT_FALSE((*p1_)->disable());
  EXPECT_FALSE((*p2_)->disable());
  EXPECT_FALSE((*p3_)->disable());
  EXPECT_FALSE((*d1_)->disable());
  EXPECT_FALSE((*l1_)->disable());
}

/// @brief Verifies chi2 is non-zero when estimations don't match measurements.
TEST_F(SlamGraphOperationsTest, GivenMeasuredGraph_ExpectNonZeroChi2) {
  (*p1_)->estimation(Position{0, 0});
  (*p2_)->estimation(Position{2, 2});
  (*p3_)->estimation(Position{3, 3});
  (*d1_)->measurement(Position{1, 1});
  (*d2_)->measurement(Position{0, 1});
  (*l1_)->measurement(Position{1, 1});

  g_.update_edges();
  auto chi2 = g_.chi2();
  EXPECT_GT(chi2, 0.0);
}

/// @brief Verifies chi2 is zero when estimations perfectly satisfy constraints.
TEST_F(SlamGraphOperationsTest, GivenPerfectEstimations_ExpectZeroChi2) {
  (*p1_)->estimation(Position{0, 0});
  (*p2_)->estimation(Position{1, 1});
  (*p3_)->estimation(Position{1, 2});
  (*d1_)->measurement(Position{1, 1});
  (*d2_)->measurement(Position{0, 1});
  (*l1_)->measurement(Position{0, 0});

  g_.update_edges();
  auto chi2 = g_.chi2();
  EXPECT_NEAR(chi2, 0.0, 1e-9);
}

/// @brief Verifies that updateEdges() recalculates chi2 after estimations change.
TEST_F(SlamGraphOperationsTest, GivenUpdateEdges_ExpectChi2ReflectsCurrentState) {
  (*p1_)->estimation(Position{0, 0});
  (*p2_)->estimation(Position{1, 1});
  (*p3_)->estimation(Position{1, 2});
  (*d1_)->measurement(Position{1, 1});
  (*d2_)->measurement(Position{0, 1});
  (*l1_)->measurement(Position{0, 0});

  g_.update_edges();
  EXPECT_NEAR(g_.chi2(), 0.0, 1e-9);

  (*p1_)->estimation(Position{5, 5});
  g_.update_edges();
  EXPECT_GT(g_.chi2(), 0.0);
}

/// @brief Applies delta vector [dx1,dy1,dx2,dy2,dx3,dy3] and verifies each
/// node estimation is updated via plus().
TEST_F(SlamGraphOperationsTest, GivenDeltaVector_ExpectEstimationsUpdated) {
  (*p1_)->estimation(Position{0, 0});
  (*p2_)->estimation(Position{1, 1});
  (*p3_)->estimation(Position{2, 2});

  auto x = SlamGraph::vector_type{0.5, -0.5, 1.0, 2.0, -1.0, 0.0};
  g_.update_nodes(x);

  EXPECT_NEAR((*p1_)->estimation().x, 0.5, 1e-9);
  EXPECT_NEAR((*p1_)->estimation().y, -0.5, 1e-9);
  EXPECT_NEAR((*p2_)->estimation().x, 2.0, 1e-9);
  EXPECT_NEAR((*p2_)->estimation().y, 3.0, 1e-9);
  EXPECT_NEAR((*p3_)->estimation().x, 1.0, 1e-9);
  EXPECT_NEAR((*p3_)->estimation().y, 2.0, 1e-9);
}

/// @brief Verifies LM step rejection and convergence from wildly wrong estimates.
TEST_F(SlamGraphOperationsTest, GivenLargeInitialError_ExpectStepRejectionAndConvergence) {
  (*p1_)->estimation(Position{1000, -500});
  (*p2_)->estimation(Position{-800, 2000});
  (*p3_)->estimation(Position{3000, -1000});

  (*d1_)->measurement(Position{1, 0});
  (*d2_)->measurement(Position{0, 1});
  (*l1_)->measurement(Position{0, 0});

  auto result = g_.optimize(100U);
  ASSERT_TRUE(result.has_value());
}

/// @brief Verifies optimization with a reversed-order edge exercises the
/// position_i < position_j H-block assembly path.
TEST_F(SlamGraphOperationsTest, GivenReversedEdgeOrder_ExpectCorrectOptimization) {
  auto d3 = g_.build<PositionDistanceEdge>(*p2_, *p1_);

  (*d1_)->measurement(Position{1, 0});
  (*d2_)->measurement(Position{1, 0});
  d3->measurement(Position{-1, 0});
  (*l1_)->measurement(Position{0, 0});

  auto result = g_.optimize(10U);
  ASSERT_TRUE(result.has_value());
}

/// @brief Verifies that lambda is positive after optimization.
TEST_F(SlamGraphOperationsTest, GivenOptimizedGraph_ExpectPositiveLambda) {
  (*d1_)->measurement(Position{1, 0});
  (*d2_)->measurement(Position{0, 1});
  (*l1_)->measurement(Position{0, 0});

  auto result = g_.optimize(10U);
  ASSERT_TRUE(result.has_value());
  EXPECT_GT(g_.algorithm.lambda(), 0.0);
}

/// @brief Verifies that shared copy assignment works for node and edge types.
TEST_F(SlamGraphOperationsTest, GivenSharedObjects_ExpectCopyAssignment) {
  auto pose_copy = *p1_;
  auto pose_target = *p2_;
  pose_target = pose_copy;
  EXPECT_EQ(pose_target.operator->(), pose_copy.operator->());

  auto edge_copy = *d1_;
  auto edge_target = *d2_;
  edge_target = edge_copy;
  EXPECT_EQ(edge_target.operator->(), edge_copy.operator->());

  auto loc_copy = *l1_;
  auto loc_target = loc_copy;
  loc_target = loc_copy;
  EXPECT_EQ(loc_target.operator->(), loc_copy.operator->());
}

/// @brief Verifies unlink of PositionLocationEdge edges from nodes.
TEST_F(SlamGraphOperationsTest, GivenGraph_ExpectPositionLocationUnlink) {
  auto edges_before = g_.size<SlamGraph::Edges>();
  g_.unlink<PositionNode, PositionLocationEdge>();
  EXPECT_LT(g_.size<SlamGraph::Edges>(), edges_before);
}

/// @brief Verifies 2-iteration optimization covers the last-iteration path.
TEST_F(SlamGraphOperationsTest, GivenTwoIterations_ExpectLastIterationSolvePath) {
  (*d1_)->measurement(Position{1, 0});
  (*d2_)->measurement(Position{0, 1});
  (*l1_)->measurement(Position{0, 0});

  auto result = g_.optimize(2U);
  ASSERT_TRUE(result.has_value());
}

/// @brief Verifies destroy of a single PositionNode node by key.
TEST_F(SlamGraphOperationsTest, GivenPositionNodeKey_ExpectDestroyByKey) {
  auto nodes_before = g_.size<go::Nodes<PositionNode>>();
  auto edges_before = g_.size<SlamGraph::Edges>();
  g_.destroy<PositionNode>(Key{2});
  EXPECT_LT(g_.size<go::Nodes<PositionNode>>(), nodes_before);
  EXPECT_LT(g_.size<SlamGraph::Edges>(), edges_before);
}

}  // namespace
