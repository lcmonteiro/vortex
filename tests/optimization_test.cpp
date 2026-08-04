/// ===========================================================================
/// @file
/// @brief End-to-end optimization test exercising the dual-number Jacobians.
/// ===========================================================================
#include <gtest/gtest.h>

#include <memory_resource>

#include "tests/fixtures/simple_slam_graph.hpp"

namespace {

using namespace vortex::test;

class SlamOptimizationTest : public ::testing::Test {
 protected:
  using Key = SlamGraph::Key;

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
  go::OptionalShared<PositionNode> p1_;
  go::OptionalShared<PositionNode> p2_;
  go::OptionalShared<PositionNode> p3_;
  go::OptionalShared<PositionDistanceEdge> d1_;
  go::OptionalShared<PositionDistanceEdge> d2_;
  go::OptionalShared<PositionLocationEdge> l1_;
};

/// @brief The prior on p1 = (1,1) and the relative constraint p2 - p1 = (1,1)
/// drive the solution to p1 = (1,1), p2 = (2,2). p3 follows p2 through d2.
TEST_F(SlamOptimizationTest, ConvergesToExpectedEstimates) {
  (*p1_)->estimation(Position{0, 0});
  (*p2_)->estimation(Position{2, 2});
  (*p3_)->estimation(Position{0, 0});

  (*l1_)->measurement(Position{1, 1});
  (*d1_)->measurement(Position{1, 1});
  (*d2_)->measurement(Position{0, 0});

  const size_t iterations = 3;
  const auto result = g_.optimize(iterations);
  ASSERT_TRUE(result.has_value());

  EXPECT_NEAR((*p1_)->estimation().x, 1.0, 1e-12);
  EXPECT_NEAR((*p1_)->estimation().y, 1.0, 1e-12);
  EXPECT_NEAR((*p2_)->estimation().x, 2.0, 1e-12);
  EXPECT_NEAR((*p2_)->estimation().y, 2.0, 1e-12);
  EXPECT_NEAR((*p3_)->estimation().x, 2.0, 1e-12);
  EXPECT_NEAR((*p3_)->estimation().y, 2.0, 1e-12);
}

/// @brief A zero-iteration optimize is a valid no-op.
TEST_F(SlamOptimizationTest, ZeroIterationsIsNoop) {
  size_t iterations = 0;
  const auto result = g_.optimize(iterations);
  EXPECT_TRUE(result.has_value());
}

}  // namespace
