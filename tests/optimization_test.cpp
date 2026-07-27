/// ===========================================================================
/// @file
/// @brief End-to-end optimization test exercising the dual-number Jacobians.
/// ===========================================================================
#include <gtest/gtest.h>

#include <memory_resource>

#include "tests/fixtures/simple_slam_problem.hpp"

namespace {

using namespace g2o_dual_test;

class SlamOptimizationTest : public ::testing::Test {
 protected:
  using Key = SlamGraph::Key;

  void SetUp() override {
    p1_ = g_.build<Pose>(Key{1});
    p2_ = g_.build<Pose>(Key{2});
    p3_ = g_.build<Pose>(Key{3});
    d1_ = g_.build<PoseDistance>(*p1_, *p2_);
    d2_ = g_.build<PoseDistance>(*p2_, *p3_);
    l1_ = g_.build<PoseLocation>(*p1_);
  }

  void TearDown() override { g_.destroy(); }

  SlamGraph g_{std::pmr::new_delete_resource()};
  go::OptionalShared<Pose> p1_;
  go::OptionalShared<Pose> p2_;
  go::OptionalShared<Pose> p3_;
  go::OptionalShared<PoseDistance> d1_;
  go::OptionalShared<PoseDistance> d2_;
  go::OptionalShared<PoseLocation> l1_;
};

/// @brief The prior on p1 = (1,1) and the relative constraint p2 - p1 = (1,1)
/// drive the solution to p1 = (1,1), p2 = (2,2). p3 follows p2 through d2.
TEST_F(SlamOptimizationTest, ConvergesToExpectedEstimates) {
  (*p1_)->estimation(Pointd{0, 0});
  (*p2_)->estimation(Pointd{2, 2});
  (*p3_)->estimation(Pointd{0, 0});

  (*l1_)->measurement(Pointd{1, 1});
  (*d1_)->measurement(Pointd{1, 1});
  (*d2_)->measurement(Pointd{0, 0});

  size_t iterations = 10;
  const auto result = g_.optimize(iterations);
  ASSERT_TRUE(result.has_value());

  EXPECT_NEAR((*p1_)->estimation().x, 1.0, 1e-9);
  EXPECT_NEAR((*p1_)->estimation().y, 1.0, 1e-9);
  EXPECT_NEAR((*p2_)->estimation().x, 2.0, 1e-9);
  EXPECT_NEAR((*p2_)->estimation().y, 2.0, 1e-9);
  EXPECT_NEAR((*p3_)->estimation().x, 2.0, 1e-9);
  EXPECT_NEAR((*p3_)->estimation().y, 2.0, 1e-9);
}

/// @brief A zero-iteration optimize is a valid no-op.
TEST_F(SlamOptimizationTest, ZeroIterationsIsNoop) {
  size_t iterations = 0;
  const auto result = g_.optimize(iterations);
  EXPECT_TRUE(result.has_value());
}

}  // namespace
