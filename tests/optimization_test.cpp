/// ===============================================================================================
/// @file
/// @brief End-to-end optimization test exercising the dual-number Jacobians.
/// ===============================================================================================
#include <gtest/gtest.h>

#include <cstddef>
#include <memory_resource>

#include "tests/fixtures/simple_slam_graph.hpp"
#include "tests/fixtures/arena_memory_resource.hpp"
#include "tests/fixtures/memory_guard.hpp"

namespace {

using namespace vortex::test;

class SlamOptimizationTest : public ::testing::Test {
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
  go::option<PositionNode> p1_;
  go::option<PositionNode> p2_;
  go::option<PositionNode> p3_;
  go::option<PositionDistanceEdge> d1_;
  go::option<PositionDistanceEdge> d2_;
  go::option<PositionLocationEdge> l1_;
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

  const auto iterations = std::size_t{3};
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
  const auto iterations = std::size_t{0};
  const auto result = g_.optimize(iterations);
  EXPECT_TRUE(result.has_value());
}

/// ===============================================================================================
/// Allocation budget
/// ===============================================================================================

/// @brief Checks the guard's replacements are the ones linked -- otherwise the budget test below
/// would pass for the wrong reason. `::operator new` is called directly because the compiler may
/// elide an unused new/delete pair, which would prove nothing.
TEST(MemoryGuard, GivenAllocationInScope_ExpectThrow) {
  EXPECT_THROW(
      {
        const memory_guard guard;
        void* const p = ::operator new(64);
        ::operator delete(p);
      },
      vortex::test::unexpected_allocation);

  // The guard leaves nothing armed behind it.
  void* const p = ::operator new(64);
  ::operator delete(p);
}

/// @brief The guard disarms before throwing, so allocation is permitted while the exception
/// unwinds. Without it the destructor below throws into an exception already in flight, aborting
/// the process instead of failing the test.
TEST(MemoryGuard, GivenRejection_ExpectAllocationPermittedWhileUnwinding) {
  struct allocates_on_destruction {
    ~allocates_on_destruction() { ::operator delete(::operator new(64)); }
  };

  EXPECT_THROW(
      {
        const memory_guard guard;
        const allocates_on_destruction unwinds;  // destroyed as the throw below unwinds
        ::operator delete(::operator new(64));
      },
      vortex::test::unexpected_allocation);

  // The guard is gone, so the next scope arms again.
  EXPECT_THROW(
      {
        const memory_guard guard;
        ::operator delete(::operator new(64));
      },
      vortex::test::unexpected_allocation);
}

/// @brief A warmed-up `optimize()` must not touch the heap at all.
///
/// The graph draws from an arena, so anything `optimize()` allocates afterwards comes from the
/// heap and throws where it happens. Being pass/fail rather than a budget, it does not depend on
/// how many iterations the solver takes.
TEST(SlamOptimizationBudget, GivenWarmedUpOptimize_ExpectNoHeapAllocation) {
  using Key = SlamGraph::key_type;

  arena_memory_resource arena{std::size_t{8} << 20U};

  SlamGraph g{&arena};
  go::option<PositionNode> p1 = g.build<PositionNode>(Key{1});
  go::option<PositionNode> p2 = g.build<PositionNode>(Key{2});
  go::option<PositionNode> p3 = g.build<PositionNode>(Key{3});
  go::option<PositionDistanceEdge> d1 = g.build<PositionDistanceEdge>(*p1, *p2);
  go::option<PositionDistanceEdge> d2 = g.build<PositionDistanceEdge>(*p2, *p3);
  go::option<PositionLocationEdge> l1 = g.build<PositionLocationEdge>(*p1);

  (*l1)->measurement(Position{1, 1});
  (*d1)->measurement(Position{1, 1});
  (*d2)->measurement(Position{0, 0});

  // Re-applied before each run so the measured pass repeats the warm-up's work; otherwise it
  // starts converged, exits after one iteration and barely exercises the per-iteration path.
  const auto seed_estimations = [&] {
    (*p1)->estimation(Position{0, 0});
    (*p2)->estimation(Position{2, 2});
    (*p3)->estimation(Position{0, 0});
  };

  const auto iterations = std::size_t{3};

  // Warm-up.
  seed_estimations();
  ASSERT_TRUE(g.optimize(iterations).has_value());

  // The guard cannot go higher: building the graph reserves the solver system, and the warm-up
  // sizes it and the LAPACK workspaces. The assertion is outside because gtest's macros allocate.
  auto succeeded = false;
  {
    const memory_guard guard;
    seed_estimations();
    succeeded = g.optimize(iterations).has_value();
  }
  EXPECT_TRUE(succeeded);
}

}  // namespace
