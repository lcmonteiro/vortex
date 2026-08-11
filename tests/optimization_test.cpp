/// ===============================================================================================
/// @file
/// @brief End-to-end optimization test exercising the dual-number Jacobians.
/// ===============================================================================================
#include <gtest/gtest.h>

#include <cstddef>
#include <memory_resource>

#include "tests/fixtures/simple_slam_graph.hpp"
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

/// @brief The zero assertion below only means anything if the guard actually observes
/// allocations, which depends on this binary's replacements for the global allocation functions
/// being the ones that get linked. Check that directly, so a linking change cannot quietly turn
/// the budget test into a no-op that passes for the wrong reason.
TEST(MemoryGuard, GivenAllocations_ExpectTheyAreObserved) {
  // Counted inside the guarded block and asserted outside it: the gtest macros allocate too, so
  // reading the counters straight into EXPECT_EQ would fold their traffic into the result.
  // `::operator new` is called directly rather than via a new-expression because the compiler is
  // allowed to elide an unused new/delete pair outright, which would make this prove nothing.
  auto allocations = std::size_t{0};
  auto deallocations = std::size_t{0};
  {
    const memory_guard guard;
    auto* const p = ::operator new(64);
    allocations = guard.allocations();
    ::operator delete(p);
    deallocations = guard.deallocations();
  }
  EXPECT_EQ(allocations, 1U);
  EXPECT_EQ(deallocations, 1U);
}

/// @brief A warmed-up `optimize()` must not touch the heap at all.
///
/// `optimize()` runs under `memory_scope{graph.memory()}`, so everything it needs should come from
/// the graph's arena, which is already sized after the first call. `memory_guard` counts the
/// global `operator new`, so this covers every source that goes through it: the arena's upstream
/// when it grows, `std::pmr`'s default resource when an allocation ignores the active scope --
/// `polymorphic_allocator` does not propagate on copy construction, which used to send 160
/// `dual::number` derivative-vector copies per call there -- and any plain container or raw `new`
/// added to the path later.
///
/// It does not cover `blaze::AlignedAllocator`, which reaches `posix_memalign` without passing
/// through `operator new`; see the note on `memory_guard`. That channel is zero too, measured out
/// of band, since `math::solve_ldlt` began caching the buffers it hands to LAPACK.
///
/// Being an exact zero rather than a budget, this does not depend on how many iterations the
/// solver takes, so a different LAPACK cannot shift it.
TEST(SlamOptimizationBudget, GivenWarmedUpOptimize_ExpectNoHeapAllocation) {
  using Key = SlamGraph::key_type;

  SlamGraph g{std::pmr::new_delete_resource()};
  go::option<PositionNode> p1 = g.build<PositionNode>(Key{1});
  go::option<PositionNode> p2 = g.build<PositionNode>(Key{2});
  go::option<PositionNode> p3 = g.build<PositionNode>(Key{3});
  go::option<PositionDistanceEdge> d1 = g.build<PositionDistanceEdge>(*p1, *p2);
  go::option<PositionDistanceEdge> d2 = g.build<PositionDistanceEdge>(*p2, *p3);
  go::option<PositionLocationEdge> l1 = g.build<PositionLocationEdge>(*p1);

  (*l1)->measurement(Position{1, 1});
  (*d1)->measurement(Position{1, 1});
  (*d2)->measurement(Position{0, 0});

  // Re-applied before each run so the measured pass starts from the same displaced state as the
  // warm-up and therefore does the same work -- otherwise it would begin already converged, exit
  // after one iteration, and barely exercise the per-iteration path this guards.
  const auto seed_estimations = [&] {
    (*p1)->estimation(Position{0, 0});
    (*p2)->estimation(Position{2, 2});
    (*p3)->estimation(Position{0, 0});
  };

  const auto iterations = std::size_t{3};

  // Warm-up: sizes the solver system and the solvers' LAPACK workspaces.
  seed_estimations();
  ASSERT_TRUE(g.optimize(iterations).has_value());

  seed_estimations();
  {
    const memory_guard guard;
    ASSERT_TRUE(g.optimize(iterations).has_value());
    EXPECT_EQ(guard.allocations(), 0U)
        << "optimize() performed " << guard.allocations() << " heap allocations, which must be 0";
  }
}

}  // namespace
