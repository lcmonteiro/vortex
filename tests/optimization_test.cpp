/// ===============================================================================================
/// @file
/// @brief End-to-end optimization test exercising the dual-number Jacobians.
/// ===============================================================================================
#include <gtest/gtest.h>

#include <cstddef>
#include <memory_resource>

#include "tests/fixtures/simple_slam_graph.hpp"
#include "tests/fixtures/tracking_resource.hpp"

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

/// @brief Heap traffic of a steady-state `optimize()`, split by where it lands.
///
/// `optimize()` runs under `memory_scope{graph.memory()}`, so work that honours the scope is
/// served from the graph's arena and never reaches either counter here. Two things can still
/// escape, and this test pins both:
///
///  - the graph's arena *upstream*, which is touched only when the arena has to grow. After
///    warm-up a repeated solve on an unchanged graph should need nothing more, so the budget is
///    zero and any growth means something is now allocating per iteration.
///
///  - the `std::pmr` *default* resource, reached by allocations that ignore the active scope.
///    `std::pmr::polymorphic_allocator` does not propagate on copy construction -- its
///    `select_on_container_copy_construction` returns a default-constructed allocator -- so every
///    copy of a `dual::number`'s derivative vectors lands here instead of in the arena. That is
///    the bulk of the current budget, not a temporary of blaze's making.
///
/// Blaze's own expression temporaries use `AlignedAllocator`, which calls `posix_memalign`
/// directly and so is invisible to both counters; measured separately it is 2 allocations per
/// call here, against the 80 below. Counting it would mean interposing a libc symbol, which is
/// not worth the portability risk, so this test brackets the part that is portably observable.
///
/// The budgets are upper bounds with headroom rather than exact counts, so a different LAPACK
/// taking a slightly different number of steps does not fail the build, while a newly introduced
/// per-edge or per-iteration allocation -- which would add tens -- does.
TEST(SlamOptimizationBudget, GivenSteadyStateOptimize_ExpectBoundedHeapTraffic) {
  using Key = SlamGraph::key_type;

  vortex::test::tracking_resource fallback{std::pmr::new_delete_resource()};
  vortex::test::tracking_resource arena_upstream{std::pmr::new_delete_resource()};
  const vortex::test::default_resource_guard guard{&fallback};

  SlamGraph g{&arena_upstream};
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
  // warm-up and therefore performs the same work -- otherwise it would begin already converged,
  // exit after one iteration, and barely exercise the per-iteration path this budget guards.
  const auto seed_estimations = [&] {
    (*p1)->estimation(Position{0, 0});
    (*p2)->estimation(Position{2, 2});
    (*p3)->estimation(Position{0, 0});
  };

  const auto iterations = std::size_t{3};

  // Warm-up: sizes the solver system and the solvers' factorization workspace.
  seed_estimations();
  ASSERT_TRUE(g.optimize(iterations).has_value());

  seed_estimations();
  fallback.reset_counters();
  arena_upstream.reset_counters();

  ASSERT_TRUE(g.optimize(iterations).has_value());

  // The arena is already big enough; a repeated solve must not make it grow.
  EXPECT_EQ(arena_upstream.allocations, 0U)
      << "optimize() grew the graph arena in steady state";

  // Allocations that bypass the active memory_scope entirely -- today exactly 160, all of them
  // dual::number derivative-vector copies (see the note above). The margin is deliberately small
  // so a regression is caught: adding one allocation per edge to the per-iteration path costs 9
  // here and trips this. If a platform legitimately lands elsewhere the message below reports the
  // number to update this constant to.
  constexpr auto kFallbackBudget = std::size_t{166};
  EXPECT_LE(fallback.allocations, kFallbackBudget)
      << "optimize() allocated " << fallback.allocations
      << " times outside the memory_scope, over the budget of " << kFallbackBudget;

  // Nothing may be released through a resource that did not allocate it.
  EXPECT_EQ(fallback.foreign_frees, 0U);
  EXPECT_EQ(arena_upstream.foreign_frees, 0U);
}

}  // namespace
