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

/// @brief The guard is only meaningful if its replacements for the global allocation functions are
/// the ones that get linked, so check that directly -- otherwise a linking change could quietly
/// turn the test below into one that passes for the wrong reason.
TEST(MemoryGuard, GivenAllocationInScope_ExpectThrow) {
  // `::operator new` is called directly rather than through a new-expression because the compiler
  // may elide an unused new/delete pair outright, which would prove nothing.
  EXPECT_THROW(
      {
        const memory_guard guard;
        void* const p = ::operator new(64);
        ::operator delete(p);
      },
      vortex::test::unexpected_allocation);

  // Outside a guard the same allocation is unremarkable, and the guard leaves nothing armed.
  void* const p = ::operator new(64);
  ::operator delete(p);
}

/// @brief The guard disarms itself before throwing, so allocation is permitted while the exception
/// unwinds. Without that, the destructor below would throw into an exception already in flight and
/// abort the process rather than failing this test -- which is also why it is asserted with a
/// destructor and not just a second call in a catch block.
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

/// @brief A resource that owns its storage satisfies allocations without reaching the heap, so
/// work drawing from one runs happily under a guard.
TEST(MemoryGuard, GivenArenaBackedAllocation_ExpectNoThrow) {
  arena_memory_resource arena{4096};
  EXPECT_NO_THROW({
    const memory_guard guard;
    auto* const p = arena.allocate(64, alignof(std::max_align_t));
    EXPECT_TRUE(arena.owns(p));
    arena.deallocate(p, 64, alignof(std::max_align_t));
  });
}

/// @brief A warmed-up `optimize()` must not touch the heap at all.
///
/// The graph draws from an arena owning its buffer, so everything `optimize()` needs is in hand
/// before the guard arms; anything it allocates after that comes from the heap and throws where it
/// happens. That covers the arena's own growth, `std::pmr`'s default resource when an allocation
/// ignores the active scope -- which used to take 160 `dual::number` derivative-vector copies per
/// call -- and any container or raw `new` added later. For `blaze::AlignedAllocator` see the note
/// on `memory_guard`; that channel is zero too, measured out of band.
///
/// Being pass/fail rather than a budget, it does not depend on how many iterations the solver
/// takes, so a different LAPACK cannot shift it.
TEST(SlamOptimizationBudget, GivenWarmedUpOptimize_ExpectNoHeapAllocation) {
  using Key = SlamGraph::key_type;

  // Sized past the graph's own cache_init_size so the arena serves it without growing.
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

  // Nothing but the call itself inside the guard: gtest's macros allocate, and would be reported
  // as the failure instead of whatever optimize() did.
  auto succeeded = false;
  {
    const memory_guard guard;
    succeeded = g.optimize(iterations).has_value();
  }
  EXPECT_TRUE(succeeded);
}

}  // namespace
