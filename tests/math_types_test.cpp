/// ===============================================================================================
/// @file
/// @brief Unit tests for `vortex::math` matrix/vector types and the patched `AlignedAllocator`
/// that ties blaze's dynamic containers to `helpers::memory_scope`.
/// ===============================================================================================
#include <gtest/gtest.h>

#include <cstddef>
#include <memory_resource>
#include <utility>

#include "foundation/math/solver.hpp"
#include "foundation/math/types.hpp"
#include "helpers/memory.hpp"
#include "tests/fixtures/memory_arena.hpp"

namespace {

using vortex::math::dynamic_matrix;
using vortex::math::dynamic_vector;
using vortex::helpers::memory_scope;
using vortex::test::memory_arena;

TEST(MathTypes, DynamicVectorUsesActiveMemoryScope) {
  memory_arena resource;
  const memory_scope scope{&resource};

  // Under a guard, so reaching the heap instead of the arena throws. Kept to the allocation
  // itself because gtest's macros allocate too.
  auto v = [] { return dynamic_vector<double>(4, 0.0); }();
  {
    const memory_arena::guard guard;
    v.resize(64, false);
  }
  v[0] = 1.0;
  v[63] = 4.0;

  EXPECT_TRUE(resource.owns(v.data()));
  EXPECT_DOUBLE_EQ(v[0], 1.0);
  EXPECT_DOUBLE_EQ(v[63], 4.0);
}

TEST(MathTypes, DynamicMatrixUsesActiveMemoryScope) {
  memory_arena resource;
  const memory_scope scope{&resource};

  dynamic_matrix<double> m(2, 2, 0.0);
  m(0, 0) = 1.0;
  m(1, 1) = 2.0;

  EXPECT_TRUE(resource.owns(m.data()));
  EXPECT_DOUBLE_EQ(m(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(m(1, 1), 2.0);
}

/// @brief `resize()` past capacity builds the replacement with a fresh allocator then swaps only
/// the buffers, leaving each container its original one. This is what
/// `block_graph_solver::build_structure()` does inside `optimize()`'s scope.
TEST(MathTypes, GivenResizeInsideScope_ExpectBuffersReleasedByOwningResource) {
  memory_arena outer;
  memory_arena arena;

  {
    const memory_scope outer_scope{&outer};
    dynamic_vector<double> v(8, 0.0);

    EXPECT_TRUE(outer.owns(v.data()));
    {
      const memory_scope arena_scope{&arena};
      v.resize(64, false);
      EXPECT_TRUE(arena.owns(v.data()));
    }
    v[0] = 1.0;
    EXPECT_DOUBLE_EQ(v[0], 1.0);
  }
}

/// @brief Same reasoning for a matrix, which `build_structure()` also resizes.
TEST(MathTypes, GivenMatrixResizeInsideScope_ExpectBuffersReleasedByOwningResource) {
  memory_arena outer;
  memory_arena arena;

  {
    const memory_scope outer_scope{&outer};
    dynamic_matrix<double> m(4, 4, 0.0);

    EXPECT_TRUE(outer.owns(m.data()));
    {
      const memory_scope arena_scope{&arena};
      m.resize(32, 32, false);
      EXPECT_TRUE(arena.owns(m.data()));
    }
    m(0, 0) = 1.0;
    EXPECT_DOUBLE_EQ(m(0, 0), 1.0);
  }
}

/// @brief Blaze's move assignment keeps the destination's allocator while stealing the source's
/// buffer, so a value moved out of a scope must still be released through the scope's resource.
TEST(MathTypes, GivenMoveAcrossScopes_ExpectBufferReleasedByOwningResource) {
  memory_arena outer;
  memory_arena arena;

  {
    const memory_scope outer_scope{&outer};
    dynamic_vector<double> v(8, 0.0);

    {
      const memory_scope arena_scope{&arena};
      dynamic_vector<double> transient(8, 1.0);
      v = std::move(transient);
      EXPECT_TRUE(arena.owns(v.data()));
    }
    EXPECT_DOUBLE_EQ(v[0], 1.0);
  }
}

/// @brief `swap()` exchanges the element buffers but not the allocators.
TEST(MathTypes, GivenSwapAcrossScopes_ExpectBuffersReleasedByOwningResource) {
  memory_arena outer;
  memory_arena arena;

  {
    const memory_scope outer_scope{&outer};
    dynamic_vector<double> a(8, 1.0);

    {
      const memory_scope arena_scope{&arena};
      dynamic_vector<double> b(8, 2.0);
      a.swap(b);
      EXPECT_TRUE(arena.owns(a.data()));
      EXPECT_TRUE(outer.owns(b.data()));
      EXPECT_DOUBLE_EQ(b[0], 1.0);
    }
    EXPECT_DOUBLE_EQ(a[0], 2.0);
  }
}

/// @brief The solvers cache their workspace in `thread_local` storage, which outlives the
/// `memory_scope` a solve runs under. Anything it retained from the arena would be released
/// through the arena at thread exit, long after it died.
TEST(MathSolver, GivenScopedArena_ExpectSolversRetainNothingAfterScopeEnds) {
  memory_arena arena;

  {
    const memory_scope scope{&arena};

    dynamic_matrix<double> h(3, 3, 0.0);
    h(0, 0) = 4.0;
    h(1, 1) = 5.0;
    h(2, 2) = 6.0;

    const dynamic_vector<double> b{1.0, 2.0, 3.0};
    dynamic_vector<double> x;

    EXPECT_TRUE(vortex::math::solve_ldlt(h, b, x));
    EXPECT_TRUE(vortex::math::solve_cholesky(h, b, x));
    EXPECT_TRUE(arena.owns(h.data()));
  }
}

/// @brief A plain blaze type -- no vortex alias, no explicit allocator -- draws from the active
/// scope. This is what the blaze patch buys, and it failing means the patch did not apply.
TEST(MathTypes, GivenPlainBlazeContainers_ExpectActiveScope) {
  memory_arena resource;
  const memory_scope scope{&resource};

  const blaze::DynamicMatrix<double> m(8, 8, 1.0);
  const blaze::DynamicVector<double> v(8, 1.0);

  EXPECT_TRUE(resource.owns(m.data()));
  EXPECT_TRUE(resource.owns(v.data()));
}

}  // namespace
