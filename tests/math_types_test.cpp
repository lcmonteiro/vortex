/// ===============================================================================================
/// @file
/// @brief Unit tests for `vortex::math` matrix/vector types and the `memory_scope_allocator` that
/// ties dynamically-sized blaze containers to `helpers::memory_scope`.
/// ===============================================================================================
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory_resource>
#include <type_traits>
#include <utility>

#include "foundation/math/solver.hpp"
#include "foundation/math/types.hpp"
#include "helpers/memory.hpp"
#include "tests/fixtures/arena_memory_resource.hpp"
#include "tests/fixtures/memory_guard.hpp"

namespace {

using vortex::math::dynamic_matrix;
using vortex::math::dynamic_vector;
using vortex::helpers::memory_scope;
using vortex::test::arena_memory_resource;
using vortex::test::memory_guard;

TEST(MathTypes, DynamicVectorUsesActiveMemoryScope) {
  arena_memory_resource resource;
  const memory_scope scope{&resource};

  // Built under a guard, so reaching the heap instead of the arena would throw. The guard is kept
  // to the allocation itself because gtest's macros allocate too.
  auto v = [] { return dynamic_vector<double>(4, 0.0); }();
  {
    const memory_guard guard;
    v.resize(64, false);
  }
  v[0] = 1.0;
  v[63] = 4.0;

  EXPECT_TRUE(resource.owns(v.data()));
  EXPECT_DOUBLE_EQ(v[0], 1.0);
  EXPECT_DOUBLE_EQ(v[63], 4.0);
}

TEST(MathTypes, DynamicMatrixUsesActiveMemoryScope) {
  arena_memory_resource resource;
  const memory_scope scope{&resource};

  dynamic_matrix<double> m(2, 2, 0.0);
  m(0, 0) = 1.0;
  m(1, 1) = 2.0;

  EXPECT_TRUE(resource.owns(m.data()));
  EXPECT_DOUBLE_EQ(m(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(m(1, 1), 2.0);
}

TEST(MathTypes, ScopedAllocatorTracksNestedScopes) {
  arena_memory_resource outer;
  arena_memory_resource inner;

  const memory_scope outer_scope{&outer};
  {
    const dynamic_vector<double> v(4, 0.0);
    EXPECT_TRUE(outer.owns(v.data()));
    EXPECT_FALSE(inner.owns(v.data()));
  }
  {
    const memory_scope inner_scope{&inner};
    const dynamic_vector<double> v(4, 0.0);
    EXPECT_TRUE(inner.owns(v.data()));
    EXPECT_FALSE(outer.owns(v.data()));
  }
}

/// @brief `resize()` past capacity builds the replacement with a fresh allocator and then swaps
/// only the buffers, leaving each container its original one -- the path
/// `block_graph_solver::build_structure()` takes inside `optimize()`'s scope, on vectors
/// constructed outside it.
TEST(MathTypes, GivenResizeInsideScope_ExpectBuffersReleasedByOwningResource) {
  arena_memory_resource outer;
  arena_memory_resource arena;

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
  // Releasing either buffer through the wrong arena would have thrown from the destructors above.
}

/// @brief Same reasoning for a matrix, which `build_structure()` also resizes.
TEST(MathTypes, GivenMatrixResizeInsideScope_ExpectBuffersReleasedByOwningResource) {
  arena_memory_resource outer;
  arena_memory_resource arena;

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
  // Releasing either buffer through the wrong arena would have thrown from the destructors above.
}

/// @brief Blaze's move assignment keeps the destination's allocator while stealing the source's
/// buffer, so a value moved out of a scope must still be released through the scope's resource.
TEST(MathTypes, GivenMoveAcrossScopes_ExpectBufferReleasedByOwningResource) {
  arena_memory_resource outer;
  arena_memory_resource arena;

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
  // `v` now holds the inner arena's buffer; releasing it through `outer` would have thrown.
}

/// @brief `swap()` exchanges the element buffers but not the allocators.
TEST(MathTypes, GivenSwapAcrossScopes_ExpectBuffersReleasedByOwningResource) {
  arena_memory_resource outer;
  arena_memory_resource arena;

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
  // Each buffer ended up in the other container; releasing either through the wrong arena throws.
}

/// @brief The solvers cache their workspace in `thread_local` storage, which outlives the
/// `memory_scope` a solve runs under. Anything that cache retained from the scope's arena would be
/// released through it long after it died -- a use-after-free at thread exit -- so assert the
/// arena gets everything back.
TEST(MathSolver, GivenScopedArena_ExpectSolversRetainNothingAfterScopeEnds) {
  arena_memory_resource arena;

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
  // The arena dies here. Had either solver's thread_local cache kept a block of it, that block
  // would be released through the dead arena at thread exit.
}

/// ===============================================================================================
/// Allocator wiring: what the project's aliases guarantee, and where blaze's own limits start.
/// ===============================================================================================

/// @brief The aliases default onto memory_scope_allocator, so `dynamic_matrix<double>` names
/// exactly the type the arguments spell out -- the parameters are there to be overridden, not to
/// change what the default resolves to.
static_assert(std::is_same_v<dynamic_matrix<double>,
                             blaze::DynamicMatrix<double, vortex::math::column_major,
                                                  vortex::math::memory_scope_allocator<double>>>);
static_assert(std::is_same_v<dynamic_vector<double>,
                             blaze::DynamicVector<double, vortex::math::column_vector,
                                                  vortex::math::memory_scope_allocator<double>>>);

/// @brief ...while an explicitly supplied allocator is still honoured.
static_assert(std::is_same_v<
              dynamic_matrix<double, vortex::math::column_major, blaze::AlignedAllocator<double>>,
              blaze::DynamicMatrix<double, vortex::math::column_major,
                                   blaze::AlignedAllocator<double>>>);

/// @brief Blaze's own spelling is unaffected -- its default template argument cannot be
/// retargeted from outside the library ([temp.param]/12 forbids a second default argument).
static_assert(std::is_same_v<blaze::GetAllocator_t<blaze::DynamicMatrix<double>>,
                             blaze::AlignedAllocator<double>>);

/// @brief Blaze drops a custom allocator when deducing an expression's result type: the containers
/// hardwire `AllocatorType` to `AlignedAllocator` rather than reporting their own `Alloc`, and
/// every arithmetic trait routes through `GetAllocator`. If an upgrade ever fixes it these start
/// failing, which is the signal to revisit the note in foundation/math/memory.hpp.
static_assert(std::is_same_v<blaze::GetAllocator_t<dynamic_matrix<double>>,
                             blaze::AlignedAllocator<double>>,
              "blaze still reports AlignedAllocator for a custom-allocator matrix");
static_assert(std::is_same_v<blaze::MultTrait_t<dynamic_matrix<double>, dynamic_matrix<double>>,
                             blaze::DynamicMatrix<double, vortex::math::column_major,
                                                  blaze::AlignedAllocator<double>>>,
              "blaze still drops the custom allocator from A * B");
static_assert(std::is_same_v<blaze::AddTrait_t<dynamic_matrix<double>, dynamic_matrix<double>>,
                             blaze::DynamicMatrix<double, vortex::math::column_major,
                                                  blaze::AlignedAllocator<double>>>,
              "blaze still drops the custom allocator from A + B");

#ifdef VORTEX_BLAZE_SCOPED_ALLOCATOR
/// @brief With the patch applied, a plain blaze type -- no vortex alias, no explicit allocator --
/// draws from the active scope. That is the whole point of patching, and the one thing no
/// project-side alias can achieve.
TEST(MathTypes, GivenPatchedBlaze_ExpectPlainContainersUseActiveScope) {
  arena_memory_resource resource;
  const memory_scope scope{&resource};

  const blaze::DynamicMatrix<double> m(8, 8, 1.0);
  const blaze::DynamicVector<double> v(8, 1.0);

  EXPECT_TRUE(resource.owns(m.data()));
  EXPECT_TRUE(resource.owns(v.data()));
}
#endif

/// @brief Buffers must satisfy blaze's SIMD alignment, which it checks via `checkAlignment` on
/// every allocation and reports as a bad_alloc when violated.
TEST(MathTypes, GivenAllocatedContainers_ExpectBlazeSimdAlignment) {
  arena_memory_resource resource;
  const memory_scope scope{&resource};

  const dynamic_vector<double> v(37, 1.0);
  const dynamic_matrix<double> m(37, 37, 1.0);

  constexpr auto required = blaze::AlignmentOf_v<double>;
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(v.data()) % required, 0U);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(m.data()) % required, 0U);
}

/// @brief Named containers are scope-allocated, including through the expressions below. What is
/// asserted here is the split: operands and assignment targets go through the scope, and the
/// suite as a whole still completes -- blaze falling back to AlignedAllocator for an intermediate
/// is a heap allocation, not a failure.
TEST(MathTypes, GivenMatrixExpressions_ExpectNamedOperandsUseScope) {
  arena_memory_resource resource;

  {
    const memory_scope scope{&resource};

    dynamic_matrix<double> a(16, 16, 1.0);
    dynamic_matrix<double> b(16, 16, 2.0);
    EXPECT_TRUE(resource.owns(a.data()));
    EXPECT_TRUE(resource.owns(b.data()));

    dynamic_matrix<double> c = a + b;
    EXPECT_DOUBLE_EQ(c(0, 0), 3.0);

    c = a * b;
    EXPECT_DOUBLE_EQ(c(0, 0), 32.0);

    c = blaze::trans(a) * b;
    EXPECT_DOUBLE_EQ(c(0, 0), 32.0);

    // The named result is scope-allocated like the operands; only blaze's own temporaries are not.
    EXPECT_TRUE(resource.owns(c.data()));
  }
}

}  // namespace
