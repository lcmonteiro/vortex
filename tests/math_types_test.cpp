/// ===============================================================================================
/// @file
/// @brief Unit tests for `vortex::math` matrix/vector types and the `memory_scope_allocator` that
/// ties dynamically-sized blaze containers to `helpers::memory_scope`.
/// ===============================================================================================
#include <gtest/gtest.h>

#include <cstddef>
#include <memory_resource>
#include <set>
#include <utility>

#include "foundation/math/solver.hpp"
#include "foundation/math/types.hpp"
#include "helpers/memory.hpp"

namespace {

using vortex::math::dynamic_matrix;
using vortex::math::dynamic_vector;
using vortex::helpers::memory_scope;

/// @brief Memory resource that records the blocks it handed out, so a test can assert that every
/// block is released through the very resource that produced it.
///
/// Blaze exchanges element buffers between containers without exchanging their allocators, so a
/// buffer can easily end up being released through the wrong resource. That mistake is invisible
/// to a plain allocation counter -- it only shows up as a block being freed by a resource that
/// never allocated it, which is what `foreign_frees` counts.
class tracking_resource : public std::pmr::memory_resource {
 public:
  explicit tracking_resource(std::pmr::memory_resource* const upstream) : upstream_{upstream} {}

  /// @brief Blocks handed out by this resource and not yet returned to it.
  [[nodiscard]] auto live() const -> std::size_t { return live_.size(); }

  std::size_t allocations{0};
  std::size_t foreign_frees{0};

 protected:
  auto do_allocate(std::size_t bytes, std::size_t alignment) -> void* override {
    ++allocations;
    auto* const p = upstream_->allocate(bytes, alignment);
    live_.insert(p);
    return p;
  }

  auto do_deallocate(void* p, std::size_t bytes, std::size_t alignment) -> void override {
    if (0 == live_.erase(p)) {
      // A block this resource never produced. Still forward it: every resource in these tests
      // shares one upstream, so the release itself stays valid and the process stays healthy.
      ++foreign_frees;
    }
    upstream_->deallocate(p, bytes, alignment);
  }

  auto do_is_equal(const std::pmr::memory_resource& other) const noexcept -> bool override {
    return this == &other;
  }

 private:
  std::pmr::memory_resource* upstream_;
  std::set<void*> live_;
};

TEST(MathTypes, DynamicVectorUsesActiveMemoryScope) {
  tracking_resource resource{std::pmr::new_delete_resource()};
  {
    const memory_scope scope{&resource};

    dynamic_vector<double> v(4, 0.0);
    v[0] = 1.0;
    v[3] = 4.0;

    EXPECT_DOUBLE_EQ(v[0], 1.0);
    EXPECT_DOUBLE_EQ(v[3], 4.0);
    EXPECT_GT(resource.allocations, 0U);
  }
  EXPECT_EQ(resource.foreign_frees, 0U);
  EXPECT_EQ(resource.live(), 0U);
}

TEST(MathTypes, DynamicMatrixUsesActiveMemoryScope) {
  tracking_resource resource{std::pmr::new_delete_resource()};
  {
    const memory_scope scope{&resource};

    dynamic_matrix<double> m(2, 2, 0.0);
    m(0, 0) = 1.0;
    m(1, 1) = 2.0;

    EXPECT_DOUBLE_EQ(m(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(m(1, 1), 2.0);
    EXPECT_GT(resource.allocations, 0U);
  }
  EXPECT_EQ(resource.foreign_frees, 0U);
  EXPECT_EQ(resource.live(), 0U);
}

TEST(MathTypes, ScopedAllocatorTracksNestedScopes) {
  tracking_resource outer{std::pmr::new_delete_resource()};
  tracking_resource inner{std::pmr::new_delete_resource()};

  const memory_scope outer_scope{&outer};
  { const dynamic_vector<double> v(4, 0.0); }
  EXPECT_GT(outer.allocations, 0U);
  EXPECT_EQ(inner.allocations, 0U);

  {
    const memory_scope inner_scope{&inner};
    const dynamic_vector<double> v(4, 0.0);
  }
  EXPECT_GT(inner.allocations, 0U);

  EXPECT_EQ(outer.foreign_frees, 0U);
  EXPECT_EQ(inner.foreign_frees, 0U);
  EXPECT_EQ(inner.live(), 0U);
}

/// @brief `resize()` past the current capacity allocates the replacement buffer with a freshly
/// default-constructed allocator and then swaps only the buffers, leaving each container with its
/// original allocator. This is the path `block_graph_solver::build_structure()` takes from inside
/// `graph::optimize()`'s scope, on vectors constructed outside it.
TEST(MathTypes, GivenResizeInsideScope_ExpectBuffersReleasedByOwningResource) {
  tracking_resource outer{std::pmr::new_delete_resource()};
  tracking_resource arena{std::pmr::new_delete_resource()};

  {
    const memory_scope outer_scope{&outer};
    dynamic_vector<double> v(8, 0.0);

    {
      const memory_scope arena_scope{&arena};
      v.resize(64, false);
      EXPECT_GT(arena.allocations, 0U);
    }
    v[0] = 1.0;
    EXPECT_DOUBLE_EQ(v[0], 1.0);
  }

  EXPECT_EQ(outer.foreign_frees, 0U);
  EXPECT_EQ(arena.foreign_frees, 0U);
  EXPECT_EQ(outer.live(), 0U);
  EXPECT_EQ(arena.live(), 0U);
}

/// @brief Same reasoning for a matrix, which `build_structure()` also resizes.
TEST(MathTypes, GivenMatrixResizeInsideScope_ExpectBuffersReleasedByOwningResource) {
  tracking_resource outer{std::pmr::new_delete_resource()};
  tracking_resource arena{std::pmr::new_delete_resource()};

  {
    const memory_scope outer_scope{&outer};
    dynamic_matrix<double> m(4, 4, 0.0);

    {
      const memory_scope arena_scope{&arena};
      m.resize(32, 32, false);
      EXPECT_GT(arena.allocations, 0U);
    }
    m(0, 0) = 1.0;
    EXPECT_DOUBLE_EQ(m(0, 0), 1.0);
  }

  EXPECT_EQ(outer.foreign_frees, 0U);
  EXPECT_EQ(arena.foreign_frees, 0U);
  EXPECT_EQ(outer.live(), 0U);
  EXPECT_EQ(arena.live(), 0U);
}

/// @brief Blaze's move assignment keeps the destination's allocator while stealing the source's
/// buffer, so a value moved out of a scope must still be released through the scope's resource.
TEST(MathTypes, GivenMoveAcrossScopes_ExpectBufferReleasedByOwningResource) {
  tracking_resource outer{std::pmr::new_delete_resource()};
  tracking_resource arena{std::pmr::new_delete_resource()};

  {
    const memory_scope outer_scope{&outer};
    dynamic_vector<double> v(8, 0.0);

    {
      const memory_scope arena_scope{&arena};
      dynamic_vector<double> transient(8, 1.0);
      v = std::move(transient);
    }
    EXPECT_DOUBLE_EQ(v[0], 1.0);
  }

  EXPECT_EQ(outer.foreign_frees, 0U);
  EXPECT_EQ(arena.foreign_frees, 0U);
  EXPECT_EQ(outer.live(), 0U);
  EXPECT_EQ(arena.live(), 0U);
}

/// @brief `swap()` exchanges the element buffers but not the allocators.
TEST(MathTypes, GivenSwapAcrossScopes_ExpectBuffersReleasedByOwningResource) {
  tracking_resource outer{std::pmr::new_delete_resource()};
  tracking_resource arena{std::pmr::new_delete_resource()};

  {
    const memory_scope outer_scope{&outer};
    dynamic_vector<double> a(8, 1.0);

    {
      const memory_scope arena_scope{&arena};
      dynamic_vector<double> b(8, 2.0);
      a.swap(b);
      EXPECT_DOUBLE_EQ(b[0], 1.0);
    }
    EXPECT_DOUBLE_EQ(a[0], 2.0);
  }

  EXPECT_EQ(outer.foreign_frees, 0U);
  EXPECT_EQ(arena.foreign_frees, 0U);
  EXPECT_EQ(outer.live(), 0U);
  EXPECT_EQ(arena.live(), 0U);
}

/// @brief The solvers cache their factorization workspace in `thread_local` storage, so it lives
/// until the thread ends. Solving happens inside the caller's `memory_scope` (see
/// `optimization::graph::optimize`), and anything that cache retained from that scope's arena
/// would be released through the arena long after it was destroyed -- a use-after-free at thread
/// exit. Assert the arena gets everything back before it dies.
TEST(MathSolver, GivenScopedArena_ExpectSolversRetainNothingAfterScopeEnds) {
  tracking_resource arena{std::pmr::new_delete_resource()};

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
    EXPECT_GT(arena.allocations, 0U);
  }

  EXPECT_EQ(arena.foreign_frees, 0U);
  EXPECT_EQ(arena.live(), 0U);
}

}  // namespace
