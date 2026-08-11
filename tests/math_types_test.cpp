/// ===============================================================================================
/// @file
/// @brief Unit tests for `vortex::math` matrix/vector types and the `memory_scope_allocator` that
/// ties dynamically-sized blaze containers to `helpers::memory_scope`.
/// ===============================================================================================
#include <gtest/gtest.h>

#include <cstddef>
#include <memory_resource>

#include "foundation/math/types.hpp"
#include "helpers/memory.hpp"

namespace {

using vortex::math::dynamic_matrix;
using vortex::math::dynamic_vector;

/// @brief Memory resource that counts how many allocations it served.
class counting_resource : public std::pmr::memory_resource {
 public:
  explicit counting_resource(std::pmr::memory_resource* const upstream) : upstream_{upstream} {}

  std::size_t allocations{0};

 protected:
  auto do_allocate(std::size_t bytes, std::size_t alignment) -> void* override {
    ++allocations;
    return upstream_->allocate(bytes, alignment);
  }
  auto do_deallocate(void* p, std::size_t bytes, std::size_t alignment) -> void override {
    upstream_->deallocate(p, bytes, alignment);
  }
  auto do_is_equal(const std::pmr::memory_resource& other) const noexcept -> bool override {
    return this == &other;
  }

 private:
  std::pmr::memory_resource* upstream_;
};

TEST(MathTypes, DynamicVectorUsesActiveMemoryScope) {
  counting_resource resource{std::pmr::new_delete_resource()};
  const vortex::helpers::memory_scope scope{&resource};

  dynamic_vector<double> v(4, 0.0);
  v[0] = 1.0;
  v[3] = 4.0;

  EXPECT_DOUBLE_EQ(v[0], 1.0);
  EXPECT_DOUBLE_EQ(v[3], 4.0);
  EXPECT_GT(resource.allocations, 0U);
}

TEST(MathTypes, DynamicMatrixUsesActiveMemoryScope) {
  counting_resource resource{std::pmr::new_delete_resource()};
  const vortex::helpers::memory_scope scope{&resource};

  dynamic_matrix<double> m(2, 2, 0.0);
  m(0, 0) = 1.0;
  m(1, 1) = 2.0;

  EXPECT_DOUBLE_EQ(m(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(m(1, 1), 2.0);
  EXPECT_GT(resource.allocations, 0U);
}

TEST(MathTypes, ScopedAllocatorTracksNestedScopes) {
  counting_resource outer{std::pmr::new_delete_resource()};
  counting_resource inner{std::pmr::new_delete_resource()};

  const vortex::helpers::memory_scope outer_scope{&outer};
  { dynamic_vector<double> v(4, 0.0); }
  EXPECT_GT(outer.allocations, 0U);
  EXPECT_EQ(inner.allocations, 0U);

  {
    const vortex::helpers::memory_scope inner_scope{&inner};
    dynamic_vector<double> v(4, 0.0);
  }
  EXPECT_GT(inner.allocations, 0U);
}

}  // namespace
