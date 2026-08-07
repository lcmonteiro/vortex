/// ===============================================================================================
/// @file
/// @brief Unit tests for `vortex::graph::VectorSet`, the insertion-ordered
/// unique-element container used for the graph engine's edge sets.
/// ===============================================================================================
#include "foundation/types/vector_set.hpp"

#include <gtest/gtest.h>

#include <memory_resource>
#include <vector>

#include "helpers/handle.hpp"

namespace {

using vortex::graph::VectorSet;
using vortex::helpers::handle;

std::vector<int> Contents(const VectorSet<int>& set) {
  return std::vector<int>{set.begin(), set.end()};
}

/// @brief The defining property: iteration follows insertion order, never the
/// natural (sorted) order of the values. This is what makes the container
/// deterministic and independent of element addresses.
TEST(VectorSet, GivenValuesInsertedOutOfSortOrder_ExpectIterationFollowsInsertionOrder) {
  VectorSet<int> set{std::pmr::new_delete_resource()};

  set.insert(30);
  set.insert(10);
  set.insert(20);

  EXPECT_EQ(Contents(set), (std::vector<int>{30, 10, 20}));
}

TEST(VectorSet, GivenPresentAndAbsentValues_ExpectFindLocatesOnlyPresent) {
  VectorSet<int> set{std::pmr::new_delete_resource()};
  set.insert(42);
  set.insert(43);

  EXPECT_NE(set.find(42), set.end());
  EXPECT_EQ(set.find(44), set.end());
}

TEST(VectorSet, GivenValue_ExpectEraseByValueRemovesItAndReportsCount) {
  VectorSet<int> set{std::pmr::new_delete_resource()};
  set.insert(10);
  set.insert(20);
  set.insert(30);

  EXPECT_EQ(set.erase(20), 1U);
  EXPECT_EQ(set.erase(44), 0U);

  EXPECT_EQ(Contents(set), (std::vector<int>{10, 30}));
  EXPECT_EQ(set.size(), 2U);
}

TEST(VectorSet, GivenIterator_ExpectEraseReturnsNextAndPreservesOrder) {
  VectorSet<int> set{std::pmr::new_delete_resource()};
  set.insert(10);
  set.insert(20);
  set.insert(30);

  const auto next = set.erase(set.find(20));

  EXPECT_EQ(*next, 30);
  EXPECT_EQ(Contents(set), (std::vector<int>{10, 30}));
}

TEST(VectorSet, GivenRange_ExpectInsertAppendsPreservingOrder) {
  VectorSet<int> set{std::pmr::new_delete_resource()};
  set.insert(1);

  const std::vector<int> more{2, 3, 4};
  set.insert(more.begin(), more.end());

  EXPECT_EQ(Contents(set), (std::vector<int>{1, 2, 3, 4}));
}

TEST(VectorSet, GivenNewValue_ExpectEmplaceConstructsAtEndAndReturnsIteratorAndTrue) {
  VectorSet<int> set{std::pmr::new_delete_resource()};
  set.insert(1);

  const auto result = set.emplace(2);

  EXPECT_EQ(*result.first, 2);
  EXPECT_TRUE(result.second);
  EXPECT_EQ(Contents(set), (std::vector<int>{1, 2}));
  EXPECT_EQ(set.size(), 2U);
}

TEST(VectorSet, GivenTwoSets_ExpectSwapExchangesContents) {
  VectorSet<int> left{std::pmr::new_delete_resource()};
  VectorSet<int> right{std::pmr::new_delete_resource()};
  left.insert(1);
  left.insert(2);
  right.insert(9);

  left.swap(right);

  EXPECT_EQ(Contents(left), (std::vector<int>{9}));
  EXPECT_EQ(Contents(right), (std::vector<int>{1, 2}));
}

/// @brief The graph stores `vortex::helpers::handle` elements. Verify that the
/// move-assignment based erase-shift and the range insert work for such a type
/// and keep insertion order.
TEST(VectorSet, GivenSharedElements_ExpectEraseAndRangeInsertKeepOrder) {
  auto* const memory = std::pmr::new_delete_resource();
  VectorSet<handle<int>> set{memory};

  const handle<int> first{memory, 10};
  const handle<int> second{memory, 20};
  const handle<int> third{memory, 30};
  set.insert(first);
  set.insert(second);
  set.insert(third);

  set.erase(second);

  std::vector<int> values;
  for (const auto& element : set) {
    values.push_back(*element);
  }
  EXPECT_EQ(values, (std::vector<int>{10, 30}));
}

TEST(VectorSet, GivenNewValue_ExpectInsertReturnsIteratorToItAndTrue) {
  VectorSet<int> set{std::pmr::new_delete_resource()};

  const auto result = set.insert(42);

  EXPECT_EQ(*result.first, 42);
  EXPECT_TRUE(result.second);
  EXPECT_EQ(set.size(), 1U);
}

TEST(VectorSet, GivenPopulatedSet_ExpectClearEmptiesIt) {
  VectorSet<int> set{std::pmr::new_delete_resource()};
  set.insert(1);
  set.insert(2);

  set.clear();

  EXPECT_EQ(set.size(), 0U);
  EXPECT_EQ(set.begin(), set.end());
}

TEST(VectorSet, GivenLastElement_ExpectEraseByIteratorReturnsEnd) {
  VectorSet<int> set{std::pmr::new_delete_resource()};
  set.insert(10);
  set.insert(20);

  const auto next = set.erase(set.find(20));

  EXPECT_EQ(next, set.end());
  EXPECT_EQ(Contents(set), (std::vector<int>{10}));
}

TEST(VectorSet, GivenFirstElement_ExpectEraseByValueKeepsRestInOrder) {
  VectorSet<int> set{std::pmr::new_delete_resource()};
  set.insert(10);
  set.insert(20);
  set.insert(30);

  EXPECT_EQ(set.erase(10), 1U);

  EXPECT_EQ(Contents(set), (std::vector<int>{20, 30}));
}

TEST(VectorSet, GivenAllElementsErasedByValue_ExpectEmpty) {
  VectorSet<int> set{std::pmr::new_delete_resource()};
  set.insert(1);
  set.insert(2);

  EXPECT_EQ(set.erase(1), 1U);
  EXPECT_EQ(set.erase(2), 1U);

  EXPECT_EQ(set.size(), 0U);
  EXPECT_EQ(set.begin(), set.end());
}

TEST(VectorSet, GivenEmptyAndPopulatedSets_ExpectSwapExchangesContents) {
  VectorSet<int> empty{std::pmr::new_delete_resource()};
  VectorSet<int> full{std::pmr::new_delete_resource()};
  full.insert(1);
  full.insert(2);

  empty.swap(full);

  EXPECT_EQ(Contents(empty), (std::vector<int>{1, 2}));
  EXPECT_EQ(full.size(), 0U);
}

TEST(VectorSet, GivenSharedElement_ExpectEmplaceConstructsInPlace) {
  auto* const memory = std::pmr::new_delete_resource();
  VectorSet<handle<int>> set{memory};

  const auto result = set.emplace(memory, 99);

  EXPECT_EQ(**result.first, 99);
  EXPECT_TRUE(result.second);
  EXPECT_EQ(set.size(), 1U);
}

}  // namespace
