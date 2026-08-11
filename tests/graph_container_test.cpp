/// ===============================================================================================
/// @file
/// @brief Unit tests for `vortex::graph::storage` (build/size/apply/find/
/// unlink/destroy/toggle) using a minimal, optimization-free node/edge graph.
/// ===============================================================================================
#include <gtest/gtest.h>

#include <memory_resource>
#include <utility>

#include "helpers/memory.hpp"
#include "tests/fixtures/simple_graph.hpp"

namespace {

using namespace vortex::test;

TEST_F(SimpleGraphFixture, CheckGraphStructure) {
  auto size = [](auto apply) {
    auto total = size_t{0};
    apply([&](auto) { ++total; });
    return total;
  };
  auto expect = size_t{};
  auto result = size_t{};

  expect = g_.size();
  result = size([&, this](auto f) { g_.apply(f); });
  EXPECT_EQ(expect, result);

  expect = g_.size<Graph::node_list>();
  result = size([&, this](auto f) { g_.apply<Graph::node_list>(f); });
  EXPECT_EQ(expect, result);

  expect = g_.size<Graph::edge_list>();
  result = size([&, this](auto f) { g_.apply<Graph::edge_list>(f); });
  EXPECT_EQ(expect, result);

  expect = g_.size<vortex::graph::items<Node2, Edge1>>();
  result = size([&, this](auto f) { g_.apply<vortex::graph::items<Node2, Edge1>>(f); });
  EXPECT_EQ(expect, result);
}

TEST_F(SimpleGraphFixture, CheckGraphStructureWhenDuplicatedKeys) {
  auto nodes_count = g_.size<Graph::node_list>();

  std::ignore = g_.build<Node1>(1);
  EXPECT_EQ(g_.size<Graph::edge_list>(), 0);
  EXPECT_EQ(g_.size<Graph::node_list>(), nodes_count);
}

TEST_F(SimpleGraphFixture, CheckGraphFind) {
  auto found = g_.find<Node1>([this](auto& node) { return node == n1_.value(); });
  EXPECT_EQ(found.value(), n1_.value());
}

TEST_F(SimpleGraphFixture, CheckGraphUnlink) {
  auto nodes_count = g_.size<Graph::node_list>();
  auto edges_count = g_.size<Graph::edge_list>();

  g_.unlink<Node1, Edge1>();
  EXPECT_EQ(g_.size<Graph::edge_list>(), --edges_count);
  EXPECT_EQ(g_.size<Graph::node_list>(), nodes_count);

  g_.unlink<Node2>();
  EXPECT_EQ(g_.size<Graph::edge_list>(), edges_count);
  EXPECT_EQ(g_.size<Graph::node_list>(), nodes_count);

  g_.unlink<Node1>(1);
  EXPECT_EQ(g_.size<Graph::edge_list>(), --edges_count);
  EXPECT_EQ(g_.size<Graph::node_list>(), nodes_count);
}

TEST_F(SimpleGraphFixture, CheckGraphNodeUnlink) {
  auto size = [](auto node) {
    auto total = size_t{0};
    node->apply([&](auto) { ++total; });
    return total;
  };
  ASSERT_EQ(size(n1_.value()), 2);
  ASSERT_EQ(size(n2_.value()), 1);

  g_.unlink<Node1, Edge2>();
  EXPECT_EQ(size(n1_.value()), 1);
  EXPECT_EQ(size(n2_.value()), 1);

  g_.unlink<Node2>();
  EXPECT_EQ(size(n1_.value()), 0);
  EXPECT_EQ(size(n2_.value()), 0);
}

TEST_F(SimpleGraphFixture, CheckGraphDestroy) {
  auto nodes_count = g_.size<Graph::node_list>();
  auto edges_count = g_.size<Graph::edge_list>();

  g_.destroy<Node2>(1);
  EXPECT_EQ(g_.size<Graph::edge_list>(), --edges_count);
  EXPECT_EQ(g_.size<Graph::node_list>(), --nodes_count);

  g_.destroy<Node1>(1);
  EXPECT_EQ(g_.size<Graph::edge_list>(), --edges_count);
  EXPECT_EQ(g_.size<Graph::node_list>(), --nodes_count);
}

TEST_F(SimpleGraphFixture, CheckGraphDestroyAll) {
  g_.destroy();
  EXPECT_EQ(g_.size<Graph::edge_list>(), 0);
  EXPECT_EQ(g_.size<Graph::node_list>(), 0);
}

TEST_F(SimpleGraphFixture, CheckGraphDestroyAllEnableDisable) {
  g_.destroy(g_.disabled);
  g_.destroy(g_.enabled);
  EXPECT_EQ(g_.size<Graph::edge_list>(), 0);
  EXPECT_EQ(g_.size<Graph::node_list>(), 0);
}

TEST_F(SimpleGraphFixture, CheckGraphDestroyIf_FirstElement) {
  g_.destroy<Node1>([this](auto& node) { return node == n1_.value(); });
  EXPECT_EQ(g_.size<Graph::node_list>(), 2);
}

TEST_F(SimpleGraphFixture, CheckGraphDestroyIf_SecondElement) {
  // We start with the same 3 nodes that were SetUp(), two of type Node1
  // and one Node2
  EXPECT_EQ(g_.size<Graph::node_list>(), 3);

  // We create one more node of type Node1, for a total of 4 nodes
  auto n1_2 = g_.build<Node1>(3);
  EXPECT_EQ(g_.size<Graph::node_list>(), 4);

  // We remove one of the nodes of type Node1
  g_.destroy<Node1>([n1_2](auto& node) { return node == n1_2; });
  EXPECT_EQ(g_.size<Graph::node_list>(), 3);
}

TEST_F(SimpleGraphFixture, GivenDifferentTypes_ExpectNodeToggle) {
  g_.toggle<Node1>(Graph::enabled);
  ASSERT_TRUE((*n1_)->disable());
  ASSERT_FALSE((*n2_)->disable());

  g_.toggle<Node1>(Graph::disabled);
  EXPECT_FALSE((*n1_)->disable());
  EXPECT_FALSE((*n2_)->disable());
}

TEST_F(SimpleGraphFixture, GivenNodesToggle_ExpectEnabledCountChange) {
  auto enabled{g_.size<Graph::node_list>(g_.enabled)};
  auto disabled{g_.size<Graph::node_list>(g_.disabled)};

  g_.toggle<Node1>(Graph::enabled);

  EXPECT_LE(g_.size<Graph::node_list>(g_.enabled), enabled);
  EXPECT_GE(g_.size<Graph::node_list>(g_.disabled), disabled);
}

TEST_F(SimpleGraphFixture, GivenEdgesToggle_ExpectEnabledCountChange) {
  auto enabled{g_.size<Graph::edge_list>(g_.enabled)};
  auto disabled{g_.size<Graph::edge_list>(g_.disabled)};

  g_.toggle<Edge1>(Graph::enabled);

  EXPECT_LE(g_.size<Graph::edge_list>(g_.enabled), enabled);
  EXPECT_GE(g_.size<Graph::edge_list>(g_.disabled), disabled);
}

TEST_F(SimpleGraphFixture, GivenDifferentKeys_ExpectNodeToggle) {
  g_.toggle<Node1>(1, Graph::enabled);
  ASSERT_TRUE((*n1_)->disable());
  ASSERT_FALSE((*n3_)->disable());

  g_.toggle<Node1>(2, Graph::enabled);
  ASSERT_TRUE((*n1_)->disable());
  ASSERT_TRUE((*n3_)->disable());

  g_.toggle<Node1>(1, Graph::disabled);
  ASSERT_FALSE((*n1_)->disable());
  ASSERT_TRUE((*n3_)->disable());

  g_.toggle<Node1>(2, Graph::disabled);
  EXPECT_FALSE((*n1_)->disable());
  EXPECT_FALSE((*n3_)->disable());
}

TEST_F(SimpleGraphFixture, GivenDifferentTypes_ExpectEdgeToggle) {
  g_.toggle<Edge1>(Graph::enabled);
  ASSERT_TRUE((*e1_)->disable());
  ASSERT_FALSE((*e2_)->disable());

  g_.toggle<Edge1>(Graph::disabled);
  EXPECT_FALSE((*e1_)->disable());
  EXPECT_FALSE((*e2_)->disable());
}

TEST_F(SimpleGraphFixture, GivenDifferentInstances_ExpectEdgeToggle) {
  g_.toggle<Edge1>(*e1_, Graph::enabled);
  ASSERT_TRUE((*e1_)->disable());
  ASSERT_FALSE((*e2_)->disable());

  g_.toggle<Edge2>(*e2_, Graph::enabled);
  ASSERT_TRUE((*e1_)->disable());
  ASSERT_TRUE((*e2_)->disable());

  g_.toggle<Edge1>(*e1_, Graph::disabled);
  ASSERT_FALSE((*e1_)->disable());
  ASSERT_TRUE((*e2_)->disable());

  g_.toggle<Edge2>(*e2_, Graph::disabled);
  EXPECT_FALSE((*e1_)->disable());
  EXPECT_FALSE((*e2_)->disable());
}

TEST_F(SimpleGraphFixture, GivenCondition_ExpectNodeToggle) {
  g_.toggle<Node1>([](const auto& node) { return (node->N_TYPES == 2) ? true : false; },
                   Graph::enabled);
  EXPECT_TRUE((*n1_)->disable());
  EXPECT_FALSE((*n2_)->disable());

  g_.toggle<Node2>([](const auto& node) { return (node->N_TYPES == 1) ? true : false; },
                   Graph::enabled);
  EXPECT_TRUE((*n1_)->disable());
  EXPECT_TRUE((*n2_)->disable());
}

TEST_F(SimpleGraphFixture, GivenCondition_ExpectEdgeToggle) {
  g_.toggle<Edge1>([](const auto& edge) { return (edge->n_nodes == 2) ? true : false; },
                   Graph::enabled);
  EXPECT_TRUE((*e1_)->disable());
  EXPECT_FALSE((*e2_)->disable());

  g_.toggle<Edge2>([](const auto& edge) { return (edge->n_nodes == 1) ? true : false; },
                   Graph::enabled);
  EXPECT_TRUE((*e1_)->disable());
  EXPECT_TRUE((*e2_)->disable());
}

/// @brief Verifies shared copy assignment for base graph types.
TEST_F(SimpleGraphFixture, GivenSharedObjects_ExpectCopyAssignment) {
  auto n_copy = *n1_;
  auto n_copy2 = *n2_;
  auto n_target = *n3_;
  n_target = n_copy;
  EXPECT_EQ(n_target.operator->(), n_copy.operator->());

  n_copy2 = *n2_;
  EXPECT_EQ(n_copy2.operator->(), (*n2_).operator->());

  auto e_copy = *e1_;
  auto e_target = e_copy;
  e_target = e_copy;
  EXPECT_EQ(e_target.operator->(), e_copy.operator->());

  auto e2_copy = *e2_;
  auto e2_target = e2_copy;
  e2_target = e2_copy;
  EXPECT_EQ(e2_target.operator->(), e2_copy.operator->());
}

/// @brief Verifies shared move assignment for base graph types: the target
/// steals the source's managed pointer.
TEST_F(SimpleGraphFixture, GivenSharedObjects_ExpectMoveAssignment) {
  auto n_source = *n1_;
  auto* const n_source_ptr = n_source.operator->();
  auto n_target = *n3_;
  ASSERT_NE(n_target.operator->(), n_source_ptr);

  n_target = std::move(n_source);

  EXPECT_EQ(n_target.operator->(), n_source_ptr);
}

/// @brief Verifies bounded_memory_resource::do_is_equal via PMR interface.
TEST_F(SimpleGraphFixture, GivenMemoryResource_ExpectEqualityComparison) {
  auto* res = std::pmr::new_delete_resource();
  vortex::helpers::bounded_memory_resource<32768> r1(res);
  vortex::helpers::bounded_memory_resource<32768> r2(res);
  std::pmr::memory_resource& mr1 = r1;
  std::pmr::memory_resource& mr2 = r2;

  EXPECT_TRUE(mr1 == mr1);
  EXPECT_FALSE(mr1 == mr2);
}

/// @brief Verifies bounded_memory_resource::do_allocate and do_deallocate.
TEST_F(SimpleGraphFixture, GivenMemoryResource_ExpectAllocateAndDeallocate) {
  auto* res = std::pmr::new_delete_resource();
  vortex::helpers::bounded_memory_resource<1024> bounded(res);
  std::pmr::memory_resource& mr = bounded;

  void* ptr = mr.allocate(512, alignof(std::max_align_t));
  ASSERT_NE(ptr, nullptr);
  mr.deallocate(ptr, 512, alignof(std::max_align_t));
}

}  // namespace
