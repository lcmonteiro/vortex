/// ===========================================================================
/// @file
/// @brief Minimal foundation-level graph fixture (nodes/edges with no
/// optimization payload) used to exercise `vortex::graph::Container` directly.
/// ===========================================================================
#ifndef VORTEX_TESTS_FIXTURES_SIMPLE_GRAPH_HPP
#define VORTEX_TESTS_FIXTURES_SIMPLE_GRAPH_HPP
#include <gtest/gtest.h>

#include "foundation/graph.hpp"

namespace vortex::test {

struct SimpleGraphFixture : public ::testing::Test {
  /// @brief Simple graph definitions.
  struct Edge1;
  struct Edge2;
  struct Node1 : graph::Node<Edge1, Edge2> {
    using Base = graph::Node<Edge1, Edge2>;
    using Base::Base;
  };
  struct Node2 : graph::Node<Edge1> {
    using Base = graph::Node<Edge1>;
    using Base::Base;
  };
  struct Edge1 : graph::Edge<Node1, Node2> {
    using Base = graph::Edge<Node1, Node2>;
    using Base::Base;
  };
  struct Edge2 : graph::Edge<Node1> {
    using Base = graph::Edge<Node1>;
    using Base::Base;
  };
  struct Graph : graph::Container<graph::types<Node1, Node2>, graph::types<Edge1, Edge2>> {
    using Base = graph::Container<graph::types<Node1, Node2>, graph::types<Edge1, Edge2>>;
    using Base::Base;
    using Base::operator=;
  };

 protected:
  void SetUp() override {
    g_.destroy();
    n1_ = g_.build<Node1>(1);
    n2_ = g_.build<Node2>(1);
    n3_ = g_.build<Node1>(2);
    e1_ = g_.build<Edge1>(*n1_, *n2_);
    e2_ = g_.build<Edge2>(*n1_);
  }

  Graph g_{std::pmr::new_delete_resource()};
  graph::OptionalShared<Node1> n1_;
  graph::OptionalShared<Node2> n2_;
  graph::OptionalShared<Node1> n3_;
  graph::OptionalShared<Edge1> e1_;
  graph::OptionalShared<Edge2> e2_;
};

}  // namespace vortex::test

#endif  // VORTEX_TESTS_FIXTURES_SIMPLE_GRAPH_HPP
