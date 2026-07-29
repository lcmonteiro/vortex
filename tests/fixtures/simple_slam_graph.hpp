/// ===========================================================================
/// @file
/// @brief Scalar-generic SLAM problem fixture for vortex.
///
/// This mirrors the classic vortex simple SLAM problem but the user types,
/// `plus()` and `error()` are now generic over the scalar type so that the
/// same cost functions can be evaluated with `double` (residual) and with a
/// dual number (exact Jacobian via forward-mode automatic differentiation).
/// ===========================================================================
#ifndef VORTEX_TESTS_FIXTURES_SIMPLE_SLAM_GRAPH_HPP
#define VORTEX_TESTS_FIXTURES_SIMPLE_SLAM_GRAPH_HPP

#include <type_traits>

#include "optimization/graph.hpp"
#include "optimization/types/position.hpp"

namespace vortex::test {

namespace go = graph::optimization;

using Position = go::types::Position<double>;
using PositionNode = go::types::PositionNode<double>;
using PositionDistanceEdge = go::types::PositionDistanceEdge<double>;
using PositionLocationEdge = go::types::PositionLocationEdge<double>;

using Nodes = go::Nodes<PositionNode>;
using Edges = go::Edges<PositionDistanceEdge, PositionLocationEdge>;
struct SlamGraph : go::Graph<Nodes, Edges> {
  using Base = go::Graph<Nodes, Edges>;
  using Base::Base;
};

}  // namespace vortex::test

#endif  // VORTEX_TESTS_FIXTURES_SIMPLE_SLAM_GRAPH_HPP
