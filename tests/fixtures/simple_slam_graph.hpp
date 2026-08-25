/// ===============================================================================================
/// @file
/// @brief Scalar-generic SLAM problem fixture for vortex.
///
/// This mirrors the classic vortex simple SLAM problem but the user types,
/// `plus()` and `error()` are now generic over the scalar type so that the
/// same cost functions can be evaluated with `double` (residual) and with a
/// dual number (exact Jacobian via forward-mode automatic differentiation).
/// ===============================================================================================
#ifndef VORTEX_TESTS_FIXTURES_SIMPLE_SLAM_GRAPH_HPP
#define VORTEX_TESTS_FIXTURES_SIMPLE_SLAM_GRAPH_HPP

#include <type_traits>

#include "vortex/optimization/graph.hpp"
#include "vortex/optimization/types/position.hpp"

namespace vortex::test {

namespace vx = vortex::optimization;

using Position = vx::types::position<double>;
using PositionNode = vx::types::position_node<double>;
using PositionDistanceEdge = vx::types::position_distance_edge<double>;
using PositionLocationEdge = vx::types::position_location_edge<double>;

using Nodes = vx::nodes<PositionNode>;
using Edges = vx::edges<PositionDistanceEdge, PositionLocationEdge>;
struct SlamGraph : vx::graph<Nodes, Edges> {
  using Base = vx::graph<Nodes, Edges>;
  using Base::Base;
};

}  // namespace vortex::test

#endif  // VORTEX_TESTS_FIXTURES_SIMPLE_SLAM_GRAPH_HPP
