/// ===========================================================================
/// @file
///
/// @brief vortex.graph component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_CONFIG_HPP
#define VORTEX_OPTIMIZATION_GRAPH_CONFIG_HPP

#include "base/graph_config.hpp"
#include "optimization/graph_algorithm_levenberg.hpp"
#include "optimization/graph_solver_block.hpp"
#include "optimization/linear_solver_cholesky.hpp"
#include "optimization/linear_solver_default.hpp"
#include "optimization/linear_solver_pcg.hpp"

namespace vortex::graph::optimization {

/// ===========================================================================
/// @brief DefaultConfig: Defines the default configuration for graph
/// optimization.
/// ===========================================================================
struct DefaultConfig : graph::DefaultConfig {
  ///< Capacity of the backlog buffer.
  static constexpr size_t BacklogCapacity = 3;

  ///< Maximum system capacity (internal matrices and vectors).
  static constexpr size_t SystemCapacity = 0x200U;

  ///< Data type for numerical values.
  using Number = double;

  ///< Default optimization algorithm.
  template <class Graph, class GraphSolver>
  using Algorithm = LevenbergAlgorithm<Graph, GraphSolver>;

  ///< Default graph solver.
  template <class Graph, class LinearSolver>
  using GraphSolver = BlockGraphSolver<Graph, LinearSolver>;

  ///< Default linear solver.
  using LinearSolver = DefaultLinearSolver;
};

}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_CONFIG_HPP
