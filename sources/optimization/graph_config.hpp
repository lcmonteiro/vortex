/// ===============================================================================================
/// @file
///
/// @brief vortex.graph component
/// ===============================================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_CONFIG_HPP
#define VORTEX_OPTIMIZATION_GRAPH_CONFIG_HPP

#include <cstddef>

#include "foundation/graph/config.hpp"
#include "optimization/graph_algorithm_levenberg.hpp"
#include "optimization/graph_solver_block.hpp"
#include "optimization/linear_solver_cholesky.hpp"
#include "optimization/linear_solver_default.hpp"
#include "optimization/linear_solver_pcg.hpp"

namespace vortex::optimization {

/// ===============================================================================================
/// @brief Default configuration for graph optimization.
/// ===============================================================================================
struct DefaultConfig : graph::default_config {
  /// @brief Capacity of the backlog buffer.
  static constexpr auto BacklogCapacity = std::size_t{3};

  /// @brief Maximum system capacity (internal matrices and vectors).
  static constexpr auto SystemCapacity = std::size_t{0x200U};

  /// @brief Data type for numerical values.
  using Number = double;

  /// @brief Default optimization algorithm.
  template <class Graph, class GraphSolver>
  using Algorithm = levenberg_algorithm<Graph, GraphSolver>;

  /// @brief Default graph solver.
  template <class Graph, class LinearSolver>
  using GraphSolver = BlockGraphSolver<Graph, LinearSolver>;

  /// @brief Default linear solver.
  using LinearSolver = DefaultLinearSolver;
};

}  // namespace vortex::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_CONFIG_HPP
