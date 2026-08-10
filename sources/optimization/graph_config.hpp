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
struct default_config : graph::default_config {
  /// @brief Capacity of the backlog buffer.
  static constexpr auto backlog_capacity = std::size_t{3};

  /// @brief Maximum system capacity (internal matrices and vectors).
  static constexpr auto system_capacity = std::size_t{0x200U};

  /// @brief Data type for numerical values.
  using number_type = double;

  /// @brief Default optimization algorithm.
  template <class Graph, class GraphSolver>
  using algorithm_type = levenberg_algorithm<Graph, GraphSolver>;

  /// @brief Default graph solver.
  template <class Graph, class LinearSolver>
  using graph_solver_type = BlockGraphSolver<Graph, LinearSolver>;

  /// @brief Default linear solver.
  using linear_solver_type = DefaultLinearSolver;
};

}  // namespace vortex::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_CONFIG_HPP
