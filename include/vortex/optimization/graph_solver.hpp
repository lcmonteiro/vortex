/// ===============================================================================================
/// @file
///
/// @brief vortex.graph component
/// ===============================================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_SOLVER_HPP
#define VORTEX_OPTIMIZATION_GRAPH_SOLVER_HPP

namespace vortex::optimization {
/// ===============================================================================================
/// @brief A templated class that provides solver functionality for a graph using a linear solver.
///
/// @tparam Graph The graph type to be solved (can be any graph type, as long as it is compatible).
/// @tparam LinearSolver The linear solver type used to perform solving
/// operations on the graph.
/// ===============================================================================================
template <class Graph, class LinearSolver>
class graph_solver {
 public:
  /// @brief Constructor of the graph_solver.
  explicit graph_solver(Graph& graph) : graph_{graph} {}

  /// @brief Deleted copy constructor and defaulted move constructor.
  graph_solver(const graph_solver&) = delete;
  graph_solver(graph_solver&&) = default;
  ~graph_solver() = default;

  /// @brief Deleted copy assignment operator and defaulted move assignment operator.
  auto operator=(const graph_solver&) -> graph_solver& = delete;
  auto operator=(graph_solver&&) -> graph_solver& = delete;

  /// @brief Initializes the graph_solver.
  ///
  /// @param bool A boolean parameter, which could be used to specify additional
  /// initialization options (optional usage depending on the solver
  /// implementation).
  /// @return Returns a boolean indicating the success (true) or failure (false)
  /// of the initialization.
  auto init(bool) -> bool { return lsolver_.init(); }

 protected:
  Graph& graph_;
  LinearSolver lsolver_;
};

}  // namespace vortex::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_SOLVER_HPP
