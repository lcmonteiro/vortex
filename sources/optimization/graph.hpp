/// ===============================================================================================
/// @file
///
/// @brief vortex.graph component
/// ===============================================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_HPP
#define VORTEX_OPTIMIZATION_GRAPH_HPP
#include <cstddef>

#include "foundation/graph.hpp"
#include "foundation/math.hpp"
#include "helpers/expected.hpp"
#include "helpers/memory.hpp"
#include "optimization/graph_config.hpp"
#include "optimization/graph_edge.hpp"  // IWYU pragma: export
#include "optimization/graph_node.hpp"  // IWYU pragma: export

namespace vortex::optimization {
using graph::handle;
using graph::optional;

/// ===============================================================================================
/// @brief A templated Graph class implementing optimization algorithms.
///
/// @tparam Nodes The collection of nodes in the graph.
/// @tparam Edges The collection of edges in the graph.
/// @tparam Config The configuration class that defines various types and
/// parameters for the graph (default is `DefaultConfig`).
/// ===============================================================================================
template <class Nodes, class Edges, class Config = DefaultConfig>
class graph : public vortex::graph::storage<Nodes, Edges, Config> {
  /// @brief Helper alias types.
  using linear_solver_type = typename Config::LinearSolver;
  using graph_solver_type = typename Config::template GraphSolver<graph, linear_solver_type>;
  using algorithm_type = typename Config::template Algorithm<graph, graph_solver_type>;

 public:
  static constexpr auto kSystemCapacity = std::size_t{Config::SystemCapacity};

  using base_type = vortex::graph::storage<Nodes, Edges, Config>;
  using key_type = typename Config::key_type;
  using number_type = typename Config::Number;
  using vector_type = math::dynamic_vector<number_type>;
  using enabled_type = typename base_type::enabled_type;
  using disabled_type = typename base_type::disabled_type;

  /// @brief Graph main constructor.
  /// @param memory_resource The memory resource used for allocating memory.
  explicit graph(std::pmr::memory_resource* const memory_resource)
      : base_type{memory_resource}, algorithm_{*this}, optimizer_revision_{} {}

  /// @brief Node and edge construction is inherited as-is from the base graph --
  /// optimization::graph adds no extra construction arguments.
  using base_type::build;

  /// @brief Runs the graph optimization algorithm.
  /// @param iterations The number of iterations to run.
  /// @param reset Whether to reset the algorithm state.
  /// @return The number of completed iterations or an unexpected error.
  auto optimize(std::size_t iterations, bool reset = true)
      -> helpers::expected<std::size_t, algorithm_error> {
    // Route transient dual-number (Jacobian) allocations through the graph's
    // memory arena for the duration of the optimization.
    const helpers::memory_scope scope{this->memory()};

    // Special case: if no iterations are requested, return immediately.
    if (0 == iterations) {
      return 0;
    }

    // Check if the graph has changed since the last optimization.
    const auto graph_changed = not this->revision()->equal(optimizer_revision_);
    const auto result_init = algorithm_.init(reset or graph_changed);
    if (not result_init) {
      return helpers::unexpected(result_init.error());
    }

    // Update the optimizer revision to match the current graph revision.
    optimizer_revision_.reset(*this->revision());

    // Special case: if only one iteration is requested, perform it and return.
    if (1 == iterations) {
      auto result_solve = algorithm_.template solve<true, true>();
      if (not result_solve) {
        return helpers::unexpected(result_solve.error());
      }
      return iterations;
    }

    // Perform the first iteration, which may have special initialization logic.
    auto result_first = algorithm_.template solve<true, false>();
    if (not result_first) {
      return helpers::unexpected(result_first.error());
    }
    if (result_first.value()) {
      return 1;
    }

    // Perform the intermediate iterations, which may have different logic than the first and last.
    for (std::size_t it = 1; it < (iterations - 1); ++it) {
      auto result_next = algorithm_.template solve<false, false>();
      if (not result_next) {
        return helpers::unexpected(result_next.error());
      }
      if (result_next.value()) {
        return it;
      }
    }

    // Perform the last iteration, which may have special finalization logic.
    auto result_last = algorithm_.template solve<false, true>();
    if (not result_last) {
      return helpers::unexpected(result_last.error());
    }

    return iterations;
  }

  /// @defgroup estimations_operations Estimations Operations
  /// @brief Functions for managing node estimations in the graph.
  ///
  /// This group includes functions to push, pull and revert estimations.
  auto push() -> void {
    ForEach<Nodes>(*this, [](auto& node) { node->push(); }, enabled_type{});
  }
  auto pull() -> void {
    ForEach<Nodes>(*this, [](auto& node) { node->pull(); }, enabled_type{});
  }
  auto revert(std::size_t n = 1) -> void {
    ForEach<Nodes>(*this, [&](auto& node) { node->revert(n); }, enabled_type{});
  }

  /// @brief Computes cached chi2 of the active portion of the graph.
  auto chi2() const -> number_type {
    number_type chi = 0;
    ForEach<Edges>(*this, [&](auto& edge) { chi += edge->chi2(); }, enabled_type{});
    return chi;
  }

  /// @brief Updates cached edge quantities for all active edges.
  auto update_edges() -> void {
    ForEach<Edges>(*this, [](auto& edge) { edge->update(); }, enabled_type{});
  }

  /// @brief Updates the active nodes from the stacked state increment vector.
  auto update_nodes(const vector_type& delta) -> void {
    std::size_t idx = 0;
    ForEach<Nodes>(
        *this,
        [&](auto& node) {
          node->update(math::subvector(delta, idx, node->dimension()));
          idx += node->dimension();
        },
        enabled_type{});
  }

  /// @brief Provides read-only access to the internal Algorithm object.
  /// This constant reference allows the user to access the underlying
  /// Algorithm object without modifying it.
  /// @return const algorithm_type& A reference to the internal Algorithm object.
  const algorithm_type& algorithm{algorithm_};

 private:
  algorithm_type algorithm_;
  helpers::revision optimizer_revision_;
};

/// @brief Builds a graph handle using the provided memory resource.
/// @tparam T The type of the Graph to build.
/// @tparam A Additional arguments for construction.
/// @param memory The memory resource to use for allocation.
/// @param args Additional arguments for object construction.
/// @return A handle to a graph of type T.
template <class T, class... A>
auto build(std::pmr::memory_resource* const memory, A&&... args) {
  return handle<T>(memory, memory, std::forward<A>(args)...);
}
}  // namespace vortex::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_HPP
