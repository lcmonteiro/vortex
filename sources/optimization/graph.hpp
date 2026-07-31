/// ===========================================================================
/// @file
///
/// @brief vortex.graph component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_HPP
#define VORTEX_OPTIMIZATION_GRAPH_HPP
#include <expected>

#include "foundation/graph/graph.hpp"
#include "foundation/math/math.hpp"
#include "helpers/memory.hpp"
#include "optimization/graph_config.hpp"
#include "optimization/graph_edge.hpp"
#include "optimization/graph_node.hpp"

namespace vortex::graph::optimization {
using graph::OptionalShared;
using graph::Shared;

using graph::Revision;

/// ===========================================================================
/// @brief A templated Graph class implementing optimization algorithms.
///
/// @tparam Nodes The collection of nodes in the graph.
/// @tparam Edges The collection of edges in the graph.
/// @tparam Config The configuration class that defines various types and
/// parameters for the graph (default is `DefaultConfig`).
/// ===========================================================================
template <class Nodes, class Edges, class Config = DefaultConfig>
class Graph : public graph::Graph<Nodes, Edges, Config> {
  /// @brief Helper alias types.
  using LSolver = typename Config::LinearSolver;
  using GSolver = typename Config::template GraphSolver<Graph, LSolver>;
  using Algorithm = typename Config::template Algorithm<Graph, GSolver>;

 public:
  static constexpr size_t kSystemCapacity = Config::SystemCapacity;
  using Base = graph::Graph<Nodes, Edges, Config>;
  using Key = typename Config::Key;
  using Number = typename Config::Number;
  using Vector = math::DynamicVector<Number>;
  using Enabled = typename Base::Enabled;
  using Disabled = typename Base::Disabled;

  /// @brief Graph main constructor.
  /// @param memory_resource The memory resource used for allocating memory.
  explicit Graph(std::pmr::memory_resource* const memory_resource)
      : Base{memory_resource}, algorithm_{*this}, optimizer_revision_{} {}

  /// @brief Node and edge construction is inherited as-is from the base graph --
  /// optimization::Graph adds no extra construction arguments.
  using Base::build;

  /// @brief Runs the graph optimization algorithm.
  /// @param iterations The number of iterations to run.
  /// @param reset Whether to reset the algorithm state.
  /// @return The number of completed iterations or an unexpected error.
  auto optimize(size_t iterations, bool reset = true) -> std::expected<size_t, AlgorithmError> {
    // Route transient dual-number (Jacobian) allocations through the graph's
    // memory arena for the duration of the optimization.
    const helpers::MemoryScope scope{this->memory()};

    // Special case: if no iterations are requested, return immediately.
    if (0 == iterations) {
      return 0;
    }

    // Check if the graph has changed since the last optimization.
    const auto graph_changed = not this->revision()->equal(optimizer_revision_);
    const auto result_init = algorithm_.init(reset or graph_changed);
    if (not result_init) {
      return std::unexpected(result_init.error());
    }

    // Update the optimizer revision to match the current graph revision.
    optimizer_revision_.reset(*this->revision());

    // Special case: if only one iteration is requested, perform it and return.
    if (1 == iterations) {
      auto result_solve = algorithm_.template solve<true, true>();
      if (not result_solve) {
        return std::unexpected(result_solve.error());
      }
      return iterations;
    }

    // Perform the first iteration, which may have special initialization logic.
    auto result_first = algorithm_.template solve<true, false>();
    if (not result_first) {
      return std::unexpected(result_first.error());
    }
    if (result_first.value()) {
      return 1;
    }

    // Perform the intermediate iterations, which may have different logic than the first and last.
    for (size_t it = 1; it < (iterations - 1); ++it) {
      auto result_next = algorithm_.template solve<false, false>();
      if (not result_next) {
        return std::unexpected(result_next.error());
      }
      if (result_next.value()) {
        return it;
      }
    }

    // Perform the last iteration, which may have special finalization logic.
    auto result_last = algorithm_.template solve<false, true>();
    if (not result_last) {
      return std::unexpected(result_last.error());
    }

    return iterations;
  }

  /// @defgroup estimations_operations Estimations Operations
  /// @brief Functions for managing node estimations in the graph.
  ///
  /// This group includes functions to push, pull and revert estimations.
  auto push() -> void {
    ForEach<Nodes>(*this, [](auto& node) { node->push(); }, Enabled{});
  }
  auto pull() -> void {
    ForEach<Nodes>(*this, [](auto& node) { node->pull(); }, Enabled{});
  }
  auto revert(size_t n = 1) -> void {
    ForEach<Nodes>(*this, [&](auto& node) { node->revert(n); }, Enabled{});
  }

  /// @brief Computes the error vectors of all edges in the activeSet, and
  /// caches them.
  auto updateErrors() -> void {
    ForEach<Edges>(*this, [](auto& edge) { edge->updateError(); }, Enabled{});
  }

  /// @brief Computes cached chi2 of the active portion of the graph.
  auto computeChi2() const -> Number {
    Number chi = 0;
    ForEach<Edges>(*this, [&](auto& edge) { chi += edge->chi2(); }, Enabled{});
    return chi;
  }

  /// @brief Updates the estimate of the active nodes.
  auto updateEstimations(const Vector& x) -> void {
    size_t idx = 0;
    ForEach<Nodes>(
        *this,
        [&](auto& node) {
          node->updateEstimation(math::subvector(x, idx, node->dimension()));
          idx += node->dimension();
        },
        Enabled{});
  }

  /// @brief Provides read-only access to the internal Algorithm object.
  /// This constant reference allows the user to access the underlying
  /// Algorithm object without modifying it.
  /// @return const Algorithm& A reference to the internal Algorithm object.
  const Algorithm& algorithm{algorithm_};

 private:
  Algorithm algorithm_;
  Revision optimizer_revision_;
};

/// @brief Builds a shared graph using the provided memory resource.
/// @tparam T The type of the Graph to build.
/// @tparam A Additional arguments for construction.
/// @param memory The memory resource to use for allocation.
/// @param args Additional arguments for object construction.
/// @return A shared graph of type T.
template <class T, class... A>
auto Build(std::pmr::memory_resource* const memory, A&&... args) {
  return Shared<T>(memory, memory, std::forward<A>(args)...);
}
}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_HPP
