/// ===============================================================================================
/// @file
///
/// @brief vortex.graph component
/// ===============================================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_SOLVER_BLOCK_HPP
#define VORTEX_OPTIMIZATION_GRAPH_SOLVER_BLOCK_HPP

#include <cstddef>

#include "foundation/math.hpp"
#include "optimization/graph_operations.hpp"
#include "optimization/graph_solver.hpp"
namespace vortex::optimization {

/// ===============================================================================================
/// @brief A templated class that provides solver functionality for a graph using a linear solver.
///
/// @tparam Graph The graph type to be solved (can be any graph type, as long as it is compatible).
/// @tparam LinearSolver The linear solver type used to perform solving operations on the graph.
/// ===============================================================================================
template <class Graph, class LinearSolver>
class block_graph_solver : public graph_solver<Graph, LinearSolver> {
 public:
  using base_type = graph_solver<Graph, LinearSolver>;
  using nodes_list = typename Graph::Nodes;
  using edges_list = typename Graph::Edges;
  using number_type = typename Graph::number_type;
  using enabled_type = typename Graph::enabled_type;
  using vector_type = math::dynamic_vector<number_type>;
  using matrix_type = math::dynamic_matrix<number_type>;
  using diagonal_type = math::dynamic_vector<number_type>;

  /// @brief Constructor of the block_graph_solver.
  /// @param graph The graph to be solved.
  explicit block_graph_solver(Graph& graph)
      : base_type{graph}, x_{}, b_{}, h_{}, h_diagonal_backup_{} {
    h_.reserve(Graph::kSystemCapacity * Graph::kSystemCapacity);
    x_.reserve(Graph::kSystemCapacity);
    b_.reserve(Graph::kSystemCapacity);
    h_diagonal_backup_.reserve(Graph::kSystemCapacity);
  }

  /// @brief Deleted copy constructor and defaulted move constructor.
  block_graph_solver(const block_graph_solver&) = delete;
  block_graph_solver(block_graph_solver&&) = default;
  ~block_graph_solver() = default;

  /// @brief Deleted copy assignment operator and defaulted move assignment operator.
  auto operator=(const block_graph_solver&) -> block_graph_solver& = delete;
  auto operator=(block_graph_solver&&) -> block_graph_solver& = default;

  /// @brief this function build the system stucture
  ///  - matrix and vector shapes
  ///  - update nodes positions
  auto build_structure() -> bool {
    auto total_dimension = std::size_t{0};

    for_each<nodes_list>(
        this->graph_,
        [&total_dimension](auto& node) {
          node->position(total_dimension);
          total_dimension += node->dimension();
        },
        enabled_type{});

    x_.resize(total_dimension, false);
    b_.resize(total_dimension, false);
    h_.resize(total_dimension, total_dimension, false);
    h_diagonal_backup_.resize(total_dimension, false);

    return total_dimension > std::size_t{0};
  }

  /// @brief this function compute system matrix
  auto build_system() -> void {
    this->h_.reset();
    this->b_.reset();

    for_each<edges_list>(
        this->graph_,
        [this](const auto& edge) {
          edge->foreach_h_block(update_h_block{this});
          edge->foreach_b_block(update_b_block{this});
        },
        enabled_type{});
  }

  /// @brief this function updates the system matrix diagonal
  /// @param update value that will be added
  auto update_diagonal(number_type update) -> void {
    h_diagonal_backup_ = math::diagonal(h_);
    math::diagonal(h_) += update;
  }

  /// @brief this function restores the system matrix diagonal
  auto restore_diagonal() -> void { math::diagonal(h_) = h_diagonal_backup_; }

  /// @brief Get solution vector
  auto x() const -> const vector_type& { return x_; }

  /// @brief Get right hand side of the system
  auto b() const -> const vector_type& { return b_; }

  /// @brief Get hessian matrix
  auto h() const -> const matrix_type& { return h_; }

  /// @brief Algorithm solve
  /// @return a boolean value whether it was solved or an algorithm error
  auto solve() -> bool { return base_type::lsolver_.solve(h_, b_, x_); }

 protected:
  /// @brief Functor that accumulates a Hessian block update into the system
  /// matrix, honoring the solver's full or triangular storage requirement.
  struct update_h_block {
    template <class Ni, class Nj, class H>
    auto operator()(const Ni& node_i, const Nj& node_j, const H& update) -> void {
      if constexpr (LinearSolver::kRequiresFullMatrix) {
        symmetric(node_i, node_j, update);
      } else {
        triangular(node_i, node_j, update);
      }
    }
    template <class Ni, class Nj, class H>
    auto symmetric(const Ni& node_i, const Nj& node_j, const H& update) -> void {
      const auto position_i = node_i->position();
      const auto position_j = node_j->position();
      const auto dimension_i = node_i->dimension();
      const auto dimension_j = node_j->dimension();
      math::submatrix(self->h_, position_i, position_j, dimension_i, dimension_j) += update;
      if (position_i != position_j) {
        math::submatrix(self->h_, position_j, position_i, dimension_j, dimension_i) +=
            math::trans(update);
      }
    }
    template <class Ni, class Nj, class H>
    auto triangular(const Ni& node_i, const Nj& node_j, const H& update) -> void {
      const auto position_i = node_i->position();
      const auto position_j = node_j->position();
      const auto dimension_i = node_i->dimension();
      const auto dimension_j = node_j->dimension();
      if (position_i >= position_j) {
        math::submatrix(self->h_, position_i, position_j, dimension_i, dimension_j) += update;
      } else {
        math::submatrix(self->h_, position_j, position_i, dimension_j, dimension_i) +=
            math::trans(update);
      }
    }
    block_graph_solver* self;
  };

  /// @brief Functor that accumulates a gradient block update into the system
  /// right-hand-side vector.
  struct update_b_block {
    template <class N, class B>
    auto operator()(const N& node, const B& update) -> void {
      math::subvector(self->b_, node->position(), node->dimension()) -= update;
    }
    block_graph_solver* self;
  };

 private:
  vector_type x_;
  vector_type b_;
  matrix_type h_;
  diagonal_type h_diagonal_backup_;
};

}  // namespace vortex::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_SOLVER_BLOCK_HPP
