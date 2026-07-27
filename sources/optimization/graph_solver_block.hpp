/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.graph component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_SOLVER_BLOCK_HPP
#define VORTEX_OPTIMIZATION_GRAPH_SOLVER_BLOCK_HPP

#include "sources/base/math.hpp"
#include "sources/optimization/graph_operations.hpp"
#include "sources/optimization/graph_solver.hpp"
namespace graph {
namespace optimization {

/// ============================================================================
/// @brief Block graph solver
/// ============================================================================
template <class Graph, class LinearSolver>
class BlockGraphSolver : public GraphSolver<Graph, LinearSolver> {
 public:
  using Nodes = typename Graph::Nodes;
  using Edges = typename Graph::Edges;
  using Number = typename Graph::Number;
  using Enabled = typename Graph::Enabled;
  using Vector = math::DynamicVector<Number>;
  using Matrix = math::DynamicMatrix<Number>;
  using Diagonal = math::DynamicVector<Number>;
  using Base = GraphSolver<Graph, LinearSolver>;

  explicit BlockGraphSolver(Graph& graph) : Base{graph}, x_{}, b_{}, h_{}, h_diagonal_backup_{} {
    h_.reserve(Graph::kSystemCapacity * Graph::kSystemCapacity);
    x_.reserve(Graph::kSystemCapacity);
    b_.reserve(Graph::kSystemCapacity);
    h_diagonal_backup_.reserve(Graph::kSystemCapacity);
  }
  ~BlockGraphSolver() = default;
  BlockGraphSolver(const BlockGraphSolver&) = delete;
  BlockGraphSolver& operator=(const BlockGraphSolver&) = delete;
  BlockGraphSolver(BlockGraphSolver&&) = default;
  BlockGraphSolver& operator=(BlockGraphSolver&&) = default;

  /// @brief this function build the system stucture
  ///  - matrix and vector shapes
  ///  - update nodes positions
  bool buildStructure() {
    auto total_dimension = size_t{0};

    ForEach<Nodes>(
        this->graph_,
        [&total_dimension](auto& node) {
          node->position(total_dimension);
          total_dimension += node->dimension();
        },
        Enabled{});

    x_.resize(total_dimension, false);
    b_.resize(total_dimension, false);
    h_.resize(total_dimension, total_dimension, false);
    h_diagonal_backup_.resize(total_dimension, false);

    return total_dimension > size_t{0};
  }

  /// @brief this function compute system matrix
  void buildSystem() {
    this->h_.reset();
    this->b_.reset();

    ForEach<Edges>(
        this->graph_,
        [this](const auto& edge) {
          edge->updateJacobians();
          edge->forEachHBlock(UpdateHBlock{this});
          edge->forEachBBlock(UpdateBBlock{this});
        },
        Enabled{});
  }

  /// @brief this function updates the system matrix diagonal
  /// @param update value that will be added
  void updateDiagonal(Number update) {
    h_diagonal_backup_ = math::diagonal(h_);
    math::diagonal(h_) += update;
  }

  /// @brief this function restores the system matrix diagonal
  void restoreDiagonal() { math::diagonal(h_) = h_diagonal_backup_; }

  /// @brief Get solution vector
  const Vector& x() const { return x_; }

  /// @brief Get right hand side of the system
  const Vector& b() const { return b_; }

  /// @brief Get hessian matrix
  const auto& h() const { return h_; }

  /// @brief Algorithm solve
  /// @return a boolean value whether it was solved or an algorithm error
  bool solve() { return Base::lsolver_.solve(h_, b_, x_); }

 protected:
  struct UpdateHBlock {
    template <class Ni, class Nj, class H>
    void operator()(const Ni& node_i, const Nj& node_j, const H& update) {
      if constexpr (LinearSolver::kRequiresFullMatrix) {
        symmetric(node_i, node_j, update);
      } else {
        triangular(node_i, node_j, update);
      }
    }
    template <class Ni, class Nj, class H>
    void symmetric(const Ni& node_i, const Nj& node_j, const H& update) {
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
    void triangular(const Ni& node_i, const Nj& node_j, const H& update) {
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
    BlockGraphSolver* self;
  };
  struct UpdateBBlock {
    template <class N, class B>
    void operator()(const N& node, const B& update) {
      math::subvector(self->b_, node->position(), node->dimension()) -= update;
    }
    BlockGraphSolver* self;
  };

 private:
  Vector x_;
  Vector b_;
  Matrix h_;
  Diagonal h_diagonal_backup_;
};

}  // namespace optimization
}  // namespace graph

#endif  // VORTEX_OPTIMIZATION_GRAPH_SOLVER_BLOCK_HPP
