/// ===========================================================================
/// @file
///
/// @brief vortex.graph component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_SOLVER_BLOCK_HPP
#define VORTEX_OPTIMIZATION_GRAPH_SOLVER_BLOCK_HPP

#include "foundation/math.hpp"
#include "optimization/graph_operations.hpp"
#include "optimization/graph_solver.hpp"
namespace vortex::graph::optimization {

/// ===========================================================================
/// @brief Block graph solver.
/// ===========================================================================
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
  auto operator=(const BlockGraphSolver&) -> BlockGraphSolver& = delete;
  BlockGraphSolver(BlockGraphSolver&&) = default;
  auto operator=(BlockGraphSolver&&) -> BlockGraphSolver& = default;

  /// @brief this function build the system stucture
  ///  - matrix and vector shapes
  ///  - update nodes positions
  auto buildStructure() -> bool {
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
  auto buildSystem() -> void {
    this->h_.reset();
    this->b_.reset();

    ForEach<Edges>(
        this->graph_,
        [this](const auto& edge) {
          edge->forEachHBlock(UpdateHBlock{this});
          edge->forEachBBlock(UpdateBBlock{this});
        },
        Enabled{});
  }

  /// @brief this function updates the system matrix diagonal
  /// @param update value that will be added
  auto updateDiagonal(Number update) -> void {
    h_diagonal_backup_ = math::diagonal(h_);
    math::diagonal(h_) += update;
  }

  /// @brief this function restores the system matrix diagonal
  auto restoreDiagonal() -> void { math::diagonal(h_) = h_diagonal_backup_; }

  /// @brief Get solution vector
  auto x() const -> const Vector& { return x_; }

  /// @brief Get right hand side of the system
  auto b() const -> const Vector& { return b_; }

  /// @brief Get hessian matrix
  auto h() const -> const Matrix& { return h_; }

  /// @brief Algorithm solve
  /// @return a boolean value whether it was solved or an algorithm error
  auto solve() -> bool { return Base::lsolver_.solve(h_, b_, x_); }

 protected:
  /// @brief Functor that accumulates a Hessian block update into the system
  /// matrix, honoring the solver's full or triangular storage requirement.
  struct UpdateHBlock {
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
    BlockGraphSolver* self;
  };
  /// @brief Functor that accumulates a gradient block update into the system
  /// right-hand-side vector.
  struct UpdateBBlock {
    template <class N, class B>
    auto operator()(const N& node, const B& update) -> void {
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

}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_SOLVER_BLOCK_HPP
