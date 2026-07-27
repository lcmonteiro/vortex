/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.optimization.linear_solver_cholesky component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_LINEAR_SOLVER_CHOLESKY_HPP
#define VORTEX_OPTIMIZATION_LINEAR_SOLVER_CHOLESKY_HPP

#include "sources/base/math_invert.hpp"
#include "sources/base/math_solver.hpp"
#include "sources/optimization/linear_solver.hpp"

namespace vortex::graph {
namespace optimization {

class CholeskyInversionLinearSolver : public LinearSolver {
 public:
  /// @brief Solves a linear system like h * x = b via Cholesky inversion
  /// (computes h^-1 then multiplies by b)
  /// It is assumed that matrix shapes are correct
  /// @param h coef matrix
  /// @param b column matrix
  /// @param x [OUT] result matrix
  template <class Matrix, class Vector>
  bool solve(const Matrix& h, const Vector& b, Vector& x) {
    auto h_tmp = Matrix{h};
    if (not math::invert_cholesky(h_tmp)) {
      return false;
    }
    x = h_tmp * b;
    return true;
  }
};

class CholeskyLinearSolver : public LinearSolver {
 public:
  /// @brief Solves a linear system like h * x = b using Cholesky factorization
  /// (L * L^T = h) followed by forward/backward substitution.
  /// It is assumed that matrix shapes are correct and h is symmetric
  /// positive-definite (lower triangular storage).
  /// @param h coef matrix
  /// @param b column matrix
  /// @param x [OUT] result matrix. On success contains the solution. On
  ///   failure x is unmodified.
  template <class Matrix, class Vector>
  bool solve(const Matrix& h, const Vector& b, Vector& x) {
    return math::solve_cholesky(h, b, x);
  }
};

}  // namespace optimization
}  // namespace vortex::graph

#endif  // VORTEX_OPTIMIZATION_LINEAR_SOLVER_CHOLESKY_HPP
