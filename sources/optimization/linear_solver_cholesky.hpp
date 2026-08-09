/// ===============================================================================================
/// @file
///
/// @brief vortex.optimization.linear_solver_cholesky component
/// ===============================================================================================
#ifndef VORTEX_OPTIMIZATION_LINEAR_SOLVER_CHOLESKY_HPP
#define VORTEX_OPTIMIZATION_LINEAR_SOLVER_CHOLESKY_HPP

#include "foundation/math/solver.hpp"
#include "optimization/linear_solver.hpp"

namespace vortex::optimization {

/// ===============================================================================================
/// @brief Linear solver that solves h * x = b using Cholesky factorization
/// (L * L^T = h) followed by forward/backward substitution.
/// ===============================================================================================
class CholeskyLinearSolver : public LinearSolver {
 public:
  /// @brief Solves a linear system like h * x = b using Cholesky factorization
  /// (L * L^T = h) followed by forward/backward substitution.
  /// It is assumed that matrix shapes are correct and h is symmetric
  /// positive-definite (lower triangular storage).
  /// @param h Coef matrix.
  /// @param b Column matrix.
  /// @param x [OUT] Result matrix. On success contains the solution. On
  ///   failure x is unmodified.
  template <class Matrix, class Vector>
  auto solve(const Matrix& h, const Vector& b, Vector& x) -> bool {
    return math::solve_cholesky(h, b, x);
  }
};

}  // namespace vortex::optimization

#endif  // VORTEX_OPTIMIZATION_LINEAR_SOLVER_CHOLESKY_HPP
