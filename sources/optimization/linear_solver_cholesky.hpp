/// ===========================================================================
/// @file
///
/// @brief vortex.optimization.linear_solver_cholesky component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_LINEAR_SOLVER_CHOLESKY_HPP
#define VORTEX_OPTIMIZATION_LINEAR_SOLVER_CHOLESKY_HPP

#include "foundation/math/math_invert.hpp"
#include "foundation/math/math_solver.hpp"
#include "optimization/linear_solver.hpp"

namespace vortex::graph::optimization {

/// ===========================================================================
/// @brief Linear solver that solves h * x = b by explicitly inverting h via
/// Cholesky decomposition.
/// ===========================================================================
class CholeskyInversionLinearSolver : public LinearSolver {
 public:
  /// @brief Solves a linear system like h * x = b via Cholesky inversion
  /// (computes h^-1 then multiplies by b).
  /// It is assumed that matrix shapes are correct.
  /// @param h Coef matrix.
  /// @param b Column matrix.
  /// @param x [OUT] Result matrix.
  template <class Matrix, class Vector>
  auto solve(const Matrix& h, const Vector& b, Vector& x) -> bool {
    auto h_tmp = Matrix{h};
    if (not math::invert_cholesky(h_tmp)) {
      return false;
    }
    x = h_tmp * b;
    return true;
  }
};

/// ===========================================================================
/// @brief Linear solver that solves h * x = b using Cholesky factorization
/// (L * L^T = h) followed by forward/backward substitution.
/// ===========================================================================
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

}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_LINEAR_SOLVER_CHOLESKY_HPP
