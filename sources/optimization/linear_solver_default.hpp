/// ===============================================================================================
/// @file
///
/// @brief vortex.optimization.linear_solver_default component
/// ===============================================================================================
#ifndef VORTEX_OPTIMIZATION_LINEAR_SOLVER_DEFAULT_HPP
#define VORTEX_OPTIMIZATION_LINEAR_SOLVER_DEFAULT_HPP

#include "foundation/math/solver.hpp"
#include "optimization/linear_solver.hpp"

namespace vortex::optimization {

/// ===============================================================================================
/// @brief Default linear solver for the system h * x = b.
/// ===============================================================================================
class DefaultLinearSolver : public LinearSolver {
 public:
  /// @brief Solves a linear system like h * x = b.
  /// It is assumed that matrix shapes are correct.
  /// @param h Coef matrix.
  /// @param b Column matrix.
  /// @param x [OUT] Result matrix.
  template <class Matrix, class Vector>
  auto solve(const Matrix& h, const Vector& b, Vector& x) -> bool {
    return math::solve_ldlt(h, b, x);
  }
};

}  // namespace vortex::optimization

#endif  // VORTEX_OPTIMIZATION_LINEAR_SOLVER_DEFAULT_HPP
