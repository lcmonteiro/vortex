/// ===========================================================================
/// @file
///
/// @brief vortex.optimization.linear_solver_default component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_LINEAR_SOLVER_DEFAULT_HPP
#define VORTEX_OPTIMIZATION_LINEAR_SOLVER_DEFAULT_HPP

#include "base/math_solver.hpp"
#include "optimization/linear_solver.hpp"

namespace vortex::graph::optimization {

class DefaultLinearSolver : public LinearSolver {
 public:
  /// @brief Solves a linear system like h * x = b
  /// It is assumed that matrix shapes are correct
  /// @param h coef matrix
  /// @param b column matrix
  /// @param x [OUT] result matrix
  template <class Matrix, class Vector>
  bool solve(const Matrix& h, const Vector& b, Vector& x) {
    return math::solve_ldlt(h, b, x);
  }
};

}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_LINEAR_SOLVER_DEFAULT_HPP
