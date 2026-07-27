/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.optimization.linear_solver_default component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_LINEAR_SOLVER_DEFAULT_HPP
#define VORTEX_OPTIMIZATION_LINEAR_SOLVER_DEFAULT_HPP

#include "sources/base/math_solver.hpp"
#include "sources/optimization/linear_solver.hpp"

namespace graph {
namespace optimization {

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

}  // namespace optimization
}  // namespace graph

#endif  // VORTEX_OPTIMIZATION_LINEAR_SOLVER_DEFAULT_HPP
