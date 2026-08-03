/// ===========================================================================
/// @file
///
/// @brief vortex.optimization.linear_solver component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_LINEAR_SOLVER_HPP
#define VORTEX_OPTIMIZATION_LINEAR_SOLVER_HPP

#include "foundation/math.hpp"

namespace vortex::graph::optimization {

/// ===========================================================================
/// @brief Base class for linear solvers of the system h * x = b.
///
/// Concrete solvers implement a `solve(h, b, x)` member and may override
/// `kRequiresFullMatrix` to request a fully populated symmetric matrix.
/// ===========================================================================
class LinearSolver {
 public:
  /// @brief Whether this solver requires the full symmetric matrix
  /// (both triangles populated) rather than just the lower triangle.
  static constexpr auto kRequiresFullMatrix = false;

  /// @brief Initializes the linear solver.
  /// @return Always `true`.
  auto init() -> bool { return true; }

  /// @brief Solves a linear system like h * x = b.
  /// @param h Coef matrix.
  /// @param b Column matrix.
  /// @param x [OUT] Result matrix.
  template <class Matrix, class Vector>
  auto solve(const Matrix& h, const Vector& b, Vector& x) -> bool;
};

}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_LINEAR_SOLVER_HPP
