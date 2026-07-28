/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.optimization.linear_solver component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_LINEAR_SOLVER_HPP
#define VORTEX_OPTIMIZATION_LINEAR_SOLVER_HPP

#include "base/math.hpp"

namespace vortex::graph::optimization {

class LinearSolver {
 public:
  /// @brief Whether this solver requires the full symmetric matrix
  /// (both triangles populated) rather than just the lower triangle.
  static constexpr auto kRequiresFullMatrix = false;

  /// @brief Linear Solver initialization
  /// @return
  bool init() { return true; }

  /// @brief Solves a linear system like h * x = b
  /// @param h coef matrix
  /// @param b column matrix
  /// @param x [OUT] result matrix
  template <class Matrix, class Vector>
  bool solve(const Matrix& h, const Vector& b, Vector& x);
};

}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_LINEAR_SOLVER_HPP
