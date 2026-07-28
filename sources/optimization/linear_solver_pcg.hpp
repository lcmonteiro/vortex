/// ===========================================================================
/// @file
///
/// @brief vortex.optimization.linear_solver_pcg component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_LINEAR_SOLVER_PCG_HPP
#define VORTEX_OPTIMIZATION_LINEAR_SOLVER_PCG_HPP

#include "foundation/math/math_invert.hpp"
#include "optimization/linear_solver.hpp"

namespace vortex::graph::optimization {

/// ===========================================================================
/// @brief Preconditioned Conjugate Gradient (PCG) linear solver for h * x = b.
///
/// @tparam Number           Scalar type used for the computation.
/// @tparam ResidualExponent Base-10 exponent of the convergence residual limit.
/// @tparam IterationsLimit  Maximum number of PCG iterations.
/// ===========================================================================
template <class Number, int ResidualExponent = -6, size_t IterationsLimit = 1000>
class PCGLinearSolver : public LinearSolver {
 public:
  static constexpr bool kRequiresFullMatrix = true;

  const size_t iterations_limit = IterationsLimit;
  const Number residual_limit = Number{std::pow(Number{10}, Number{ResidualExponent})};

  /// @brief Solves a linear system like h * x = b using the Preconditioned
  /// Conjugate Gradient method.
  /// It is assumed that matrix shapes are correct.
  /// @param h Coef matrix.
  /// @param b Column matrix.
  /// @param x [OUT] Result matrix.
  template <class Matrix, class Vector>
  auto solve(const Matrix& h, const Vector& b, Vector& x) -> bool {
    const auto diag_inv = 1.0 / diagonal(h);
    auto r = Vector(b - (h * x));
    auto z = Vector(diag_inv * r);
    auto p = z;
    auto rz = dot(r, z);

    const auto rz_init = rz;
    const auto rz_end = residual_limit * rz_init;

    for (size_t iter = 1;; ++iter) {
      const auto ap = h * p;
      const auto alpha = rz / dot(p, ap);

      x += alpha * p;
      r -= alpha * ap;
      z = diag_inv * r;

      const auto rz_new = math::dot(r, z);

      if (rz_new <= rz_end or iter >= iterations_limit) {
        return rz_new <= rz_end;
      }

      const auto beta = rz_new / rz;
      p = z + beta * p;
      rz = rz_new;
    }
    return false;
  }
};

}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_LINEAR_SOLVER_PCG_HPP
