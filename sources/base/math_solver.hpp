/// ===========================================================================
/// @file
///
/// @brief vortex.math_base component
/// ===========================================================================
#ifndef VORTEX_BASE_MATH_SOLVER_HPP
#define VORTEX_BASE_MATH_SOLVER_HPP

#include "base/math.hpp"
#include "helpers/contracts.hpp"

namespace vortex::math {

/// ===========================================================================
/// @brief Solves a symmetric indefinite system of linear equations using the
/// Bunch-Kaufman LDL^T factorization with lower triangular storage.
/// @tparam Matrix The type of the matrix (must support Blaze library operations
/// and be symmetric).
/// @tparam Vector The type of the vector (must support Blaze library
/// operations).
/// @param h The symmetric (Hermitian) matrix representing the system of
/// equations (A in Ax = b), using lower triangular storage.
/// @param b The vector representing the right-hand side of the equation (b in
/// Ax = b).
/// @param x The vector that will store the solution to the equation (x in Ax =
/// b).
/// @return Returns `true` if the system was solved successfully, `false`
/// otherwise. The function does not throw exceptions.
/// ===========================================================================
template <class Matrix, class Vector,
          blaze::EnableIf_t<blaze::IsColumnMajorMatrix_v<Matrix>>* = nullptr>
inline auto solve_ldlt(const Matrix& h, const Vector& b, Vector& x) -> bool {
  VORTEX_PRECONDITION(h.rows() == h.columns(), "non-square matrix");
  VORTEX_PRECONDITION(h.rows() == b.size(), "incompatible matrix and vector");

  static thread_local Matrix h_factor;
  h_factor = h;

  const auto n = blaze::numeric_cast<blaze::blas_int_t>(h_factor.rows());
  const auto lda = blaze::numeric_cast<blaze::blas_int_t>(h_factor.spacing());
  const auto ldb = blaze::numeric_cast<blaze::blas_int_t>(b.size());
  const auto nrhs = blaze::blas_int_t{1};
  const auto lwork = blaze::blas_int_t{n * lda};
  auto info = blaze::blas_int_t{0};

  blaze::resize(x, blaze::numeric_cast<size_t>(n));
  blaze::smpAssign(x, b);

  auto ipiv = blaze::DynamicVector<blaze::blas_int_t>(blaze::numeric_cast<size_t>(n));
  auto work =
      blaze::DynamicVector<blaze::ElementType_t<Matrix>>(blaze::numeric_cast<size_t>(lwork));

  // Solve A * x = b via Bunch-Kaufman diagonal pivoting: P * L * D * L^T * P^T
  blaze::sysv('L', n, nrhs, h_factor.data(), lda, ipiv.data(), x.data(), ldb, work.data(), lwork,
              &info);

  return info == 0;
}

/// ===========================================================================
/// @brief Solves a symmetric positive-definite system of linear equations
/// using Cholesky factorization (L * L^T = A) followed by forward/backward
/// substitution.
/// @tparam Matrix The type of the matrix (must support Blaze library operations
/// and be symmetric positive-definite).
/// @tparam Vector The type of the vector (must support Blaze library
/// operations).
/// @param h The symmetric positive-definite matrix representing the system of
/// equations (A in Ax = b), using lower triangular storage.
/// @param b The vector representing the right-hand side of the equation (b in
/// Ax = b).
/// @param x The vector that will store the solution to the equation (x in Ax =
/// b). On success contains the solution; on failure x is unmodified.
/// @return Returns `true` if the system was solved successfully, `false`
/// otherwise. The function does not throw exceptions.
/// ===========================================================================
template <class Matrix, class Vector,
          blaze::EnableIf_t<blaze::IsColumnMajorMatrix_v<Matrix>>* = nullptr>
inline auto solve_cholesky(const Matrix& h, const Vector& b, Vector& x) -> bool {
  VORTEX_PRECONDITION(h.rows() == h.columns(), "non-square matrix");
  VORTEX_PRECONDITION(h.rows() == b.size(), "incompatible matrix and vector");

  static thread_local Matrix h_factor;
  h_factor = h;

  const auto n = blaze::numeric_cast<blaze::blas_int_t>(h_factor.rows());
  const auto lda = blaze::numeric_cast<blaze::blas_int_t>(h_factor.spacing());
  const auto ldb = blaze::numeric_cast<blaze::blas_int_t>(b.size());
  const auto nrhs = blaze::blas_int_t{1};
  auto info = blaze::blas_int_t{0};

  blaze::potrf('L', n, h_factor.data(), lda, &info);
  if (info != 0) {
    return false;
  }

  // Solve A * x = b via Cholesky factor: forward solve L * y = b, then
  // backward solve L^T * x = y
  blaze::resize(x, blaze::numeric_cast<size_t>(n));
  blaze::smpAssign(x, b);
  blaze::potrs('L', n, nrhs, h_factor.data(), lda, x.data(), ldb, &info);

  return info == 0;
}

}  // namespace vortex::math
#endif  // VORTEX_BASE_MATH_SOLVER_HPP
