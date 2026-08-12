/// ===============================================================================================
/// @file
///
/// @brief vortex.math.solver component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_MATH_MATH_SOLVER_HPP
#define VORTEX_FOUNDATION_MATH_MATH_SOLVER_HPP

#include <cstddef>

#include "foundation/math/types.hpp"
#include "helpers/contracts.hpp"

namespace vortex::math {

/// ===============================================================================================
/// @brief Types of the scratch buffers the solvers hand to LAPACK.
///
/// Element type and storage order come from `Matrix`; its allocator deliberately does not. These
/// live in `thread_local` storage, so declaring them `Matrix` -- which is the caller's type, hence
/// `memory_scope_allocator` -- would let them hold a block of an arena from some earlier solve and
/// release it through that arena at thread exit, long after it died. `heap_allocator` says that
/// independence explicitly, so it survives VORTEX_BLAZE_SCOPED_ALLOCATOR redirecting blaze's own.
///
/// Holding them across calls is the point: they grow on demand, so a repeated solve of the same
/// size allocates nothing. Routing them into the arena would be worse anyway -- the graph caps a
/// block at 32 KiB and the LDL^T workspace is `n * lda`, 2 MiB at the default `system_capacity`.
/// ===============================================================================================
template <class Matrix>
using factor_matrix =
    blaze::DynamicMatrix<blaze::ElementType_t<Matrix>, blaze::columnMajor,
                         heap_allocator<blaze::ElementType_t<Matrix>>>;

template <class Matrix>
using factor_vector = blaze::DynamicVector<blaze::ElementType_t<Matrix>, blaze::columnVector,
                                           heap_allocator<blaze::ElementType_t<Matrix>>>;

using pivot_vector = blaze::DynamicVector<blaze::blas_int_t, blaze::columnVector,
                                          heap_allocator<blaze::blas_int_t>>;

/// ===============================================================================================
/// @brief Solves a symmetric indefinite system of linear equations using the Bunch-Kaufman LDL^T
/// factorization with lower triangular storage.
/// @tparam Matrix The type of the matrix (must support Blaze library operations and be symmetric).
/// @tparam Vector The type of the vector (must support Blaze library operations).
/// @param h The symmetric (Hermitian) matrix representing the system of equations (A in Ax = b),
/// using lower triangular storage.
/// @param b The vector representing the right-hand side of the equation (b in Ax = b).
/// @param x The vector that will store the solution to the equation (x in Ax = b).
/// @return Returns `true` if the system was solved successfully, `false` otherwise.
// The function does not throw exceptions.
/// ===============================================================================================
template <class Matrix, class Vector>
requires blaze::IsColumnMajorMatrix_v<Matrix>
inline auto solve_ldlt(const Matrix& h, const Vector& b, Vector& x) -> bool {
  VORTEX_PRECONDITION(h.rows() == h.columns(), "non-square matrix");
  VORTEX_PRECONDITION(h.rows() == std::size(b), "incompatible matrix and vector");
  using blaze::blas_int_t;
  using blaze::numeric_cast;

  static thread_local factor_matrix<Matrix> h_factor;
  static thread_local pivot_vector ipiv;
  static thread_local factor_vector<Matrix> work;

  h_factor = h;

  const auto n = numeric_cast<blas_int_t>(h_factor.rows());
  const auto lda = numeric_cast<blas_int_t>(h_factor.spacing());
  const auto ldb = numeric_cast<blas_int_t>(std::size(b));
  const auto nrhs = blas_int_t{1};
  const auto lwork = blas_int_t{n * lda};
  auto info = blas_int_t{0};

  blaze::resize(x, numeric_cast<std::size_t>(n));
  blaze::smpAssign(x, b);

  // `resize` only reallocates when growing, and LAPACK fills both, so nothing to preserve.
  ipiv.resize(numeric_cast<std::size_t>(n), false);
  work.resize(numeric_cast<std::size_t>(lwork), false);

  // Solve A * x = b via Bunch-Kaufman diagonal pivoting: P * L * D * L^T * P^T
  blaze::sysv('L', n, nrhs, h_factor.data(), lda, ipiv.data(), x.data(), ldb, work.data(), lwork,
              &info);

  return info == 0;
}

/// ===============================================================================================
/// @brief Solves a symmetric positive-definite system of linear equations using Cholesky
/// factorization (L * L^T = A) followed by forward/backward substitution.
/// @tparam Matrix The type of the matrix (must support Blaze library operations and be symmetric
/// positive-definite).
/// @tparam Vector The type of the vector (must support Blaze library operations).
/// @param h The symmetric positive-definite matrix representing the system of equations (A in Ax =
/// b), using lower triangular storage.
/// @param b The vector representing the right-hand side of the equation (b in Ax = b).
/// @param x The vector that will store the solution to the equation (x in Ax =  b). On success
/// contains the solution; on failure x is unmodified.
/// @return Returns `true` if the system was solved successfully, `false`
/// otherwise. The function does not throw exceptions.
/// ===============================================================================================
template <class Matrix, class Vector>
requires blaze::IsColumnMajorMatrix_v<Matrix>
inline auto solve_cholesky(const Matrix& h, const Vector& b, Vector& x) -> bool {
  VORTEX_PRECONDITION(h.rows() == h.columns(), "non-square matrix");
  VORTEX_PRECONDITION(h.rows() == std::size(b), "incompatible matrix and vector");

  static thread_local factor_matrix<Matrix> h_factor;
  h_factor = h;

  const auto n = blaze::numeric_cast<blaze::blas_int_t>(h_factor.rows());
  const auto lda = blaze::numeric_cast<blaze::blas_int_t>(h_factor.spacing());
  const auto ldb = blaze::numeric_cast<blaze::blas_int_t>(std::size(b));
  const auto nrhs = blaze::blas_int_t{1};
  auto info = blaze::blas_int_t{0};

  blaze::potrf('L', n, h_factor.data(), lda, &info);
  if (info != 0) {
    return false;
  }

  // Solve A * x = b via Cholesky factor: forward solve L * y = b, then
  // backward solve L^T * x = y
  blaze::resize(x, blaze::numeric_cast<std::size_t>(n));
  blaze::smpAssign(x, b);
  blaze::potrs('L', n, nrhs, h_factor.data(), lda, x.data(), ldb, &info);

  return info == 0;
}

}  // namespace vortex::math
#endif  // VORTEX_FOUNDATION_MATH_MATH_SOLVER_HPP
