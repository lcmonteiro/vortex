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
/// @brief Types of the scratch buffers the solvers below hand to LAPACK.
///
/// They live in `thread_local` storage and grow on demand, so a repeated solve of the same size
/// allocates nothing. That makes them outlive any `memory_scope`, which is why they use blaze's
/// default allocator: a buffer drawn from the caller's arena would be released through it long
/// after the arena was destroyed.
///
/// Routing them into the arena is worse than it looks anyway. The graph caps a block at
/// `cache_block_max_size` (32 KiB) and the LDL^T workspace is sized `n * lda` -- 2 MiB at the
/// default `system_capacity` of 512. That trips the bounded resource's precondition in debug, and
/// in release the pool forwards oversized blocks to the monotonic buffer, which never reuses them:
/// every solve would consume another 2 MiB for the life of the graph.
/// ===============================================================================================
template <class Matrix>
using factor_matrix = blaze::DynamicMatrix<blaze::ElementType_t<Matrix>, blaze::columnMajor>;

template <class Matrix>
using factor_vector = blaze::DynamicVector<blaze::ElementType_t<Matrix>>;

using pivot_vector = blaze::DynamicVector<blaze::blas_int_t>;

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

  // `resize` only reallocates when growing, so these settle at the largest system seen on this
  // thread. Neither is read before LAPACK fills it, so there is nothing to preserve.
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
