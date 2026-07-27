/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief g2o_dual.math_base component
/// ===========================================================================
#ifndef G2O_DUAL_BASE_MATH_INVERT_HPP
#define G2O_DUAL_BASE_MATH_INVERT_HPP

#include "sources/base/math.hpp"
#include "sources/helpers/compat.hpp"

namespace math {

/// ===========================================================================
/// @brief Inverts a square matrix using Cholesky decomposition.
/// @tparam Matrix The type of the matrix.
/// @param matrix The matrix to be inverted. It must be a square
/// positive-definite matrix.
/// @return Returns `true` if the inversion was successful, `false` otherwise.
/// The function does not throw exceptions.
/// ===========================================================================
template <class Matrix, blaze::EnableIf_t<blaze::IsColumnMajorMatrix_v<Matrix> >* = nullptr>
inline bool invert_cholesky(Matrix& matrix) {
  G2O_DUAL_PRECONDITION(matrix.rows() == matrix.columns(), "non-square matrix");
  const auto n = blaze::numeric_cast<blaze::blas_int_t>(matrix.rows());
  const auto lda = blaze::numeric_cast<blaze::blas_int_t>(matrix.spacing());
  auto info = blaze::blas_int_t{0};
  blaze::potrf('L', n, matrix.data(), lda, &info);
  if (info != 0) {
    return false;
  }
  blaze::potri('L', n, matrix.data(), lda, &info);
  if (info != 0) {
    return false;
  }
  for (size_t i = 1UL; i < matrix.rows(); ++i) {
    for (size_t j = 0UL; j < i; ++j) {
      matrix(j, i) = blaze::conj(matrix(i, j));
    }
  }
  return true;
}

}  // namespace math
#endif  // G2O_DUAL_BASE_MATH_INVERT_HPP
