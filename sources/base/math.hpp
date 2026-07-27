/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.math_base component
/// ===========================================================================
#ifndef VORTEX_BASE_MATH_HPP
#define VORTEX_BASE_MATH_HPP

#include <blaze/Math.h>

namespace vortex::math {
/// ===========================================================================
/// Matrix & Vector Types
/// @brief math base interface types
/// ===========================================================================
using blaze::columnMajor;
using blaze::columnVector;
using blaze::rowMajor;
using blaze::rowVector;

template <class Type, size_t R, size_t C, bool SO = columnMajor>
using StaticMatrix = blaze::StaticMatrix<Type, R, C, SO>;

template <class Type, size_t R, size_t C, bool SO = columnMajor>
using HybridMatrix = blaze::HybridMatrix<Type, R, C, SO>;

template <class Type, bool SO = columnMajor>
using DynamicMatrix = blaze::DynamicMatrix<Type, SO>;

template <class Type, bool SO = columnMajor>
using IdentityMatrix = blaze::IdentityMatrix<Type, SO>;

template <class Type, bool TF = columnVector>
using DynamicVector = blaze::DynamicVector<Type, TF>;

template <class Type, size_t Size, bool TF = columnVector>
using StaticVector = blaze::StaticVector<Type, Size, TF>;

template <class Type, size_t Size, bool TF = columnVector>
using HybridVector = blaze::HybridVector<Type, Size, TF>;

/// ===========================================================================
/// Matrix Type Adapters
/// @brief math base matrix adapters
/// ===========================================================================
template <class MatrixType>
using SymmetricMatrix = blaze::SymmetricMatrix<MatrixType>;

template <class MatrixType>
using DiagonalMatrix = blaze::DiagonalMatrix<MatrixType>;

/// ===========================================================================
/// Type identifiers
/// @brief Type identifiers.
/// ===========================================================================

/// @brief Check if the given type is a matrix.
/// @tparam Type The type to check.
template <class Type>
using IsMatrix = blaze::IsMatrix<Type>;

/// @brief Check if the given type is a vector.
/// @tparam Type The type to check.
template <class Type>
using IsVector = blaze::IsVector<Type>;

/// ===========================================================================
/// Forward Operations
/// @brief math base required operations
/// ===========================================================================
using blaze::column;
using blaze::diagonal;
using blaze::dot;
using blaze::max;
using blaze::submatrix;
using blaze::subvector;
using blaze::trans;

}  // namespace vortex::math

#endif  // VORTEX_BASE_MATH_HPP
