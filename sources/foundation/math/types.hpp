/// ===============================================================================================
/// @file
///
/// @brief vortex.math.types component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_MATH_MATH_TYPES_HPP
#define VORTEX_FOUNDATION_MATH_MATH_TYPES_HPP

#include <cstddef>

#include <blaze/Math.h>

namespace vortex::math {
/// ===============================================================================================
/// Matrix & Vector Types
/// ===============================================================================================
constexpr bool column_major = blaze::columnMajor;
constexpr bool column_vector = blaze::columnVector;
constexpr bool row_major = blaze::rowMajor;
constexpr bool row_vector = blaze::rowVector;

template <class Type, std::size_t R, std::size_t C, bool SO = column_major>
using static_matrix = blaze::StaticMatrix<Type, R, C, SO>;

template <class Type, std::size_t R, std::size_t C, bool SO = column_major>
using hybrid_matrix = blaze::HybridMatrix<Type, R, C, SO>;

/// ===============================================================================================
/// @brief Dynamically-sized blaze containers, on blaze's default allocator.
///
/// The allocator is not overridden here. `blaze::AlignedAllocator` is itself redirected into the
/// active `helpers::memory_scope` by foundation/math/patch/blaze-scoped-allocator.patch, so these -- and a plain
/// `blaze::DynamicMatrix<double>`, and every temporary blaze materialises -- all draw from the same
/// place. Naming an allocator here would only cover the first of the three, since blaze drops it
/// when deducing expression result types.
///
/// Storage that must outlive the scope it is sized in has to be sized under a scope that outlives
/// it instead.
/// ===============================================================================================
template <class Type, bool SO = column_major>
using dynamic_matrix = blaze::DynamicMatrix<Type, SO>;

template <class Type, bool SO = column_major>
using identity_matrix = blaze::IdentityMatrix<Type, SO>;

template <class Type, bool TF = column_vector>
using dynamic_vector = blaze::DynamicVector<Type, TF>;

template <class Type, std::size_t Size, bool TF = column_vector>
using static_vector = blaze::StaticVector<Type, Size, TF>;

template <class Type, std::size_t Size, bool TF = column_vector>
using hybrid_vector = blaze::HybridVector<Type, Size, TF>;

/// ===============================================================================================
/// Matrix Type Adapters
/// ===============================================================================================
template <class MatrixType>
using symmetric_matrix = blaze::SymmetricMatrix<MatrixType>;

template <class MatrixType>
using diagonal_matrix = blaze::DiagonalMatrix<MatrixType>;

/// ===============================================================================================
/// Type Identifiers
/// ===============================================================================================
/// @brief Check if the given type is a matrix.
/// @tparam Type The type to check.
template <class Type>
using is_matrix = blaze::IsMatrix<Type>;
template <class Type>
constexpr bool is_matrix_v = is_matrix<Type>::value;

/// @brief Check if the given type is a vector.
/// @tparam Type The type to check.
template <class Type>
using is_vector = blaze::IsVector<Type>;
template <class Type>
constexpr bool is_vector_v = is_vector<Type>::value;

/// ===============================================================================================
/// Forward Operations
/// ===============================================================================================
using blaze::column;
using blaze::diagonal;
using blaze::dot;
using blaze::max;
using blaze::submatrix;
using blaze::subvector;
using blaze::trans;

}  // namespace vortex::math

#endif  // VORTEX_FOUNDATION_MATH_MATH_TYPES_HPP
