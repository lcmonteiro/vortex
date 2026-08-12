/// ===============================================================================================
/// @file
///
/// @brief vortex.math.types component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_MATH_MATH_TYPES_HPP
#define VORTEX_FOUNDATION_MATH_MATH_TYPES_HPP

#include <cstddef>

#include <blaze/Math.h>

#include "foundation/math/memory.hpp"

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
/// @brief Dynamically-sized blaze containers defaulted onto `memory_scope_allocator`.
///
/// These are the project's spelling of `blaze::DynamicMatrix`/`blaze::DynamicVector`. `Alloc` is a
/// default rather than a fixed argument so an explicit one is still honoured --
/// `dynamic_matrix<double, column_major, blaze::AlignedAllocator<double>>` opts back out -- and
/// `SO`/`TF` mirror the fixed-size aliases above. Blaze's fourth parameter, the group tag, is left
/// alone: it exists to make quantities from different groups fail to combine, which the project
/// does not use, and omitting it selects the same `Group0` that spelling it would.
///
/// @note The default argument of `blaze::DynamicMatrix` itself cannot be retargeted from here:
/// [temp.param]/12 forbids giving a template parameter default arguments in two declarations, and
/// blaze exposes no configuration hook for it. Writing `blaze::DynamicMatrix<double>` therefore
/// still selects `blaze::AlignedAllocator`, so prefer these aliases throughout the project.
/// ===============================================================================================
template <class Type, bool SO = column_major, class Alloc = memory_scope_allocator<Type>>
using dynamic_matrix = blaze::DynamicMatrix<Type, SO, Alloc>;

template <class Type, bool SO = column_major>
using identity_matrix = blaze::IdentityMatrix<Type, SO>;

template <class Type, bool TF = column_vector, class Alloc = memory_scope_allocator<Type>>
using dynamic_vector = blaze::DynamicVector<Type, TF, Alloc>;

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
