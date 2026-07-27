/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.optimization.variants component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_VARIANTS_INFORMATION_MATRIX_HPP
#define VORTEX_OPTIMIZATION_VARIANTS_INFORMATION_MATRIX_HPP

#include <variant>

#include "sources/base/math.hpp"

namespace graph {
namespace optimization {
namespace variants {

/// @brief Can be used to enable methods where a matrix is needed.
/// @tparam Type Type to check if it is a matrix.
template <class Type>
using if_matrix = std::enable_if_t<math::IsMatrix<Type>::value, bool>;

/// @brief Can be used to enable methods where a vector is needed.
/// @tparam Type Type to check if it is a vector.
template <class Type>
using if_vector = std::enable_if_t<math::IsVector<Type>::value, bool>;

/// ===========================================================================
/// @brief Information Matrices: Identity Matrix
/// ===========================================================================

/// @brief An information matrix that is an identity matrix.
/// @tparam Number Type of the elements of the matrix.
/// @tparam Dimension The dimension the matrix. The matrix is square
/// (Dimension*Dimension).
template <class Number, size_t Dimension>
struct IdentityInformationOption {
  /// @brief Constuctor of an information matrix that is an identity matrix.
  IdentityInformationOption() : matrix_{Dimension} {};

  /// @brief Gets the matrix isntance.
  /// @return The matrix instance.
  const auto& get() const { return matrix_; }

 private:
  /// @brief The matrix instance.
  math::IdentityMatrix<Number> matrix_;
};

/// ===========================================================================
/// @brief Information Matrices: Diagonal Matrix
/// ===========================================================================

/// @brief An information matrix that is a diagonal matrix.
/// @tparam Number Type of the elements of the matrix.
/// @tparam Dimension The dimension the matrix. The matrix is square
/// (Dimension*Dimension).
template <class Number, size_t Dimension>
struct DiagonalInformationOption {
  /// @brief Constuctor of an information matrix that is a diagonal matrix.
  DiagonalInformationOption() : matrix_{math::IdentityMatrix<Number>(Dimension)} {};

  /// @brief Sets the value of the information matrix based on another matrix.
  /// @tparam M Type of the vector.
  /// @tparam  This method is only enabled for Matrix types.
  /// @param matrix The matrix containing the values to set the information to.
  template <class Matrix, if_matrix<Matrix> = true>
  void set(const Matrix& matrix) {
    matrix_ = matrix;
  }

  /// @brief Sets the value of the information matrix diagonal based on a
  /// vector.
  /// @tparam V Type of the vector.
  /// @tparam  This method is only enabled for Vector types.
  /// @param vector The vector containing the values to set the information
  /// diagonal to.
  template <class Vector, if_vector<Vector> = true>
  void set(const Vector& vector) {
    math::diagonal(matrix_) = vector;
  }

  /// @brief Sets the value of the information matrix diagonal based on a group
  /// of values.
  /// @tparam ...T Type of the values.
  /// @param ...values The values.
  template <class Type, class... Types>
  void set(const Type& value, const Types&... values) {
    /// Ensure the number or received argument match the matrix dimension.
    static_assert((1 + sizeof...(Types)) == Dimension,
                  "The number of values must match the square matrix dimension.");

    math::diagonal(matrix_) = {value, values...};
  }

  /// @brief Gets the matrix isntance.
  /// @return The matrix instance.
  const auto& get() const { return matrix_; }

 private:
  /// @brief Type of the base matrix.
  using BaseMatrix = math::StaticMatrix<Number, Dimension, Dimension>;

  /// @brief The matrix instance.
  math::DiagonalMatrix<BaseMatrix> matrix_;
};

/// ===========================================================================
/// @brief Information Matrices: Normal Matrix
/// ===========================================================================

/// @brief An information matrix.
/// @tparam Number Type of the elements of the matrix.
/// @tparam Dimension The dimension the matrix. The matrix is square
/// (Dimension*Dimension).
template <class Number, size_t Dimension>
struct SymmetricInformationOption {
  /// @brief Constuctor of an information matrix.
  SymmetricInformationOption() : matrix_{math::IdentityMatrix<Number>(Dimension)} {};

  /// @brief Sets the value of the information matrix based on another matrix.
  /// @tparam M Type of the vector.
  /// @tparam  This method is only enabled for Matrix types.
  /// @param matrix The matrix containing the values to set the information to.
  template <class Matrix, if_matrix<Matrix> = true>
  void set(const Matrix& matrix) {
    matrix_ = matrix;
  }

  /// @brief Gets the matrix instance.
  /// @return The matrix instance.
  const auto& get() const { return matrix_; }

 private:
  /// @brief Type of the base matrix.
  using BaseMatrix = math::StaticMatrix<Number, Dimension, Dimension>;

  /// @brief The matrix instance.
  math::SymmetricMatrix<BaseMatrix> matrix_;
};

/// ===========================================================================
/// @brief Information Matrix Alternatives
/// ===========================================================================

/// @brief Information matrix options.
constexpr size_t kIdentityMatrix = 0;
constexpr size_t kDiagonalMatrix = 1;
constexpr size_t kSymmetricMatrix = 2;

/// @brief Information Matrix Variant
/// @tparam Derived
template <class Derived, class Config, size_t Dimension>
struct InformationVariant {
  using Number = typename Config::Number;

  /// @brief Variant containing the Information matrix format options.
  using MatrixVariant = std::variant<IdentityInformationOption<Number, Dimension>,
                                     DiagonalInformationOption<Number, Dimension>,
                                     SymmetricInformationOption<Number, Dimension>>;

  /// @brief Constructor of the InformationVariant, according to the option set
  /// in Derived::kInformation.
  /// @warning Asserts if the value set in Derived::kInformationis valid.
  constexpr InformationVariant()
      : matrix_(std::variant_alternative_t<Derived::kInformation, MatrixVariant>{}) {
    static_assert(Derived::kInformation >= kIdentityMatrix, "kInformation >= kIdentityMatrix");

    static_assert(Derived::kInformation <= kSymmetricMatrix, "kInformation <= kSymmetricMatrix");
  }

  /// @brief Gets the pointer of the active information matrix
  /// @return Information matrix pointer
  constexpr auto* operator->() { return &get(); }

  /// @brief Gets the pointer of the active information matrix
  /// @return Information matrix pointer
  constexpr auto* operator->() const { return &get(); }

  /// @brief Gets the reference of the active information matrix
  /// @return Information matrix reference
  constexpr auto& get() { return std::get<Derived::kInformation>(matrix_); }

  /// @brief Gets the reference of the active information matrix
  /// @return Information matrix reference
  constexpr auto& get() const { return std::get<Derived::kInformation>(matrix_); }

 private:
  /// @brief Holds the informarion matrix.
  MatrixVariant matrix_;
};

}  // namespace variants
}  // namespace optimization
}  // namespace graph

#endif  // VORTEX_OPTIMIZATION_VARIANTS_INFORMATION_MATRIX_HPP
