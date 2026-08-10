/// ===============================================================================================
/// @file
///
/// @brief vortex.optimization.variants component
/// ===============================================================================================
#ifndef VORTEX_OPTIMIZATION_VARIANTS_INFORMATION_MATRIX_HPP
#define VORTEX_OPTIMIZATION_VARIANTS_INFORMATION_MATRIX_HPP

#include <cstddef>
#include <variant>

#include "foundation/math.hpp"
#include "foundation/math/types.hpp"

namespace vortex::optimization {
namespace variants {

/// ===============================================================================================
/// @brief Information Matrices: Identity Matrix
/// ===============================================================================================

/// @brief An information matrix that is an identity matrix.
/// @tparam Number Type of the elements of the matrix.
/// @tparam Dimension The dimension the matrix. The matrix is square
/// (Dimension*Dimension).
template <class Number, std::size_t Dimension>
struct identity_information_option {
  using number_type = Number;

  /// @brief Constructor of an information matrix that is an identity matrix.
  identity_information_option() : matrix_{Dimension} {};

  /// @brief Gets the matrix instance.
  /// @return The matrix instance.
  const auto& get() const { return matrix_; }

 private:
  /// @brief The matrix instance.
  math::identity_matrix<number_type> matrix_;
};

/// ===============================================================================================
/// @brief Information Matrices: Diagonal Matrix
/// ===============================================================================================

/// @brief An information matrix that is a diagonal matrix.
/// @tparam Number Type of the elements of the matrix.
/// @tparam Dimension The dimension the matrix. The matrix is square
/// (Dimension*Dimension).
template <class Number, std::size_t Dimension>
struct diagonal_information_option {
  using number_type = Number;

  /// @brief Constructor of an information matrix that is a diagonal matrix.
  diagonal_information_option() : matrix_{math::identity_matrix<number_type>(Dimension)} {};

  /// @brief Sets the value of the information matrix based on another matrix.
  /// @tparam Matrix The matrix type.
  /// @param matrix The matrix containing the values to set the information to.
  /// @note Only enabled for matrix types.
  template <class Matrix>
  requires math::is_matrix_v<Matrix>
  auto set(const Matrix& matrix) -> void {
    matrix_ = matrix;
  }

  /// @brief Sets the value of the information matrix diagonal\ based on a
  /// vector.
  /// @tparam Vector The vector type.
  /// @param vector The vector containing the values to set the information
  /// diagonal to.
  /// @note Only enabled for vector types.
  template <class Vector>
  requires math::is_vector_v<Vector>
  auto set(const Vector& vector) -> void {
    math::diagonal(matrix_) = vector;
  }

  /// @brief Sets the value of the information matrix diagonal based on a group
  /// of values.
  /// @tparam Type Type of the first value.
  /// @tparam Types Types of the remaining values.
  /// @param value The first value.
  /// @param values The remaining values.
  template <class Type, class... Types>
  auto set(const Type& value, const Types&... values) -> void {
    // Ensure the number of received arguments matches the matrix dimension.
    static_assert((1 + sizeof...(Types)) == Dimension,
                  "The number of values must match the square matrix dimension.");
    math::diagonal(matrix_) = math::static_vector<Type, 1 + sizeof...(Types)>{value, values...};
  }

  /// @brief Gets the matrix instance.
  /// @return The matrix instance.
  const auto& get() const { return matrix_; }

 private:
  /// @brief Type of the base matrix.
  using base_matrix_type = math::static_matrix<number_type, Dimension, Dimension>;

  /// @brief The matrix instance.
  math::diagonal_matrix<base_matrix_type> matrix_;
};

/// ===============================================================================================
/// @brief Information Matrices: Normal Matrix
/// ===============================================================================================

/// @brief An information matrix.
/// @tparam Number Type of the elements of the matrix.
/// @tparam Dimension The dimension the matrix. The matrix is square
/// (Dimension*Dimension).
template <class Number, std::size_t Dimension>
struct symmetric_information_option {
  using number_type = Number;

  /// @brief Constructor of an information matrix.
  symmetric_information_option() : matrix_{math::identity_matrix<number_type>(Dimension)} {};

  /// @brief Sets the value of the information matrix based on another matrix.
  /// @tparam Matrix The matrix type.
  /// @param matrix The matrix containing the values to set the information to.
  /// @note Only enabled for matrix types.
  template <class Matrix>
  requires math::is_matrix_v<Matrix>
  auto set(const Matrix& matrix) -> void {
    matrix_ = matrix;
  }

  /// @brief Gets the matrix instance.
  /// @return The matrix instance.
  const auto& get() const { return matrix_; }

 private:
  /// @brief Type of the base matrix.
  using base_matrix_type = math::static_matrix<number_type, Dimension, Dimension>;

  /// @brief The matrix instance.
  math::symmetric_matrix<base_matrix_type> matrix_;
};

/// ===============================================================================================
/// @brief Information Matrix Alternatives
/// ===============================================================================================

/// @brief Information matrix options.
constexpr std::size_t identity_information = 0;
constexpr std::size_t diagonal_information = 1;
constexpr std::size_t symmetric_information = 2;

/// @brief Information matrix variant.
/// @tparam Derived The derived edge type.
/// @tparam Config The edge configuration.
/// @tparam Dimension The information matrix dimension.
template <class Derived, class Config, std::size_t Dimension>
struct information_variant {
  using number_type = typename Config::number_type;

  /// @brief Variant containing the Information matrix format options.
  using storage_type = std::variant<identity_information_option<number_type, Dimension>,
                                    diagonal_information_option<number_type, Dimension>,
                                    symmetric_information_option<number_type, Dimension>>;

  /// @brief Constructor of the information_variant, according to the option set
  /// in Derived::information_option.
  /// @warning Asserts if the value set in Derived::information_option is valid.
  constexpr information_variant()
      : matrix_(std::variant_alternative_t<Derived::information_option, storage_type>{}) {
    static_assert(Derived::information_option >= identity_information,
                  "information_option >= identity_information");
    static_assert(Derived::information_option <= symmetric_information,
                  "information_option <= symmetric_information");
  }

  /// @brief Gets the pointer of the active information matrix.
  /// @return Information matrix pointer.
  constexpr auto* operator->() { return &get(); }

  /// @brief Gets the pointer of the active information matrix.
  /// @return Information matrix pointer.
  constexpr auto* operator->() const { return &get(); }

  /// @brief Gets the reference of the active information matrix.
  /// @return Information matrix reference.
  constexpr auto& get() { return std::get<Derived::information_option>(matrix_); }

  /// @brief Gets the reference of the active information matrix.
  /// @return Information matrix reference.
  constexpr auto& get() const { return std::get<Derived::information_option>(matrix_); }

 private:
  /// @brief Holds the information matrix.
  storage_type matrix_;
};

}  // namespace variants
}  // namespace vortex::optimization

#endif  // VORTEX_OPTIMIZATION_VARIANTS_INFORMATION_MATRIX_HPP
