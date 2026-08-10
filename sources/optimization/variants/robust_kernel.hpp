/// ===============================================================================================
/// @file
///
/// @brief vortex.optimization.variants component
/// ===============================================================================================
#ifndef VORTEX_OPTIMIZATION_VARIANTS_ROBUST_KERNEL_HPP
#define VORTEX_OPTIMIZATION_VARIANTS_ROBUST_KERNEL_HPP

#include <cmath>
#include <cstddef>
#include <variant>

namespace vortex::optimization {
namespace variants {

/// ===============================================================================================
/// @brief Robust Kernels
/// ===============================================================================================
template <class Number>
struct null_kernel_option {
  using number_type = Number;

  /// @brief Updates the chi-squared value.
  /// @param chi2 The chi-squared value.
  auto update(number_type chi2) -> void { chi2_ = chi2; }

  /// @brief Gets the chi-squared value.
  /// @return The chi-squared value.
  auto chi2() const { return chi2_; }

  /// @brief Robustifies the information matrix.
  /// @tparam Information The information matrix type.
  /// @param information The information matrix.
  /// @return The robustified information.
  template <class Information>
  const auto& robustify(const Information& information) {
    return information;
  }

 private:
  number_type chi2_;
};

/// @brief Huber robust kernel: down-weights large residuals to reduce the influence of outliers.
/// @tparam Number Scalar type used for the kernel computation.
template <class Number>
struct huber_kernel_option {
  using number_type = Number;
  
  /// @brief Setter for robust kernel delta.
  /// @param value New delta value.
  auto delta(number_type value) -> void {
    delta_ = value;
    delta_sqr_ = value * value;
  }

  /// @brief Getter for robust kernel delta.
  /// @return The current delta value.
  auto delta() const -> number_type { return delta_; }

  /// @brief Updates the internal state.
  /// @param chi2 The chi-squared value.
  auto update(number_type chi2) -> void {
    if (chi2 <= delta_sqr_) {
      rho_ = chi2;
      rho_prime_ = 1.;
    } else {
      const number_type chi2_sqrt = std::sqrt(chi2);
      rho_ = (2 * chi2_sqrt * delta_) - delta_sqr_;
      rho_prime_ = delta_ / chi2_sqrt;
    }
  };

  /// @brief Gets the chi-squared value.
  /// @return The chi-squared value.
  auto chi2() const { return rho_; }

  /// @brief Robustifies the information matrix.
  /// @tparam Information The information matrix type.
  /// @param information The information matrix.
  /// @return The robustified information.
  template <class Information>
  auto robustify(const Information& information) {
    return rho_prime_ * information;
  }

 private:
  number_type delta_{1.};
  number_type delta_sqr_;
  number_type rho_;
  number_type rho_prime_;
};

/// ===============================================================================================
/// @brief Robust Kernels Alternatives
/// ===============================================================================================

/// @brief Robust kernel options.
constexpr std::size_t null_kernel = 0;
constexpr std::size_t huber_kernel = 1;

/// @brief Robust kernel variant.
/// @tparam Derived The derived edge type.
/// @tparam Config The edge configuration.
template <class Derived, class Config>
struct kernel_variant {
  using number_type = typename Config::number_type;
  using storage_type = std::variant<null_kernel_option<number_type>, huber_kernel_option<number_type>>;

  constexpr kernel_variant()
      : storage_(std::variant_alternative_t<Derived::kernel_option, storage_type>{}) {
    static_assert(Derived::kernel_option >= null_kernel, "kernel_option >= null_kernel");
    static_assert(Derived::kernel_option <= huber_kernel, "kernel_option <= huber_kernel");
  }

  /// @brief Gets the pointer of the active kernel.
  /// @return Kernel pointer.
  constexpr auto* operator->() { return &get(); }
  constexpr auto* operator->() const { return &get(); }

  /// @brief Gets the reference of the active kernel.
  /// @return Kernel reference.
  constexpr auto& get() { return std::get<Derived::kernel_option>(storage_); }
  constexpr auto& get() const { return std::get<Derived::kernel_option>(storage_); }

 private:
  storage_type storage_;
};

}  // namespace variants
}  // namespace vortex::optimization

#endif  // VORTEX_OPTIMIZATION_VARIANTS_ROBUST_KERNEL_HPP
