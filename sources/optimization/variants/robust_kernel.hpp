/// ===========================================================================
/// @file
///
/// @brief vortex.optimization.variants component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_VARIANTS_ROBUST_KERNEL_HPP
#define VORTEX_OPTIMIZATION_VARIANTS_ROBUST_KERNEL_HPP

#include <cstddef>
#include <variant>

namespace vortex::graph::optimization {
namespace variants {

/// ===========================================================================
/// @brief Robust Kernels
/// ===========================================================================
template <class Number>
struct NullKernel {
  /// @brief Updates the chi-squared value.
  /// @param chi2 The chi-squared value.
  auto update(Number chi2) -> void { chi2_ = chi2; }

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
  Number chi2_;
};

/// @brief Huber robust kernel: down-weights large residuals to reduce the
/// influence of outliers.
/// @tparam Number Scalar type used for the kernel computation.
template <class Number>
struct HuberKernel {
  /// @brief Setter for robust kernel delta.
  /// @param value New delta value.
  auto delta(Number value) -> void {
    delta_ = value;
    delta_sqr_ = value * value;
  }

  /// @brief Getter for robust kernel delta.
  /// @return The current delta value.
  auto delta() const -> Number { return delta_; }

  /// @brief Updates the internal state.
  /// @param chi2 The chi-squared value.
  auto update(Number chi2) -> void {
    if (chi2 <= delta_sqr_) {
      rho_ = chi2;
      rho_prime_ = 1.;
    } else {
      const Number chi2_sqrt = std::sqrt(chi2);
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
  Number delta_{1.};
  Number delta_sqr_;
  Number rho_;
  Number rho_prime_;
};

/// ===========================================================================
/// @brief Robust Kernels Alternatives
/// ===========================================================================

/// @brief Robust kernel options.
constexpr std::size_t kNullKernel = 0;
constexpr std::size_t kHuberKernel = 1;

/// @brief Robust kernel variant.
/// @tparam Derived The derived edge type.
/// @tparam Config The edge configuration.
template <class Derived, class Config>
struct KernelVariant {
  using Number = typename Config::Number;
  using Storage = std::variant<NullKernel<Number>, HuberKernel<Number>>;

  constexpr KernelVariant() : storage_(std::variant_alternative_t<Derived::kKernel, Storage>{}) {
    static_assert(Derived::kKernel >= kNullKernel, "kKernel >= kNullKernel");
    static_assert(Derived::kKernel <= kHuberKernel, "kKernel <= kHuberKernel");
  }

  /// @brief Gets the pointer of the active kernel.
  /// @return Kernel pointer.
  constexpr auto* operator->() { return &get(); }
  constexpr auto* operator->() const { return &get(); }

  /// @brief Gets the reference of the active kernel.
  /// @return Kernel reference.
  constexpr auto& get() { return std::get<Derived::kKernel>(storage_); }
  constexpr auto& get() const { return std::get<Derived::kKernel>(storage_); }

 private:
  Storage storage_;
};

}  // namespace variants
}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_VARIANTS_ROBUST_KERNEL_HPP
