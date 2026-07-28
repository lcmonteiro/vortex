/// ===========================================================================
/// @file
///
/// @brief vortex.optimization.variants component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_VARIANTS_ROBUST_KERNEL_HPP
#define VORTEX_OPTIMIZATION_VARIANTS_ROBUST_KERNEL_HPP

#include <variant>

namespace vortex::graph::optimization {
namespace variants {

/// ===========================================================================
/// @brief Robust Kernels
/// ===========================================================================
template <class Number>
struct NullKernel {
  /// @brief Update the chi-squared value
  /// @param chi2 The chi-squared value
  auto update(Number chi2) -> void { chi2_ = chi2; }

  /// @brief Gets the chi-squared value
  /// @return chi-squared value
  auto chi2() const { return chi2_; }

  /// @brief Robustify information
  /// @tparam Information
  /// @param information
  /// @return The robustified Version
  template <class Information>
  const auto& robustify(const Information& information) {
    return information;
  }

 private:
  Number chi2_;
};

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

  /// @brief Update the internal state
  /// @param chi2 The chi-squared value
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

  /// @brief Gets the chi-squared value
  /// @return chi-squared value
  auto chi2() const { return rho_; }

  /// @brief Robustify information
  /// @tparam Information
  /// @param information
  /// @return The robustified Version
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

/// @brief Robust Kernel Options
constexpr size_t kNullKernel = 0;
constexpr size_t kHuberKernel = 1;

/// @brief Robust Kernel Variant
/// @tparam Derived
template <class Derived, class Config>
struct KernelVariant {
  using Number = typename Config::Number;
  using Storage = std::variant<NullKernel<Number>, HuberKernel<Number>>;

  constexpr KernelVariant() : storage_(std::variant_alternative_t<Derived::kKernel, Storage>{}) {
    static_assert(Derived::kKernel >= kNullKernel, "kKernel >= kNullKernel");
    static_assert(Derived::kKernel <= kHuberKernel, "kKernel <= kHuberKernel");
  }

  /// @brief gets the pointer of the active kernel
  /// @return kernel pointer
  constexpr auto* operator->() { return &get(); }
  constexpr auto* operator->() const { return &get(); }

  /// @brief gets the reference of the active kernel
  /// @return kernel reference
  constexpr auto& get() { return std::get<Derived::kKernel>(storage_); }
  constexpr auto& get() const { return std::get<Derived::kKernel>(storage_); }

 private:
  Storage storage_;
};

}  // namespace variants
}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_VARIANTS_ROBUST_KERNEL_HPP
