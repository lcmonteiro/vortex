/// ===============================================================================================
/// @file
///
/// @brief vortex.graph component
/// ===============================================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_ALGORITHM_LEVENBERG_HPP
#define VORTEX_OPTIMIZATION_GRAPH_ALGORITHM_LEVENBERG_HPP

#include <algorithm>
#include <cmath>

#include "foundation/math.hpp"
#include "helpers/expected.hpp"
#include "helpers/numeric.hpp"
#include "optimization/graph_algorithm.hpp"

namespace vortex::optimization {

/// ===============================================================================================
/// @brief Constexpr-friendly identity cast
/// ===============================================================================================
template <class Type>
inline constexpr auto constant(Type v) -> Type {
  return v;
}

/// ===============================================================================================
/// @brief Default configuration used by levenberg_algorithm.
/// ===============================================================================================
struct LevenbergConfig {
  static constexpr auto LambdaInit{0.0};
  static constexpr auto LowerScale{1. / 3.0};
  static constexpr auto UpperScale{2. / 3.0};
  static constexpr auto Tau{1e-5};
  static constexpr auto Retries{5U};
  static constexpr auto RhoEpsilon{1e-9};
};

/// ===============================================================================================
/// @brief Levenberg-Marquardt optimization algorithm.
/// ===============================================================================================
template <class Graph, class GraphSolver, class Config = LevenbergConfig>
class levenberg_algorithm : public algorithm<Graph, GraphSolver> {
  using base_type = algorithm<Graph, GraphSolver>;
  using number_type = typename Graph::number_type;

 public:
  explicit levenberg_algorithm(Graph& graph)
      : base_type(graph), lambda_{-1.}, lambda_factor_{2.}, current_chi_{0.} {}

  levenberg_algorithm(const levenberg_algorithm&) = delete;
  levenberg_algorithm(levenberg_algorithm&&) = default;
  ~levenberg_algorithm() = default;

  auto operator=(const levenberg_algorithm&) -> levenberg_algorithm& = delete;
  auto operator=(levenberg_algorithm&&) -> levenberg_algorithm& = default;

  /// @brief Algorithm initialization
  /// @return nothing or an algorithm error
  auto init(bool reset) -> helpers::expected<void, algorithm_error> {
    auto result = base_type::init(reset);
    if (not result) {
      return result;
    }

    if (reset) {
      if (not solver_.buildStructure()) {
        return helpers::unexpected(algorithm_error::fail);
      }
    }

    return {};
  }

  /// @brief Performs a Levenberg-Marquardt solve iteration.
  /// @return Whether the optimization converged, or an algorithm error.
  template <bool first_iteration, bool last_iteration>
  auto solve() -> helpers::expected<bool, algorithm_error> {
    if constexpr (first_iteration) {
      graph_.update_edges();
      solver_.buildSystem();
      lambda_ = compute_lambda_init();
      lambda_factor_ = 2;
      current_chi_ = graph_.chi2();
    }

    for (std::size_t it = 0; it < constant(Config::Retries); ++it) {
      solver_.updateDiagonal(lambda_);

      if (solver_.solve()) {
        graph_.push();
        graph_.update_nodes(solver_.x());
        graph_.update_edges();

        const number_type chi = graph_.chi2();
        const number_type scale = compute_scale() + constant<number_type>(1e-3);
        const number_type rho = (current_chi_ - chi) / scale;

        // If the improvement ratio (rho) is positive,
        // it continues adjusting lambda
        if (rho > constant<number_type>(Config::RhoEpsilon)) {
          if constexpr (not last_iteration) {
            adjust_lambda_adaptive(rho);
            solver_.buildSystem();
            current_chi_ = chi;
          }
          return false;  // Continue solving
        }

        // If rho is not negative,
        // it indicates sufficient improvement, and it can stop
        if (rho > constant<number_type>(-Config::RhoEpsilon)) {
          return true;  // Optimization converged
        }

        graph_.pull();
      }

      // On failure, increase lambda aggressively to slow down optimization
      increase_lambda_aggressive();
      solver_.restoreDiagonal();

      // Check for numeric instability in lambda
      if (not std::isfinite(lambda_)) {
        return helpers::unexpected(algorithm_error::numeric_limit);
      }
    }
    // Retries exhausted without ever accepting an improving step -- this is
    // not convergence, so it must not be reported the same as `return true`
    // above (which means "found a good enough step and can stop").
    return helpers::unexpected(algorithm_error::not_converged);
  }

  /// @brief Retrieves the lambda value from the algorithm.
  /// @return The lambda value of type `number_type`.
  auto lambda() const -> number_type { return lambda_; }

 protected:
  /// @brief Computes the initial lambda or returns the one from config.
  /// @return The initial lambda value.
  auto compute_lambda_init() {
    if constexpr (Config::LambdaInit > 0) {
      return constant<number_type>(Config::LambdaInit);
    } else {
      return constant<number_type>(Config::Tau) * math::max(math::diagonal(solver_.h()));
    }
  }

  /// @brief Computes the scaler for the chi2 difference.
  /// @return The scaler value.
  auto compute_scale() -> number_type {
    return math::dot(solver_.x(), (solver_.x() * lambda_) + solver_.b());
  }

  /// @brief Increases lambda aggressively when the step fails.
  auto increase_lambda_aggressive() -> void {
    lambda_ *= lambda_factor_;
    lambda_factor_ *= 2;
  }

  /// @brief Adjusts lambda adaptively when the step succeeds.
  /// Scales lambda based on the `rho` value, ensuring smooth convergence.
  /// @param rho Gain ratio, indicating how much to scale lambda.
  auto adjust_lambda_adaptive(number_type rho) -> void {
    constexpr number_type alpha_min = constant<number_type>(Config::LowerScale);
    constexpr number_type alpha_max = constant<number_type>(Config::UpperScale);
    const number_type alpha = 1. - helpers::int_pow<3>((2 * rho) - 1);
    const number_type alpha_clamp = std::clamp(alpha, alpha_min, alpha_max);
    lambda_ *= alpha_clamp;
    lambda_factor_ = 2;
  }

 private:
  using base_type::graph_;
  using base_type::solver_;
  // Levenberg state
  number_type lambda_;
  number_type lambda_factor_;
  number_type current_chi_;
};

}  // namespace vortex::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_ALGORITHM_LEVENBERG_HPP
