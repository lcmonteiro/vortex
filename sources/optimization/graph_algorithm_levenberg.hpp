/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.graph component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_ALGORITHM_LEVENBERG_HPP
#define VORTEX_OPTIMIZATION_GRAPH_ALGORITHM_LEVENBERG_HPP

#include <algorithm>
#include <cmath>
#include <expected>

#include "base/math.hpp"
#include "helpers/compat.hpp"
#include "optimization/graph_algorithm.hpp"

namespace vortex::graph::optimization {

/// ===========================================================================
/// Default Config
/// @brief this is a default configuration used on LevenbergAlgorithm
/// ===========================================================================
struct LevenbergConfig {
  static constexpr auto LambdaInit{0.0};
  static constexpr auto LowerScale{1. / 3.0};
  static constexpr auto UpperScale{2. / 3.0};
  static constexpr auto Tau{1e-5};
  static constexpr auto Retries{5U};
  static constexpr auto RhoEpsilon{1e-9};
};
template <class Type>
inline constexpr Type Constant(Type v) {
  return v;
}

/// ===========================================================================
/// LevenbergAlgorithm
/// ===========================================================================
template <class Graph, class GraphSolver, class Config = LevenbergConfig>
class LevenbergAlgorithm : public Algorithm<Graph, GraphSolver> {
  using Base = Algorithm<Graph, GraphSolver>;
  using Number = typename Graph::Number;

 public:
  explicit LevenbergAlgorithm(Graph& graph)
      : Base(graph), lambda_{-1.}, lambda_factor_{2.}, current_chi_{0.} {}
  ~LevenbergAlgorithm() = default;
  LevenbergAlgorithm(const LevenbergAlgorithm&) = delete;
  LevenbergAlgorithm& operator=(const LevenbergAlgorithm&) = delete;
  LevenbergAlgorithm(LevenbergAlgorithm&&) = default;
  LevenbergAlgorithm& operator=(LevenbergAlgorithm&&) = default;

  /// @brief Algorithm initialization
  /// @return nothing or an algorithm error
  std::expected<void, AlgorithmError> init(bool reset) {
    auto result = Base::init(reset);
    if (not result) {
      return result;
    }

    if (reset) {
      if (not solver_.buildStructure()) {
        return std::unexpected(AlgorithmError::FAIL);
      }
    }

    return {};
  }

  /// @brief Algorithm solve
  /// @return a boolean value whether it was solved or an algorithm error
  template <bool first_iteration, bool last_iteration>
  std::expected<bool, AlgorithmError> solve() {
    if constexpr (first_iteration) {
      graph_.updateErrors();
      solver_.buildSystem();
      lambda_ = computeLambdaInit();
      lambda_factor_ = 2;
      current_chi_ = graph_.computeChi2();
    }

    for (size_t it = 0; it < Constant(Config::Retries); ++it) {
      solver_.updateDiagonal(lambda_);

      if (solver_.solve()) {
        graph_.push();
        graph_.updateEstimations(solver_.x());
        graph_.updateErrors();

        const Number chi = graph_.computeChi2();
        const Number scale = computeScale() + Constant<Number>(1e-3);
        const Number rho = (current_chi_ - chi) / scale;

        // If the improvement ratio (rho) is positive,
        // it continues adjusting lambda
        if (rho > Constant<Number>(Config::RhoEpsilon)) {
          if constexpr (not last_iteration) {
            adjustLambdaAdaptive(rho);
            solver_.buildSystem();
            current_chi_ = chi;
          }
          return false;  // Continue solving
        }

        // If rho is not negative,
        // it indicates sufficient improvement, and it can stop
        if (rho > Constant<Number>(-Config::RhoEpsilon)) {
          return true;  // Optimization converged
        }

        graph_.pull();
      }

      // On failure, increase lambda aggressively to slow down optimization
      increaseLambdaAggressive();
      solver_.restoreDiagonal();

      // Check for numeric instability in lambda
      if (not std::isfinite(lambda_)) {
        return std::unexpected(AlgorithmError::NUMERIC_LIMIT);
      }
    }
    return true;
  }

  /// @brief Retrieves the lambda value from the algorithm.
  /// @return The lambda value of type `Number`.
  Number lambda() const { return lambda_; }

 protected:
  /// @brief computes the initial lambda or return the one from config
  /// @return lambda
  auto computeLambdaInit() {
    if constexpr (Config::LambdaInit > 0) {
      return Constant<Number>(Config::LambdaInit);
    } else {
      return Constant<Number>(Config::Tau) * math::max(math::diagonal(solver_.h()));
    }
  }

  /// @brief computes the scaler for the chi2 difference
  /// @return scaler
  Number computeScale() { return math::dot(solver_.x(), (solver_.x() * lambda_) + solver_.b()); }

  /// @brief Increases lambda aggressively when the step fails.
  void increaseLambdaAggressive() {
    lambda_ *= lambda_factor_;
    lambda_factor_ *= 2;
  }

  /// @brief Adjusts lambda adaptively when the step succeeds.
  /// Scales lambda based on the `rho` value, ensuring smooth convergence.
  /// @param rho Gain ratio, indicating how much to scale lambda.
  void adjustLambdaAdaptive(Number rho) {
    constexpr Number alpha_min = Constant<Number>(Config::LowerScale);
    constexpr Number alpha_max = Constant<Number>(Config::UpperScale);
    const Number alpha = 1. - graph::detail::int_pow<3>((2 * rho) - 1);
    const Number alpha_clamp = std::clamp(alpha, alpha_min, alpha_max);
    lambda_ *= alpha_clamp;
    lambda_factor_ = 2;
  }

 private:
  using Base::graph_;
  using Base::solver_;
  // Levenberg state
  Number lambda_;
  Number lambda_factor_;
  Number current_chi_;
};

}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_ALGORITHM_LEVENBERG_HPP
