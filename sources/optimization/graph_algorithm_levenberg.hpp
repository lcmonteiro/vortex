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
#include "helpers/utility.hpp"
#include "optimization/graph_algorithm.hpp"

namespace vortex::optimization {

/// ===============================================================================================
/// @brief Default configuration used by levenberg_algorithm.
/// ===============================================================================================
struct levenberg_config {
  static constexpr auto lambda_init{0.0};
  static constexpr auto lower_scale{1. / 3.0};
  static constexpr auto upper_scale{2. / 3.0};
  static constexpr auto tau{1e-5};
  static constexpr auto retries{5U};
  static constexpr auto rho_epsilon{1e-9};
};

/// ===============================================================================================
/// @brief Levenberg-Marquardt optimization algorithm.
/// ===============================================================================================
template <class Graph, class GraphSolver, class Config = levenberg_config>
class levenberg_algorithm : public algorithm<Graph, GraphSolver> {
  using config_type = Config;
  using base_type = algorithm<Graph, GraphSolver>;
  using number_type = typename Graph::number_type;

 public:
  /// @brief Constructor of the Levenberg-Marquardt optimization algorithm.
  /// @param graph The graph to be optimized.
  explicit levenberg_algorithm(Graph& graph)
      : base_type(graph), lambda_{-1.}, lambda_factor_{2.}, current_chi_{0.} {}

  /// @brief Deleted copy constructor and defaulted move constructor.
  levenberg_algorithm(const levenberg_algorithm&) = delete;
  levenberg_algorithm(levenberg_algorithm&&) = default;
  ~levenberg_algorithm() = default;

  /// @brief Deleted copy assignment operator and defaulted move assignment operator.
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
      if (not solver_.build_structure()) {
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
      solver_.build_system();
      lambda_ = compute_lambda_init();
      lambda_factor_ = 2;
      current_chi_ = graph_.chi2();
    }

    for (std::size_t it = 0; it < helpers::constant(config_type::retries); ++it) {
      solver_.update_diagonal(lambda_);

      if (solver_.solve()) {
        graph_.push();
        graph_.update_nodes(solver_.x());
        graph_.update_edges();

        const number_type chi = graph_.chi2();
        const number_type scale = compute_scale() + helpers::constant<number_type>(1e-3);
        const number_type rho = (current_chi_ - chi) / scale;

        // If the improvement ratio (rho) is positive,
        // it continues adjusting lambda
        if (rho > helpers::constant<number_type>(config_type::rho_epsilon)) {
          if constexpr (not last_iteration) {
            update_lambda_adaptive(rho);
            solver_.build_system();
            current_chi_ = chi;
          }
          return false;  // Continue solving
        }

        // If rho is not negative,
        // it indicates sufficient improvement, and it can stop
        if (rho > helpers::constant<number_type>(-config_type::rho_epsilon)) {
          return true;  // Optimization converged
        }

        graph_.pull();
      }

      // On failure, increase lambda aggressively to slow down optimization
      update_lambda_aggressive();
      solver_.restore_diagonal();

      // Check for numeric instability in lambda
      if (not std::isfinite(lambda_)) {
        return helpers::unexpected(algorithm_error::numeric_limit);
      }
    }
    // Retries exhausted without accepting a step. Whether this is a failure
    // or simply "nothing left to improve" depends on context the algorithm
    // doesn't have (namely, whether earlier iterations already made
    // progress), so just report it and let the caller decide -- see
    // graph::optimize().
    return helpers::unexpected(algorithm_error::not_converged);
  }

  /// @brief Retrieves the lambda value from the algorithm.
  /// @return The lambda value of type `number_type`.
  auto lambda() const -> number_type { return lambda_; }

 protected:
  /// @brief Computes the initial lambda or returns the one from config.
  /// @return The initial lambda value.
  auto compute_lambda_init() {
    if constexpr (config_type::lambda_init > 0) {
      return helpers::constant<number_type>(config_type::lambda_init);
    } else {
      return helpers::constant<number_type>(config_type::tau) * math::max(math::diagonal(solver_.h()));
    }
  }

  /// @brief Computes the scale for the chi2 difference.
  /// @return The scale value.
  auto compute_scale() -> number_type {
    return math::dot(solver_.x(), (solver_.x() * lambda_) + solver_.b());
  }

  /// @brief Updates lambda aggressively when the step fails.
  auto update_lambda_aggressive() -> void {
    lambda_ *= lambda_factor_;
    lambda_factor_ *= 2;
  }

  /// @brief Updates lambda adaptively when the step succeeds.
  /// Scales lambda based on the `rho` value, ensuring smooth convergence.
  /// @param rho Gain ratio, indicating how much to scale lambda.
  auto update_lambda_adaptive(number_type rho) -> void {
    constexpr number_type alpha_min = helpers::constant<number_type>(config_type::lower_scale);
    constexpr number_type alpha_max = helpers::constant<number_type>(config_type::upper_scale);
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
