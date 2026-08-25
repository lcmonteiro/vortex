/// ===============================================================================================
/// @file
///
/// @brief vortex.graph component
/// ===============================================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_EDGE_HPP
#define VORTEX_OPTIMIZATION_GRAPH_EDGE_HPP

#include <array>
#include <cstddef>
#include <iterator>
#include <span>
#include <utility>

#include "vortex/foundation/dual.hpp"
#include "vortex/foundation/graph.hpp"
#include "vortex/foundation/graph/storage.hpp"
#include "vortex/foundation/math.hpp"
#include "vortex/helpers/contracts.hpp"
#include "vortex/helpers/indices.hpp"
#include "vortex/helpers/invoke.hpp"
#include "vortex/helpers/unroll.hpp"
#include "vortex/optimization/graph_config.hpp"
#include "vortex/optimization/graph_operations.hpp"
#include "vortex/optimization/variants/information_matrix.hpp"
#include "vortex/optimization/variants/robust_kernel.hpp"

namespace vortex::optimization {
using graph::nodes;

/// ===============================================================================================
/// @brief Edge: Represents a graph edge used for measurements.
///
/// @tparam Derived    The derived edge type.
/// @tparam Dimension  The dimension of the edge.
/// @tparam Type       The data type of the edge's measurement.
/// @tparam Nodes      The type of nodes connected by the edge.
/// @tparam Config     Configuration settings for the edge.
/// ===============================================================================================
template <class Derived, auto Dimension, class Type, class Nodes, class Config = default_config>
class edge : public helpers::types_build_t<graph::edge, Nodes> {
  using kernel_variant = variants::kernel_variant<Derived, Config>;
  /// @brief Information matrix alternatives.
  using information_variant = variants::information_variant<Derived, Config, Dimension>;

  /// @brief Helper alias types.
  template <std::size_t I>
  using node_type_at = helpers::types_element_build_t<I, Nodes>;
  using base_type = helpers::types_build_t<graph::edge, Nodes>;
  using number_type = typename Config::number_type;

 public:
  /// @brief Default information matrix type.
  static constexpr std::size_t information_option = variants::identity_information;

  /// @brief Default kernel type.
  static constexpr std::size_t kernel_option = variants::null_kernel;

  /// @brief Measurement type of the edge.
  using measurement_type = Type;

  /// @brief Information matrix type of the edge.
  using information_matrix = math::static_matrix<number_type, Dimension, Dimension>;

  /// @brief Error type of the edge.
  template <class Scalar>
  using error_vector = math::static_vector<Scalar, Dimension>;

  /// @brief Constructor of the edge.
  using base_type::base_type;

  /// @brief Gets the dimension of the node.
  /// @return The dimension of the node.
  static constexpr auto dimension() -> std::size_t { return Dimension; }

  /// @brief Gets measurement.
  /// @return The current measurement value.
  auto measurement() const -> const measurement_type& { return measurement_; }

  /// @brief Sets measurement.
  /// @param value The new measurement value.
  auto measurement(const measurement_type& value) -> void { measurement_ = value; }

  /// @brief Set information matrix. Must be called with at least one argument.
  /// @tparam T Type of the first value.
  /// @tparam ...Ts Type of the other values.
  /// @param value The matrix to set the information matrix to.
  /// @param ...values The other values.
  template <class T, class... Ts>
  auto information(T&& value, Ts&&... values) -> void {
    information_->set(std::forward<T>(value), std::forward<Ts>(values)...);
  }

  /// @brief Gets the information matrix.
  /// @return The information matrix.
  const auto& information() const { return information_->get(); }

  /// @brief Gets the chi-squared value.
  /// @return The chi-squared value.
  auto chi2() const -> number_type { return kernel_->chi2(); }

  /// @brief Gets the current error value.
  /// @return The error value.
  const auto& error() const { return error_; }

  /// @brief Robust kernel public accessor.
  struct kernel_variant_accessor {
    explicit kernel_variant_accessor(kernel_variant& variant) : variant_{variant} {}

    /// @brief Sets the robust kernel delta.
    /// @param value The new delta value.
    auto delta(const number_type value) -> void { variant_->delta(value); }

    /// @brief Gets the robust kernel delta.
    auto delta() const -> number_type { return variant_->delta(); }

   private:
    kernel_variant& variant_;
  } kernel{kernel_};

  /// @brief Updates the error, chi-squared value, and both jacobian forms for every connected node
  /// in a single evaluation of `error(...)`.
  auto update() -> void {
    // compute the residuals and their derivatives with respect to each node's.
    const auto residuals = helpers::invoke(
        [this](auto&&... estimations) {
          return self()->error(std::forward<decltype(estimations)>(estimations)...);
        },
        [this]<auto I> {
          constexpr auto D = std::get<I>(node_dimensions);
          constexpr auto O = std::get<I>(node_offsets);
          const auto delta = dual::zeros<number_type, D, O>();
          return get_node<I>(*this)->plus(std::span{std::data(delta), std::size(delta)});
        },
        helpers::expand<base_type::n_nodes>{});
    VORTEX_ASSERT(dimension() == std::size(residuals), "residual size mismatch");

    // update the error vector with the real values of the residuals.
    helpers::unroll(helpers::range<dimension()>{}, [this, &residuals]<auto I> {
      const auto& residual = residuals[I];
      error_[I] = residual.value();
    });

    // update kernel and chi-squared value.
    kernel_->update(math::dot(error_, information() * error_));

    // update jacobians and their transposes.
    helpers::unroll(helpers::range<base_type::n_nodes>{}, [this]<auto I> {
      auto& jacobian = std::get<I>(jacobian_);
      jacobian.reset();
    });
    helpers::unroll(helpers::range<dimension()>{}, [this, &residuals]<auto I> {
      const auto& residual = residuals[I];
      const auto end = std::cend(residual.dvalues());
      auto it = std::cbegin(residual.dvalues());
      helpers::unroll(helpers::range<base_type::n_nodes>{}, [this, &it, end]<auto J> {
        constexpr auto D = std::get<J>(node_dimensions);
        constexpr auto O = std::get<J>(node_offsets);
        auto& jacobian = std::get<J>(jacobian_);
        for (; it != end and it->index < (O + D); ++it) {
          jacobian(I, it->index - O) = it->value;
        }
      });
    });
    helpers::unroll(helpers::range<base_type::n_nodes>{}, [this]<auto I> {
      const auto& jacobian = std::get<I>(jacobian_);
      auto& jacobian_transpose = std::get<I>(jacobian_transpose_);
      jacobian_transpose = math::trans(jacobian) * kernel_->robustify(information());
    });
  }

  /// @brief Applies a function to each H block.
  /// @param callable Function that receives the nodes and the block value.
  template <class Fn>
  auto foreach_h_block(Fn&& callable) -> void {
    const auto h_block_wrapper = [this, &callable]<auto I, auto J> {
      auto& node_i = get_node<I>(*this);
      auto& node_j = get_node<J>(*this);
      if ((not node_i->disable()) and (not node_j->disable())) {
        const auto block = std::get<I>(this->jacobian_transpose_) * std::get<J>(this->jacobian_);
        callable(node_i, node_j, block);
      }
    };
    helpers::unroll_pair(helpers::range<base_type::n_nodes>{}, h_block_wrapper);
  }

  /// @brief Applies a function to each B block.
  /// @param callable Function that receives the nodes and the block value.
  template <class Fn>
  auto foreach_b_block(Fn&& callable) -> void {
    const auto b_block_wrapper = [this, &callable]<auto I> {
      auto& node = get_node<I>(*this);
      if (not node->disable()) {
        const auto block = std::get<I>(this->jacobian_transpose_) * this->error_;
        callable(node, block);
      }
    };
    helpers::unroll(helpers::range<base_type::n_nodes>{}, b_block_wrapper);
  }

 protected:
  /// @brief Jacobian types and values.
  /// We have two jacobian types: the normal version and the transpose  /// version to minimize math
  /// operations.
  /// @note The transpose version is assumed to already include the
  /// information matrix.
  template <class Node>
  using jacobian_matrix =
      math::static_matrix<number_type, dimension(), Node::dimension(), math::column_major>;
  template <class Node>
  using jacobian_matrix_t =
      math::static_matrix<number_type, Node::dimension(), dimension(), math::row_major>;

  /// @brief Node dimensions in the combined tangent space.
  static constexpr auto node_dimensions = []<auto... Is>(helpers::indices<Is...>) {
    return std::array<std::size_t, sizeof...(Is)>{node_type_at<Is>::dimension()...};
  }(helpers::make_indices<base_type::n_nodes>{});

  /// @brief Node offsets in the combined tangent space.
  static constexpr auto node_offsets = []<auto I, auto... Is>(helpers::indices<I, Is...>) {
    std::array<std::size_t, sizeof...(Is) + 1> offset{0};
    ((offset[Is] = offset[Is - 1] + node_dimensions[Is - 1]), ...);
    return offset;
  }(helpers::make_indices<base_type::n_nodes>{});

 private:
  /// @brief Helper function for casting to derived type.
  /// @return A pointer to derived type.
  auto self() -> Derived* { return static_cast<Derived*>(this); }

  /// @brief Measurement value.
  measurement_type measurement_{};

  /// @brief Error value.
  error_vector<number_type> error_{};

  /// @brief Information matrix.
  information_variant information_{};

  /// @brief Robust kernel alternatives.
  kernel_variant kernel_{};

  /// @brief Jacobian types.
  using jacobian_matrix_tuple = helpers::types_wrap_build_t<std::tuple, jacobian_matrix, Nodes>;
  using jacobian_matrix_t_tuple = helpers::types_wrap_build_t<std::tuple, jacobian_matrix_t, Nodes>;
  jacobian_matrix_tuple jacobian_{};
  jacobian_matrix_t_tuple jacobian_transpose_{};
};

}  // namespace vortex::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_EDGE_HPP
