/// ===============================================================================================
/// @file
///
/// @brief vortex.graph component
/// ===============================================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_EDGE_HPP
#define VORTEX_OPTIMIZATION_GRAPH_EDGE_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <utility>

#include "foundation/dual.hpp"
#include "foundation/graph.hpp"
#include "foundation/math.hpp"
#include "helpers/contracts.hpp"
#include "helpers/invoke.hpp"
#include "helpers/unroll.hpp"
#include "optimization/graph_config.hpp"
#include "optimization/graph_operations.hpp"
#include "optimization/variants/information_matrix.hpp"
#include "optimization/variants/robust_kernel.hpp"

namespace vortex::optimization {

/// ===============================================================================================
/// @brief Type Container: A utility template to define nodes types.
/// ===============================================================================================
template <class... Ts>
using Nodes = helpers::types<Ts...>;

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
  using kernel_variant = variants::KernelVariant<Derived, Config>;
  /// @brief Information matrix alternatives.
  using information_variant = variants::InformationVariant<Derived, Config, Dimension>;

 public:
  /// @brief The dimension of the edge.
  static constexpr std::size_t dimension = Dimension;

  /// @brief Default information matrix type.
  static constexpr std::size_t information_option = variants::kIdentityMatrix;

  /// @brief Default kernel type.
  static constexpr std::size_t kernel_option = variants::null_kernel;

  /// @brief Robust kernel alternatives.
  using kernel_types = variants::KernelVariant<Derived, Config>;

  /// @brief Helper alias types.
  template <std::size_t I>
  using node_type = helpers::types_element_build_t<I, Nodes>;
  using number_type = typename Config::number_type;
  using measurement_type = Type;
  using base_type = helpers::types_build_t<graph::edge, Nodes>;
  using matrix_type = math::static_matrix<number_type, dimension, dimension>;

  template <class Scalar>
  using error_type = math::static_vector<Scalar, dimension>;

  using base_type::base_type;

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
  struct KernelVariantAcessor {
    explicit KernelVariantAcessor(kernel_variant& variant) : variant_{variant} {}

    /// @brief Sets the robust kernel delta.
    /// @param value The new delta value.
    auto delta(const number_type value) -> void { variant_->delta(value); }

    /// @brief Gets the robust kernel delta.
    auto delta() const -> number_type { return variant_->delta(); }

   private:
    kernel_variant& variant_;
  } kernel{kernel_};

  /// @brief Updates the error, chi-squared value, and both jacobian forms
  /// for every connected node in a single evaluation of `error(...)`.
  auto update() -> void {
    // Evaluate the derived edge's `error(...)` with dual estimations for every node at once.
    const auto residual = helpers::invoke(  // Invoke error function with dual estimations
        ErrorCallback{self()},              // Invoke the error function of the derived edge
        EstimationCallback{self()},         // Get the dual estimation
        helpers::Expand<base_type::n_nodes>{});

    // Copy the dual residual into the error vector and update the robust kernel's chi-squared.
    std::transform(             // Copy the dual residual into the error vector
        std::cbegin(residual),  // Begin of the dual residual
        std::cend(residual),    // End of the dual residual
        std::begin(error_),     // Write to the error vector
        [](const auto& component) { return component.value(); });
    kernel_->update(math::dot(error_, information() * error_));

    // Extract the jacobian and jacobian_transpose blocks for each node from the dual residual.
    helpers::unroll(                              // Iterate over each node index
        helpers::Indexes<base_type::n_nodes>{},   // Index sequence for the number of nodes
        JacobianUpdateCallback{self(), residual}  // update the jacobian and jacobian_transposs
    );
  }

  /// @brief Applies a function to each H block.
  /// @param callable Function that receives the nodes and the block value.
  template <class Fn>
  auto forEachHBlock(Fn&& callable) -> void {
    helpers::unroll_pair(helpers::Indexes<base_type::n_nodes>{},
                         HBlockCallback<Fn>{this, std::forward<Fn>(callable)});
  }

  /// @brief Applies a function to each B block.
  /// @param callable Function that receives the nodes and the block value.
  template <class Fn>
  auto forEachBBlock(Fn&& callable) -> void {
    helpers::unroll(helpers::Indexes<base_type::n_nodes>{},
                    BBlockCallback<Fn>{this, std::forward<Fn>(callable)});
  }

 protected:
  /// @brief Jacobian types and values.
  /// We have two jacobian types: the normal version and the transpose
  /// version to minimize math operations.
  /// @note The transpose version is assumed to already include the
  /// information matrix.
  template <typename Node>
  using jacobian_matrix_t_type = math::static_matrix<number_type, Node::kDimension, dimension, math::row_major>;
  template <typename Node>
  using jacobian_matrix = math::static_matrix<number_type, dimension, Node::kDimension>;

  /// @brief `kNodeDimension[I]` is `node_type<I>::kDimension`, in `Nodes` order.
  static constexpr auto kNodeDimension = []<std::size_t... Is>(std::index_sequence<Is...>) {
    return std::array<std::size_t, sizeof...(Is)>{node_type<Is>::kDimension...};
  }(std::make_index_sequence<base_type::n_nodes>{});

  /// @brief `kNodeOffset[I]` is node `I`'s starting index in the edge's combined tangent space.
  static constexpr auto kNodeOffset = []<std::size_t I, std::size_t... Is>(std::index_sequence<I, Is...>) {
    std::array<std::size_t, sizeof...(Is) + 1> offset{0};
    ((offset[Is] = offset[Is - 1] + kNodeDimension[Is - 1]), ...);
    return offset;
  }(std::make_index_sequence<base_type::n_nodes>{});

 private:
  /// @brief Helper function for casting to derived type.
  /// @return A pointer to derived type.
  auto self() -> Derived* { return static_cast<Derived*>(this); }

  /// @brief Invoke the error function
  struct ErrorCallback {
    Derived* self;
    template <class... T>
    auto operator()(T&&... values) {
      return self->error(std::forward<T>(values)...);
    }
  };

  /// @brief Produces dual-number estimations for *every* connected node at
  /// once, each seeded at its own non-overlapping range within the edge's
  /// combined tangent space.
  struct EstimationCallback {
    Derived* self;
    template <std::size_t I>
    auto operator()() {
      constexpr auto D = std::get<I>(kNodeDimension);
      constexpr auto O = std::get<I>(kNodeOffset);
      return GetNode<I>(*self)->plus(dual::zeros<number_type, D, O>());
    }
  };

  /// @brief Reads node `I`'s jacobian block from its offset range in the combined dual residual.
  template <class Residual>
  struct JacobianUpdateCallback {
    Derived* self;
    const Residual& residual;

    template <class T>
    auto robustify(const T& information) {
      return self->kernel_->robustify(information);
    }

    template <std::size_t I>
    auto operator()() -> void {
      using NodeType = node_type<I>;
      constexpr auto D = std::get<I>(kNodeDimension);
      constexpr auto O = std::get<I>(kNodeOffset);

      auto& jacobian = std::get<I>(self->jacobian_);
      VORTEX_ASSERT(dimension == std::size(residual), "residual size mismatch");
      for (std::size_t row{0}; row < dimension; ++row) {
        const auto& partials = residual[row];
        const auto partials_size = std::clamp(std::size(partials) - O, std::size_t{0}, D);
        for (std::size_t col{0}; col < partials_size; ++col) {
          jacobian(row, col) = partials.dvalue(O + col);
        }
      }

      auto& jacobian_transpose = std::get<I>(self->jacobian_transpose_);
      jacobian_transpose = math::trans(jacobian) * robustify(self->information());
    }
  };

  /// @brief Computes B block for each node.
  template <class Fn>
  struct BBlockCallback {
    edge* self;
    Fn callback;

    template <std::size_t I>
    auto operator()() -> void {
      auto& node = GetNode<I>(*self);
      if (not node->disable()) {
        const auto block = std::get<I>(self->jacobian_transpose_) * self->error_;
        callback(node, block);
      }
    }
  };

  /// @brief Compute H block for each pair of nodes.
  template <class Fn>
  struct HBlockCallback {
    edge* self;
    Fn callback;

    template <std::size_t I, std::size_t J>
    auto operator()() -> void {
      auto& node_i = GetNode<I>(*self);
      auto& node_j = GetNode<J>(*self);
      if ((not node_i->disable()) and (not node_j->disable())) {
        const auto block = std::get<I>(self->jacobian_transpose_) * std::get<J>(self->jacobian_);
        callback(node_i, node_j, block);
      }
    }
  };

  /// @brief Measurement value.
  measurement_type measurement_{};

  /// @brief Error vector.
  error_type<number_type> error_{};

  /// @brief Information matrix.
  information_variant information_{};

  /// @brief Robust kernel alternatives.
  kernel_variant kernel_{};

  /// @brief Jacobian types.
  using tuple_jacobian_t = helpers::types_wrap_build_t<std::tuple, jacobian_matrix_t_type, Nodes>;
  using tuple_jacobian = helpers::types_wrap_build_t<std::tuple, jacobian_matrix, Nodes>;
  tuple_jacobian_t jacobian_transpose_{};
  tuple_jacobian jacobian_{};
};

}  // namespace vortex::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_EDGE_HPP
