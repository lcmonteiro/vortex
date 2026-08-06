/// ===========================================================================
/// @file
///
/// @brief vortex.graph component
/// ===========================================================================
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

namespace vortex::graph::optimization {

/// ===========================================================================
/// @brief Type Container: A utility template to define nodes types.
/// ===========================================================================
template <class... Ts>
using Nodes = helpers::types<Ts...>;

/// ===========================================================================
/// @brief Edge: Represents a graph edge used for measurements.
///
/// @tparam Derived    The derived edge type.
/// @tparam Dimension  The dimension of the edge.
/// @tparam Type       The data type of the edge's measurement.
/// @tparam Nodes      The type of nodes connected by the edge.
/// @tparam Config     Configuration settings for the edge.
/// ===========================================================================
template <class Derived, auto Dimension, class Type, class Nodes, class Config = DefaultConfig>
class Edge : public helpers::types_build_t<graph::Edge, Nodes> {
  using KernelVariant = variants::KernelVariant<Derived, Config>;

 public:
  /// @brief The dimension of the edge.
  static constexpr std::size_t kDimension = Dimension;

  /// @brief Default information matrix type.
  static constexpr std::size_t kInformation = variants::kIdentityMatrix;

  /// @brief Information matrix alternatives.
  using InformationVariant = variants::InformationVariant<Derived, Config, Dimension>;

  /// @brief Default kernel type.
  static constexpr std::size_t kKernel = variants::kNullKernel;

  /// @brief Robust kernel alternatives.
  using KernelTypes = variants::KernelVariant<Derived, Config>;

  /// @brief Helper alias types.
  template <std::size_t I>
  using Node = helpers::types_element_build_t<I, Nodes>;
  using Number = typename Config::Number;
  using Measurement = Type;
  using Base = helpers::types_build_t<graph::Edge, Nodes>;
  using Matrix = math::StaticMatrix<Number, kDimension, kDimension>;
  using Vector = math::StaticVector<Number, kDimension>;
  using Base::Base;

  template <class Scalar>
  using Error = std::array<Scalar, kDimension>;

  /// @brief Gets measurement.
  /// @return The current measurement value.
  auto measurement() const -> const Measurement& { return measurement_; }

  /// @brief Sets measurement.
  /// @param value The new measurement value.
  auto measurement(const Measurement& value) -> void { measurement_ = value; }

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
  auto chi2() const -> Number { return kernel_->chi2(); }

  /// @brief Gets the current error value.
  /// @return The error value.
  const auto& error() const { return error_; }

  /// @brief Robust kernel public accessor.
  struct KernelVariantAcessor {
    explicit KernelVariantAcessor(KernelVariant& variant) : variant_{variant} {}

    /// @brief Sets the robust kernel delta.
    /// @param value The new delta value.
    auto delta(const Number value) -> void { variant_->delta(value); }

    /// @brief Gets the robust kernel delta.
    auto delta() const -> Number { return variant_->delta(); }

   private:
    KernelVariant& variant_;
  } kernel{kernel_};

  /// @brief Updates the error, chi-squared value, and both jacobian forms
  /// for every connected node in a single evaluation of `error(...)`.
  auto update() -> void {
    // Evaluate the derived edge's `error(...)` with dual estimations for every node at once.
    const auto residual = helpers::invoke(  // Invoke error function with dual estimations
        ErrorCallback{self()},              // Invoke the error function of the derived edge
        EstimationCallback{self()},         // Get the dual estimation
        helpers::Expand<Base::NNodes>{});

    // Copy the dual residual into the error vector and update the robust kernel's chi-squared.
    std::transform(             // Copy the dual residual into the error vector
        std::cbegin(residual),  // Begin of the dual residual
        std::cend(residual),    // End of the dual residual
        std::begin(error_),     // Write to the error vector
        [](const auto& component) { return component.value(); });
    kernel_->update(math::dot(error_, information() * error_));

    // Extract the jacobian and jacobian_transpose blocks for each node from the dual residual.
    helpers::unroll(                              // Iterate over each node index
        helpers::Indexes<Base::NNodes>{},         // Index sequence for the number of nodes
        JacobianUpdateCallback{self(), residual}  // update the jacobian and jacobian_transposs
    );
  }

  /// @brief Applies a function to each H block.
  /// @param callable Function that receives the nodes and the block value.
  template <class Fn>
  auto forEachHBlock(Fn&& callable) -> void {
    helpers::unroll_pair(helpers::Indexes<Base::NNodes>{},
                         HBlockCallback<Fn>{this, std::forward<Fn>(callable)});
  }

  /// @brief Applies a function to each B block.
  /// @param callable Function that receives the nodes and the block value.
  template <class Fn>
  auto forEachBBlock(Fn&& callable) -> void {
    helpers::unroll(helpers::Indexes<Base::NNodes>{},
                    BBlockCallback<Fn>{this, std::forward<Fn>(callable)});
  }

 protected:
  /// @brief Jacobian types and values.
  /// We have two jacobian types: the normal version and the transpose
  /// version to minimize math operations.
  /// @note The transpose version is assumed to already include the
  /// information matrix.
  template <typename Node>
  using JacobianMatrixT = math::StaticMatrix<Number, Node::kDimension, kDimension, math::rowMajor>;
  template <typename Node>
  using JacobianMatrix = math::StaticMatrix<Number, kDimension, Node::kDimension>;

  /// @brief `kNodeDimension[I]` is `Node<I>::kDimension`, in `Nodes` order.
  static constexpr auto kNodeDimension = []<std::size_t... Is>(std::index_sequence<Is...>) {
    return std::array<std::size_t, sizeof...(Is)>{Node<Is>::kDimension...};
  }(std::make_index_sequence<Base::NNodes>{});

  /// @brief `kNodeOffset[I]` is node `I`'s starting index in the edge's combined tangent space.
  static constexpr auto kNodeOffset = []<std::size_t I, std::size_t... Is>(std::index_sequence<I, Is...>) {
    std::array<std::size_t, sizeof...(Is) + 1> offset{0};
    ((offset[Is] = offset[Is - 1] + kNodeDimension[Is - 1]), ...);
    return offset;
  }(std::make_index_sequence<Base::NNodes>{});

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
      return GetNode<I>(*self)->plus(dual::zeros<Number, D, O>());
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
      using NodeType = Node<I>;
      constexpr auto D = std::get<I>(kNodeDimension);
      constexpr auto O = std::get<I>(kNodeOffset);

      auto& jacobian = std::get<I>(self->jacobian_);
      VORTEX_ASSERT(kDimension == std::size(residual), "residual size mismatch");
      for (std::size_t row{0}; row < kDimension; ++row) {
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
    Edge* self;
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
    Edge* self;
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
  Measurement measurement_{};

  /// @brief Error vector.
  Vector error_{};

  /// @brief Information matrix.
  InformationVariant information_{};

  /// @brief Robust kernel alternatives.
  KernelVariant kernel_{};

  /// @brief Jacobian types.
  using JacobianTupleT = helpers::types_wrap_build_t<std::tuple, JacobianMatrixT, Nodes>;
  using JacobianTuple = helpers::types_wrap_build_t<std::tuple, JacobianMatrix, Nodes>;
  JacobianTupleT jacobian_transpose_{};
  JacobianTuple jacobian_{};
};

}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_EDGE_HPP
