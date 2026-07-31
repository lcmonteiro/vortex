/// ===========================================================================
/// @file
///
/// @brief vortex.graph component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_EDGE_HPP
#define VORTEX_OPTIMIZATION_GRAPH_EDGE_HPP

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <iterator>
#include <vector>

#include "foundation/dual/dual.hpp"
#include "foundation/graph/graph.hpp"
#include "foundation/math/math.hpp"
#include "helpers/index.hpp"
#include "helpers/invoke.hpp"
#include "optimization/graph_config.hpp"
#include "optimization/graph_operations.hpp"
#include "optimization/variants/information_matrix.hpp"
#include "optimization/variants/robust_kernel.hpp"

namespace vortex::graph::optimization {
using helpers::Types;

/// ===========================================================================
/// @brief Type Container: A utility template to define nodes types.
/// ===========================================================================
template <class... Ts>
using Nodes = Types<Ts...>;

/// ===========================================================================
/// @brief Edge: Represents a graph edge used for measurements.
///
/// @tparam Derived    The derived edge type.
/// @tparam Dimension  The dimension of the edge.
/// @tparam Type       The data type of the edge's measurement.
/// @tparam Nodes      The type of nodes connected by the edge.
/// @tparam Config     Configuration settings for the edge.
/// ===========================================================================
template <class Derived, size_t Dimension, class Type, class Nodes, class Config = DefaultConfig>
class Edge : public helpers::TypesBuild<graph::Edge, Nodes> {
  using KernelVariant = variants::KernelVariant<Derived, Config>;

 public:
  /// @brief The dimension of the edge.
  static constexpr size_t kDimension = Dimension;

  /// @brief Default information matrix type.
  static constexpr size_t kInformation = variants::kIdentityMatrix;

  /// @brief Information matrix alternatives.
  using InformationVariant = variants::InformationVariant<Derived, Config, Dimension>;

  /// @brief Default kernel type.
  static constexpr size_t kKernel = variants::kNullKernel;

  /// @brief Robust kernel alternatives.
  using KernelTypes = variants::KernelVariant<Derived, Config>;

  /// @brief Helper alias types.
  template <size_t I>
  using Node = helpers::TypesElementBuild<I, Nodes>;
  using Number = typename Config::Number;
  using Measurement = Type;
  using Base = helpers::TypesBuild<graph::Edge, Nodes>;
  using Matrix = math::StaticMatrix<Number, kDimension, kDimension>;
  using Vector = math::StaticVector<Number, kDimension>;
  using Base::Base;

  /// @brief Dual number scalar used for automatic differentiation.
  using Dual = ::vortex::dual::number<Number>;

  /// @brief Scalar-generic error container returned by a derived edge.
  ///
  /// A derived edge must implement a templated `error(...)` that accepts the
  /// connected node estimations expressed in the scalar type `Scalar` and
  /// returns an `Error<Scalar>` (i.e. `std::array<Scalar, kDimension>`). This
  /// lets the same cost function be evaluated with `Number` (to obtain the
  /// residual) and with `Dual` (to obtain exact Jacobians).
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

  /// @brief Updates the error value and chi-squared value.
  auto updateError() -> void {
    const auto residual = helpers::invoke(
        ErrorCall{self()},           // Invoke the error function of the derived edge
        EstimationCallback{self()},  // Get the estimation values of the connected nodes
        helpers::Expand<Base::NNodes>{});
    std::copy(std::cbegin(residual), std::cend(residual), std::begin(error_));
    kernel_->update(math::dot(error_, information() * error_));
  }

  /// @brief Updates the jacobian values for all nodes.
  auto updateJacobians() -> void {
    helpers::for_each(helpers::Indexes<Base::NNodes>{}, JacobiansUpdateCallback{self()});
  }

  /// @brief Applies a function to each H block.
  /// @param callable Function that receives the nodes and the block value.
  template <class Fn>
  auto forEachHBlock(Fn&& callable) -> void {
    helpers::for_each_pair(helpers::Indexes<Base::NNodes>{},
                           HBlockCallback<Fn>{this, std::forward<Fn>(callable)});
  }

  /// @brief Applies a function to each B block.
  /// @param callable Function that receives the nodes and the block value.
  template <class Fn>
  auto forEachBBlock(Fn&& callable) -> void {
    helpers::for_each(helpers::Indexes<Base::NNodes>{},
                      BBlockCallback<Fn>{this, std::forward<Fn>(callable)});
  }

 protected:
  /// @brief Jacobian types and values.
  /// We have two jacobian types: the normal version and the transpose
  /// version to minimize math operations.
  /// @note The transpose version is assumed to already include the
  /// information matrix.
  template <typename Node>
  using JacobianMatrixT = math::HybridMatrix<Number, Node::kDimension, kDimension, math::rowMajor>;
  template <typename Node>
  using JacobianMatrix = math::HybridMatrix<Number, kDimension, Node::kDimension>;

  /// @brief Calculates the jacobian via forward-mode automatic
  /// differentiation using dual numbers.
  ///
  /// The tangent increment of node @p I is seeded with independent dual
  /// variables (indices 0..dim-1) while every other node is evaluated at a
  /// constant (zero) dual increment. The derived edge's templated `error(...)`
  /// is then evaluated once with dual estimations, and the exact partial
  /// derivatives are read directly from the resulting dual residuals.
  /// @tparam I node index.
  /// @return The jacobian matrix.
  template <size_t I>
  auto jacobian() {
    using NodeType = Node<I>;

    const auto residual = helpers::invoke(  //
        ErrorCall{self()},                  // Invoke the error function
        DualEstimationCallback<I>{self()},  // Get the dual estimations
        helpers::Expand<Base::NNodes>{});

    auto jacobian = JacobianMatrix<NodeType>(kDimension, NodeType::kDimension, Number{0});
    for (size_t row{0}; row < kDimension; ++row) {
      const auto& partials = residual[row];
      assert(partials.size() <= NodeType::kDimension);
      for (size_t col{0}; col < partials.size(); ++col) {
        jacobian(row, col) = partials.dvalue(col);
      }
    }
    return jacobian;
  }

 private:
  /// @brief Helper function for casting to derived type.
  /// @return A pointer to derived type.
  auto self() -> Derived* { return static_cast<Derived*>(this); }

  /// @brief Invoke the error function
  struct ErrorCall {
    Derived* self;
    template <class... T>
    auto operator()(T&&... values) {
      return self->error(std::forward<T>(values)...);
    }
  };

  /// @brief Gets the estimation value given a node index.
  struct EstimationCallback {
    Derived* self;
    template <size_t I>
    constexpr auto operator()() -> decltype(auto) {
      return GetNode<I>(*self)->estimation();
    }
  };

  /// @brief Produces dual-number estimations used for automatic
  /// differentiation. The tangent increment of node @p UpdateIndex is seeded
  /// with independent dual variables (value 0, derivative 1 at its own index)
  /// while every other node receives a constant (zero) dual increment. Feeding
  /// these through each node's `plus(delta)` yields the estimation expressed in
  /// the dual scalar type.
  /// @tparam UpdateIndex Index of the node being differentiated.
  template <size_t UpdateIndex>
  struct DualEstimationCallback {
    Derived* self;
    template <size_t I>
    auto operator()() {
      using NodeType = Node<I>;

      std::array<Dual, NodeType::kDimension> delta;
      for (size_t index{0}; index < NodeType::kDimension; ++index) {
        if constexpr (UpdateIndex == I) {
          delta[index] = Dual{Number{0}, index};
        } else {
          delta[index] = Dual{Number{0}};
        }
      }
      return GetNode<I>(*self)->plus(delta);
    }
  };

  /// @brief Updates the two jacobian forms.
  struct JacobiansUpdateCallback {
    Derived* self;

    template <class T>
    auto robustify(const T& information) {
      return self->kernel_->robustify(information);
    }

    template <size_t I>
    auto operator()() -> void {
      auto& jacobian = std::get<I>(self->jacobian_);
      auto& jacobian_transpose = std::get<I>(self->jacobian_transpose_);
      jacobian = self->template jacobian<I>();
      jacobian_transpose = math::trans(jacobian) * robustify(self->information());
    }
  };

  /// @brief Computes B block for each node.
  template <class Fn>
  struct BBlockCallback {
    Edge* self;
    Fn callback;

    template <size_t I>
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

    template <size_t I, size_t J>
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
  using JacobianTupleT = helpers::TypesWrapBuild<std::tuple, JacobianMatrixT, Nodes>;
  using JacobianTuple = helpers::TypesWrapBuild<std::tuple, JacobianMatrix, Nodes>;
  JacobianTupleT jacobian_transpose_{};
  JacobianTuple jacobian_{};
};

}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_EDGE_HPP
