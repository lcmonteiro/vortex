/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
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

#include "sources/base/graph.hpp"
#include "sources/base/math.hpp"
#include "sources/dual/dual.hpp"
#include "sources/helpers/index.hpp"
#include "sources/helpers/invoke.hpp"
#include "sources/optimization/graph_config.hpp"
#include "sources/optimization/graph_operations.hpp"
#include "sources/optimization/variants/information_matrix.hpp"
#include "sources/optimization/variants/robust_kernel.hpp"

namespace vortex::graph {
namespace optimization {
using detail::Types;

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
class Edge : public detail::TypesBuild<graph::Edge, Nodes> {
  using KernelVariant = variants::KernelVariant<Derived, Config>;

 public:
  ///< The dimension of the edge.
  static constexpr size_t kDimension = Dimension;

  ///< Default information matrix type.
  static constexpr size_t kInformation = variants::kIdentityMatrix;

  /// @brief information matrix alternatives
  using InformationVariant = variants::InformationVariant<Derived, Config, Dimension>;

  ///< Default kernel type.
  static constexpr size_t kKernel = variants::kNullKernel;

  /// @brief robust kernel alternatives.
  using KernelTypes = variants::KernelVariant<Derived, Config>;

  ///< Helper alises types.
  template <size_t I>
  using Node = detail::TypesElementBuild<I, Nodes>;
  using Number = typename Config::Number;
  using Measurement = Type;
  using Base = detail::TypesBuild<graph::Edge, Nodes>;
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
  const Measurement& measurement() const { return measurement_; }

  /// @brief Sets measurement.
  /// @param value The new measurement value.
  void measurement(const Measurement& value) { measurement_ = value; }

  /// @brief Set information matrix. Must be called with at least one argument.
  /// @tparam T Type of the first value.
  /// @tparam ...Ts Type of the other values.
  /// @param value The matrix to set the information matrix to.
  /// @param ...values The other values.
  template <class T, class... Ts>
  void information(T&& value, Ts&&... values) {
    information_->set(std::forward<T>(value), std::forward<Ts>(values)...);
  }

  /// @brief Get the informaion matrix.
  /// @return The information matrix.
  const auto& information() const { return information_->get(); }

  /// @brief Gets the chi-squared value
  /// @return chi-squared value
  Number chi2() const { return kernel_->chi2(); }

  /// @brief Gets the current error value
  /// @return error value
  const auto& error() const { return error_; }

  /// @brief Robust Kernel Public Accessor
  struct KernelVariantAcessor {
    explicit KernelVariantAcessor(KernelVariant& variant) : variant_{variant} {}

    /// @brief Sets the robust kernel delta
    /// @param delta The new delta value.
    void delta(const Number value) { variant_->delta(value); }

    /// @brief Gets the robust kernel delta
    Number delta() const { return variant_->delta(); }

   private:
    KernelVariant& variant_;
  } kernel{kernel_};

  /// @brief Updates the error value and chi-squared value.
  void updateError() {
    const auto residual = detail::invoke(
        ErrorCall{self()},           // Invoke the error function of the derived edge
        EstimationCallback{self()},  // Get the estimation values of the connected nodes
        detail::Expand<Base::NNodes>{});
    std::copy(std::cbegin(residual), std::cend(residual), std::begin(error_));
    kernel_->update(math::dot(error_, information() * error_));
  }

  /// @brief Updates the jacobian values for all nodes.
  void updateJacobians() {
    graph::detail::for_each(graph::detail::Indexes<Base::NNodes>{},
                            JacobiansUpdateCallback{self()});
  }

  /// @brief apply a function to each H block
  /// @param callable function that receives the nodes and the block value
  template <class Fn>
  void forEachHBlock(Fn&& callable) {
    detail::for_each_pair(detail::Indexes<Base::NNodes>{},
                          HBlockCallback<Fn>{this, std::forward<Fn>(callable)});
  }

  /// @brief apply a function to each B block
  /// @param callable function that receives the nodes and the block value
  template <class Fn>
  void forEachBBlock(Fn&& callable) {
    detail::for_each(detail::Indexes<Base::NNodes>{},
                     BBlockCallback<Fn>{this, std::forward<Fn>(callable)});
  }

 protected:
  /// @brief Jacobian Types and values.
  /// We have two jacobian types the normal version and the transpose
  /// version to minimize math operations.
  /// @note we will assume the transpose version has included the information.
  /// matrix
  template <typename Node>
  using JacobianTransposeMatrix =
      math::HybridMatrix<Number, Node::kMaxDimension, kDimension, math::rowMajor>;
  template <typename Node>
  using JacobianMatrix = math::HybridMatrix<Number, kDimension, Node::kMaxDimension>;

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

    const auto node_dimension = GetNode<I>(*this)->dimension();
    auto jacobian = JacobianMatrix<NodeType>(kDimension, node_dimension, Number{0});

    const auto residual = detail::invoke(
        ErrorCall{self()},                                  // Invoke the error function
        DualEstimationCallback<I>{self(), node_dimension},  // Get the dual estimations
        detail::Expand<Base::NNodes>{});

    for (size_t row{0}; row < kDimension; ++row) {
      const auto& partials = residual[row];
      assert(partials.size() <= node_dimension);
      for (size_t col{0}; col < partials.size(); ++col) {
        jacobian(row, col) = partials.dvalue(col);
      }
    }
    return jacobian;
  }

 private:
  /// @brief Helper function for casting to derived type.
  /// @return A pointer to derived type.
  Derived* self() { return static_cast<Derived*>(this); }

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
    constexpr decltype(auto) operator()() {
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
    size_t update_dimension;
    template <size_t I>
    auto operator()() {
      const auto dimension = GetNode<I>(*self)->dimension();
      std::vector<Dual> delta;
      delta.reserve(dimension);
      for (size_t index{0}; index < dimension; ++index) {
        if constexpr (UpdateIndex == I) {
          delta.emplace_back(Number{0}, index);
        } else {
          delta.emplace_back(Number{0});
        }
      }
      return GetNode<I>(*self)->plus(delta);
    }
  };

  /// @brief Updates the two jabobian forms.
  struct JacobiansUpdateCallback {
    Derived* self;

    template <class T>
    auto robustify(const T& information) {
      return self->kernel_->robustify(information);
    }

    template <size_t I>
    void operator()() {
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
    void operator()() {
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
    void operator()() {
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
  using JacobianTransposeTuple = detail::TypesWrapBuild<std::tuple, JacobianTransposeMatrix, Nodes>;
  using JacobianTuple = detail::TypesWrapBuild<std::tuple, JacobianMatrix, Nodes>;
  JacobianTransposeTuple jacobian_transpose_{};
  JacobianTuple jacobian_{};
};

}  // namespace optimization
}  // namespace vortex::graph

#endif  // VORTEX_OPTIMIZATION_GRAPH_EDGE_HPP
