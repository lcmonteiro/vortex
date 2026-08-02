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
#include <utility>
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

  /// @brief Scalar-generic error container returned by a derived edge.
  ///
  /// A derived edge implements a templated `error(...)` that accepts the
  /// connected node estimations with arbitrary scalar types and returns an
  /// `Error<Scalars...>`. The element type is deduced from the arithmetic result
  /// of the supplied scalar types, allowing residuals to be evaluated with
  /// `Number`, `Dual`, or a mixture of compatible scalar types.
  template <class... Scalars>
  using Error = std::array<std::decay_t<decltype((std::declval<Scalars>() + ...))>, kDimension>;

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
  ///
  /// Evaluates the derived edge's `error(...)` with plain (non-dual) node
  /// estimations, and updates the robust kernel's chi-squared statistic
  /// from the resulting residual. This does not touch the jacobians; use
  /// `update()` for a combined error + jacobian refresh.
  auto updateError() -> void {
    const auto residual = helpers::invoke(
        ErrorCall{self()},           // Invoke the error function of the derived edge
        EstimationCallback{self()},  // Get the estimation values of the connected nodes
        helpers::Expand<Base::NNodes>{});
    std::copy(std::cbegin(residual), std::cend(residual), std::begin(error_));
    kernel_->update(math::dot(error_, information() * error_));
  }

  /// @brief Updates the jacobian values for all nodes.
  ///
  /// @note This evaluates the derived edge's `error(...)` once per
  /// connected node (`Base::NNodes` evaluations total), each time seeding
  /// only that node's tangent increment with independent dual variables.
  /// Prefer `update()` when both the error and the jacobians are needed,
  /// since it performs the equivalent work with a single evaluation of
  /// `error(...)`.
  auto updateJacobians() -> void {
    helpers::for_each(helpers::Indexes<Base::NNodes>{}, JacobiansUpdateCallback{self()});
  }

  /// @brief Updates the error, chi-squared value, and both jacobian forms
  /// for every connected node in a single evaluation of `error(...)`.
  ///
  /// Unlike `updateJacobians()`, which seeds one node at a time and
  /// re-evaluates `error(...)` once per node, this seeds *all* connected
  /// nodes' tangent increments simultaneously into one combined dual
  /// vector (see `DualEstimationCallback2` / `kNodeOffset`), so a single
  /// call to `error(...)` yields, in its dual residual, the exact partial
  /// derivatives with respect to every node at once. The scalar part of
  /// the residual doubles as the plain error value, so this also replaces
  /// `updateError()`.
  auto update() -> void {
    const auto residual = helpers::invoke(  //
        ErrorCall{self()},                  // Invoke the error function of the derived edge
        DualEstimationCallback2{self()},    // Seed every node's tangent at once
        helpers::Expand<Base::NNodes>{});

    // The scalar part of each dual residual component is the plain error.
    std::transform(             //
        std::cbegin(residual),  //
        std::cend(residual),    //
        std::begin(error_),     //
        [](const auto& component) { return component.value(); });
    kernel_->update(math::dot(error_, information() * error_));

    // Extract each node's jacobian / jacobian_transpose block from the
    // single dual residual above, reading the derivative range that
    // corresponds to that node's offset in the combined tangent space.
    helpers::for_each(                     //
        helpers::Indexes<Base::NNodes>{},  //
        JacobianExtractCallback{self(), residual});
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

  /// @brief Dimension of each connected node, in `Nodes` order.
  ///
  /// `kNodeDimension[I] == Node<I>::kDimension`. Built once at compile
  /// time by expanding `I = 0 .. Base::NNodes - 1` and reading each node
  /// type's `kDimension`.
  static constexpr auto kNodeDimension = []<size_t... Is>(std::index_sequence<Is...>) {
    return std::array<size_t, sizeof...(Is)>{Node<Is>::kDimension...};
  }(std::make_index_sequence<Base::NNodes>{});

  /// @brief Offset of each connected node within the edge's combined
  /// tangent space (the concatenation of every connected node's tangent
  /// increment).
  ///
  /// `kNodeOffset[I]` is the exclusive prefix sum of `kNodeDimension`:
  /// `kNodeOffset[0] == 0` and `kNodeOffset[I] == kNodeOffset[I - 1] +
  /// kNodeDimension[I - 1]` for `I > 0`. This is the derivative index at
  /// which node `I`'s dual seed range starts when all nodes are seeded
  /// simultaneously (see `DualEstimationCallback2`).
  ///
  /// @note The peeled first index `I` in the lambda's parameter pack is
  /// only used to size the resulting `std::array` back to `Base::NNodes`
  /// (`sizeof...(Is) + 1`); `offset[0]` stays `0` from the aggregate
  /// initializer and the fold below only ever writes `offset[Is]` for
  /// `Is >= 1`.
  static constexpr auto kNodeOffset = []<size_t I, size_t... Is>(std::index_sequence<I, Is...>) {
    std::array<size_t, sizeof...(Is) + 1> offset{0};
    ((offset[Is] = offset[Is - 1] + kNodeDimension[Is - 1]), ...);
    return offset;
  }(std::make_index_sequence<Base::NNodes>{});

  /// @brief Total size of the edge's combined tangent space, i.e. the sum
  /// of every connected node's dimension.
  static constexpr size_t kTangentSize =
      kNodeOffset[Base::NNodes - 1] + kNodeDimension[Base::NNodes - 1];

  /// @brief Calculates the jacobian via forward-mode automatic
  /// differentiation using dual numbers.
  ///
  /// The tangent increment of node @p I is seeded with independent dual
  /// variables (indices 0..dim-1) while every other node is evaluated at a
  /// constant (zero) dual increment. The derived edge's templated `error(...)`
  /// is then evaluated once with dual estimations, and the exact partial
  /// derivatives are read directly from the resulting dual residuals.
  ///
  /// @note This evaluates `error(...)` once for this node alone. When
  /// jacobians for every node are needed, prefer `update()`, which seeds
  /// all nodes at once and reads every node's block from a single
  /// evaluation.
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
      if constexpr (UpdateIndex == I) {
        constexpr auto D = std::get<I>(kNodeDimension);
        return GetNode<I>(*self)->plus(dual::zeros<Number, D>());
      } else {
        return GetNode<I>(*self)->estimation();
      }
    }
  };

  /// @brief Produces dual-number estimations for *every* connected node at
  /// once, each seeded at its own non-overlapping range within the edge's
  /// combined tangent space.
  ///
  /// Node `I`'s tangent increment is seeded with `kNodeDimension[I]`
  /// independent dual variables starting at derivative index
  /// `kNodeOffset[I]` (value 0, unit derivative at its own index; see
  /// `dual::zeros`). Because every node is seeded in the same call, a
  /// single evaluation of the derived edge's `error(...)` yields, in the
  /// dual residual, the exact partial derivatives with respect to *all*
  /// connected nodes simultaneously — the block for node `I` is read from
  /// derivative indices `[kNodeOffset[I], kNodeOffset[I] +
  /// kNodeDimension[I])` (see `JacobianExtractCallback`).
  ///
  /// @note Compared to `DualEstimationCallback<I>`, which seeds one node
  /// at a time and requires `Base::NNodes` evaluations of `error(...)` to
  /// cover every node, this trades a larger per-operation dual vector
  /// (size `kTangentSize` instead of a single node's dimension) for a
  /// single evaluation of `error(...)`. Whether this is a net win depends
  /// on the relative cost of evaluating `error(...)` versus the cost of
  /// dual arithmetic over the larger combined vector.
  struct DualEstimationCallback2 {
    Derived* self;
    template <size_t I>
    auto operator()() {
      constexpr auto D = std::get<I>(kNodeDimension);
      constexpr auto O = std::get<I>(kNodeOffset);
      return GetNode<I>(*self)->plus(dual::zeros<Number, D, O>());
    }
  };

  /// @brief Updates the two jacobian forms for a single node by evaluating
  /// `error(...)` once for that node alone (via `jacobian<I>()`).
  ///
  /// @note Used by `updateJacobians()`, which calls this once per
  /// connected node. Prefer `update()` / `JacobianExtractCallback` when
  /// jacobians for every node are needed, to avoid the repeated
  /// evaluations of `error(...)`.
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

  /// @brief Extracts the jacobian and jacobian_transpose block for a
  /// single node from a residual that was computed with *every* node
  /// seeded simultaneously (see `DualEstimationCallback2`).
  ///
  /// Reads each residual row's derivative components in the range
  /// `[kNodeOffset[I], kNodeOffset[I] + kNodeDimension[I])`, which
  /// corresponds to node `I`'s tangent increment within the combined dual
  /// vector.
  ///
  /// @tparam Residual Type of the dual residual produced by evaluating
  /// `error(...)` with `DualEstimationCallback2`.
  template <class Residual>
  struct JacobianExtractCallback {
    Derived* self;
    const Residual& residual;

    template <class T>
    auto robustify(const T& information) {
      return self->kernel_->robustify(information);
    }

    template <size_t I>
    auto operator()() -> void {
      using NodeType = Node<I>;
      constexpr auto D = std::get<I>(kNodeDimension);
      constexpr auto O = std::get<I>(kNodeOffset);

      auto& jacobian = std::get<I>(self->jacobian_);
      for (size_t row{0}; row < kDimension; ++row) {
        const auto& partials = residual[row];
        assert(O + D <= partials.size());
        for (size_t col{0}; col < D; ++col) {
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
