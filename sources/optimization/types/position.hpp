/// ===============================================================================================
/// @file
///
/// @brief vortex.optimization.types.position component
/// ===============================================================================================
#ifndef VORTEX_OPTIMIZATION_TYPES_POSITION_HPP
#define VORTEX_OPTIMIZATION_TYPES_POSITION_HPP

#include "optimization/graph_edge.hpp"
#include "optimization/graph_node.hpp"

namespace vortex::optimization::types {

/// @brief A scalar-generic 2D position / translation.
///
/// Used both as a node estimation type and as an edge measurement type. Being
/// templated on the scalar type lets the same value participate in real-valued
/// residual evaluation (`Number == double`) and in dual-number Jacobian
/// evaluation (forward-mode automatic differentiation).
///
/// @tparam Number Scalar type of the coordinates.
template <class Number>
struct Position {
  /// @brief coordinates of the position.
  Number x{};
  Number y{};

  /// @brief friend operators for arithmetic and comparison of Position objects.
  friend auto operator+(const Position<Number>& a, const Position<Number>& b) -> Position<Number> {
    return {a.x + b.x, a.y + b.y};
  }
  friend auto operator-(const Position<Number>& a, const Position<Number>& b) -> Position<Number> {
    return {a.x - b.x, a.y - b.y};
  }
  friend auto operator==(const Position<Number>& a, const Position<Number>& b) -> bool {
    return a.x == b.x && a.y == b.y;
  }
  friend auto operator!=(const Position<Number>& a, const Position<Number>& b) -> bool {
    return !(a == b);
  }
};

// Forward declarations of the edge types connected to a PositionNode.
template <class Number>
struct PositionLocationEdge;
template <class Number>
struct PositionDistanceEdge;
// Forward declaration of the node type referenced by the edge base aliases.
template <class Number>
struct PositionNode;

/// @brief Node base alias for a 2D position vertex.
///
/// Wires the CRTP @p Derived node to its fixed dimension (2), estimation type
/// (`Position<Number>`) and the set of edges it participates in.
///
/// @tparam Derived The concrete node type (CRTP).
/// @tparam Number  Scalar type of the estimation coordinates.
template <class Derived, class Number>
using PositionNodeBase =                                                    //< Node base alias
    node<Derived,                                                           //< Derived
         2,                                                                 //< Dimension
         Position<Number>,                                                  //< Type
         Edges<PositionLocationEdge<Number>, PositionDistanceEdge<Number>>  //< Edges
         >;

/// @brief A 2D position vertex (state dimension 2, estimation `Position`).
///
/// Participates in `PositionLocationEdge` (absolute position prior) and
/// `PositionDistanceEdge` (relative translation measurement) edges.
///
/// @tparam Number Scalar type of the estimation coordinates.
template <class Number>
struct PositionNode : PositionNodeBase<PositionNode<Number>, Number> {
  using Base = PositionNodeBase<PositionNode<Number>, Number>;
  using Base::Base;

  /// @brief Manifold retraction: adds a tangent-space increment to the current
  /// estimate.
  ///
  /// Templated on the increment scalar type so it works both for real-valued
  /// optimizer steps and for dual-number seeding (exact Jacobians via
  /// forward-mode automatic differentiation).
  ///
  /// @tparam Delta A vector-like increment whose element type may be `Number`
  ///   or a dual number.
  /// @param delta Increment vector of length 2: [dx, dy].
  /// @return A new `Position` with the updated coordinates.
  template <class Delta>
  auto plus(const Delta& delta) const {
    return Position{this->estimation().x + delta[0], this->estimation().y + delta[1]};
  }
};

/// @brief Edge base alias for a relative translation between two positions.
///
/// @tparam Derived The concrete edge type (CRTP).
/// @tparam Number  Scalar type of the measurement coordinates.
template <class Derived, class Number>
using PositionDistanceEdgeBase =                            //< Edge base alias
    edge<Derived,                                           //< Derived
         2,                                                 //< Dimension
         Position<Number>,                                  //< Type
         Nodes<PositionNode<Number>, PositionNode<Number>>  //< Nodes
         >;

/// @brief Relative translation measurement between two `PositionNode` nodes.
///
/// @tparam Number Scalar type of the measurement coordinates.
template <class Number>
struct PositionDistanceEdge : PositionDistanceEdgeBase<PositionDistanceEdge<Number>, Number> {
  using Base = PositionDistanceEdgeBase<PositionDistanceEdge<Number>, Number>;
  using Base::Base;
  // template <class T>
  // using Error = typename Base::template Error<T>;
  // /// @brief Computes the residual between the measured and observed relative
  /// translation.
  ///
  /// @tparam T Scalar type (`Number` for residual, or a dual number for
  ///   Jacobians).
  /// @param a Estimate of the first (source) position.
  /// @param b Estimate of the second (target) position.
  /// @return Error vector [observed_dx − measured_x, observed_dy − measured_y].
  template <class T>
  auto error(const Position<T>& a, const Position<T>& b) -> typename Base::template error_vector_type<T> {
    return {(b.x - a.x) - this->measurement().x, (b.y - a.y) - this->measurement().y};
  }
};

/// @brief Edge base alias for an absolute position prior on a single node.
///
/// @tparam Derived The concrete edge type (CRTP).
/// @tparam Number  Scalar type of the measurement coordinates.
template <class Derived, class Number>
using PositionLocationEdgeBase =      //< Edge base alias
    edge<Derived,                     //< Derived
         2,                           //< Dimension
         Position<Number>,            //< Type
         Nodes<PositionNode<Number>>  //< Nodes
         >;

/// @brief Absolute position (prior) measurement of a single `PositionNode`.
///
/// @tparam Number Scalar type of the measurement coordinates.
template <class Number>
struct PositionLocationEdge : PositionLocationEdgeBase<PositionLocationEdge<Number>, Number> {
  using Base = PositionLocationEdgeBase<PositionLocationEdge<Number>, Number>;
  using Base::Base;

  /// @brief Computes the residual between the estimated and measured absolute
  /// position.
  ///
  /// @tparam T Scalar type (`Number` for residual, or a dual number for
  ///   Jacobians).
  /// @param a Estimate of the position.
  /// @return Error vector [a.x − measured_x, a.y − measured_y].
  template <class T>
  auto error(const Position<T>& a) -> typename Base::template error_vector_type<T> {
    return {a.x - this->measurement().x, a.y - this->measurement().y};
  }
};

}  // namespace vortex::optimization::types

#endif  // VORTEX_OPTIMIZATION_TYPES_POSITION_HPP
