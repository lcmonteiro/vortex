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
struct position {
  /// @brief coordinates of the position.
  Number x{};
  Number y{};

  /// @brief friend operators for arithmetic and comparison of position objects.
  friend auto operator+(const position<Number>& a, const position<Number>& b) -> position<Number> {
    return {a.x + b.x, a.y + b.y};
  }
  friend auto operator-(const position<Number>& a, const position<Number>& b) -> position<Number> {
    return {a.x - b.x, a.y - b.y};
  }
  friend auto operator==(const position<Number>& a, const position<Number>& b) -> bool {
    return a.x == b.x && a.y == b.y;
  }
  friend auto operator!=(const position<Number>& a, const position<Number>& b) -> bool {
    return !(a == b);
  }
};

// Forward declarations of the edge types connected to a position_node.
template <class Number>
struct position_location_edge;
template <class Number>
struct position_distance_edge;
// Forward declaration of the node type referenced by the edge base aliases.
template <class Number>
struct position_node;

/// @brief Node base alias for a 2D position vertex.
///
/// Wires the CRTP @p Derived node to its fixed dimension (2), estimation type
/// (`position<Number>`) and the set of edges it participates in.
///
/// @tparam Derived The concrete node type (CRTP).
/// @tparam Number  Scalar type of the estimation coordinates.
template <class Derived, class Number>
using position_node_base =                                                  //< Node base alias
    node<Derived,                                                           //< Derived
         2,                                                                 //< Dimension
         position<Number>,                                                  //< Type
         Edges<position_location_edge<Number>, position_distance_edge<Number>>  //< Edges
         >;

/// @brief A 2D position vertex (state dimension 2, estimation `position`).
///
/// Participates in `position_location_edge` (absolute position prior) and
/// `position_distance_edge` (relative translation measurement) edges.
///
/// @tparam Number Scalar type of the estimation coordinates.
template <class Number>
struct position_node : position_node_base<position_node<Number>, Number> {
  using Base = position_node_base<position_node<Number>, Number>;
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
  /// @return A new `position` with the updated coordinates.
  template <class Delta>
  auto plus(const Delta& delta) const {
    return position{this->estimation().x + delta[0], this->estimation().y + delta[1]};
  }
};

/// @brief Edge base alias for a relative translation between two positions.
///
/// @tparam Derived The concrete edge type (CRTP).
/// @tparam Number  Scalar type of the measurement coordinates.
template <class Derived, class Number>
using position_distance_edge_base =                          //< Edge base alias
    edge<Derived,                                           //< Derived
         2,                                                 //< Dimension
         position<Number>,                                  //< Type
         Nodes<position_node<Number>, position_node<Number>>  //< Nodes
         >;

/// @brief Relative translation measurement between two `position_node` nodes.
///
/// @tparam Number Scalar type of the measurement coordinates.
template <class Number>
struct position_distance_edge : position_distance_edge_base<position_distance_edge<Number>, Number> {
  using Base = position_distance_edge_base<position_distance_edge<Number>, Number>;
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
  auto error(const position<T>& a, const position<T>& b) -> typename Base::template error_vector_type<T> {
    return {(b.x - a.x) - this->measurement().x, (b.y - a.y) - this->measurement().y};
  }
};

/// @brief Edge base alias for an absolute position prior on a single node.
///
/// @tparam Derived The concrete edge type (CRTP).
/// @tparam Number  Scalar type of the measurement coordinates.
template <class Derived, class Number>
using position_location_edge_base =      //< Edge base alias
    edge<Derived,                     //< Derived
         2,                           //< Dimension
         position<Number>,            //< Type
         Nodes<position_node<Number>>  //< Nodes
         >;

/// @brief Absolute position (prior) measurement of a single `position_node`.
///
/// @tparam Number Scalar type of the measurement coordinates.
template <class Number>
struct position_location_edge : position_location_edge_base<position_location_edge<Number>, Number> {
  using Base = position_location_edge_base<position_location_edge<Number>, Number>;
  using Base::Base;

  /// @brief Computes the residual between the estimated and measured absolute
  /// position.
  ///
  /// @tparam T Scalar type (`Number` for residual, or a dual number for
  ///   Jacobians).
  /// @param a Estimate of the position.
  /// @return Error vector [a.x − measured_x, a.y − measured_y].
  template <class T>
  auto error(const position<T>& a) -> typename Base::template error_vector_type<T> {
    return {a.x - this->measurement().x, a.y - this->measurement().y};
  }
};

}  // namespace vortex::optimization::types

#endif  // VORTEX_OPTIMIZATION_TYPES_POSITION_HPP
