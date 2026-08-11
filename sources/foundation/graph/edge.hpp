/// ===============================================================================================
/// @file
///
/// @brief vortex.graph.edge component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_GRAPH_GRAPH_EDGE_HPP
#define VORTEX_FOUNDATION_GRAPH_GRAPH_EDGE_HPP

#include <cstddef>
#include <tuple>

#include "helpers/apply.hpp"
#include "helpers/handle.hpp"
#include "helpers/types.hpp"

namespace vortex::graph {

/// ===============================================================================================
/// @brief A graph edge element that connects multiple nodes.
///
/// The Edge class connects nodes of various types in a graph. Each node is
/// stored as a shared reference, allowing for efficient memory management.
/// Functions can be applied to all connected nodes or to specific nodes by type
/// or index. The edge can also be enabled or disabled.
///
/// @tparam Nodes Variadic template representing the types of nodes connected by
/// this edge.
/// ===============================================================================================
template <class... Nodes>
class edge {
 public:
  /// @brief The number of node types connected by this edge.
  static constexpr auto n_nodes = sizeof...(Nodes);

  /// @brief Constructs an edge connecting the specified nodes.
  /// @note Move and copy constructors purposely omitted
  explicit edge(const helpers::handle<Nodes>&... nodes) : disable_{}, nodes_{nodes...} {}

  /// @brief Gets a node by type.
  /// @tparam T Node type.
  /// @return The node handle.
  template <class T>
  const auto& node() const {
    return std::get<helpers::handle<T>>(nodes_);
  }

  /// @brief Gets a node by index.
  /// @tparam I Index.
  /// @return The node handle.
  template <std::size_t I>
  const auto& node() const {
    return std::get<I>(nodes_);
  }

  /// @brief Applies a function to all nodes.
  /// @tparam F Function type.
  /// @param func Function reference.
  template <class F>
  auto apply(F&& func) -> void {
    helpers::apply(std::forward<F>(func), nodes_);
  }

  template <class F>
  auto apply(F&& func) const -> void {
    helpers::apply(std::forward<F>(func), nodes_);
  }

  template <class T, class F>
  auto apply(F&& func) -> void {
    std::forward<F>(func)(node<T>(nodes_));
  }

  template <class T, class F>
  auto apply(F&& func) const -> void {
    std::forward<F>(func)(node<T>(nodes_));
  }

  /// @brief Checks if the edge is disable.
  /// @return `true` if disable otherwise `false`.
  auto disable() const -> bool { return disable_; }

 protected:
  template <class, class, class>
  friend class storage;

  /// @brief Disables or enables the edge.
  /// @param value If `true`, the edge will be disabled;
  /// if `false`, it will be enabled.
  auto disable(bool value) -> void { disable_ = value; }

 private:
  bool disable_{};
  std::tuple<helpers::handle<Nodes>...> nodes_;
};

/// ===============================================================================================
/// @brief Concept satisfied by any specialization of `edge`.
/// ===============================================================================================
template <class T>
concept edge_type = requires { []<class... Ts>(const edge<Ts...>&) {}(std::declval<T>()); };

}  // namespace vortex::graph
#endif  // VORTEX_FOUNDATION_GRAPH_GRAPH_EDGE_HPP
