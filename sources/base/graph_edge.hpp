/// ===========================================================================
/// @file
///
/// @brief vortex.graph_engine.edge component
/// ===========================================================================
#ifndef VORTEX_BASE_GRAPH_EDGE_HPP
#define VORTEX_BASE_GRAPH_EDGE_HPP

#include <memory>
#include <memory_resource>
#include <tuple>

#include "helpers/apply.hpp"
#include "helpers/shared.hpp"
namespace vortex::graph {

/// ===========================================================================
/// @class Edge
/// @brief A graph edge element that connects multiple nodes.
/// @tparam Nodes... - Variadic template representing the types of nodes
/// connected by this edge.
///
/// The Edge class connects nodes of various types in a graph. Each node is
/// stored as a shared reference, allowing for efficient memory management.
/// Functions can be applied to all connected nodes or to specific nodes by type
/// or index. The edge can also be enabled or disabled.
/// ===========================================================================
template <class... Nodes>
class Edge {
 public:
  static constexpr auto NNodes = sizeof...(Nodes);

  explicit Edge(const helpers::Shared<Nodes>&... nodes) : disable_{}, nodes_{nodes...} {}
  // Move and copy constructors purposely omitted

  /// @brief Get node by type
  /// @tparam T - node type
  /// @return shared node
  template <class T>
  const auto& node() const {
    return std::get<helpers::Shared<T>>(nodes_);
  }

  /// @brief Get node by index
  /// @tparam I - index
  /// @return shared node
  template <size_t I>
  const auto& node() const {
    return std::get<I>(nodes_);
  }

  /// @brief Apply a function to all nodes
  /// @tparam F - function type
  /// @param func - function reference
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
  friend class Graph;

  /// @brief Disables or enables the edge.
  /// @param value If `true`, the edge will be disabled;
  /// if `false`, it will be enabled.
  auto disable(bool value) -> void { disable_ = value; }

 private:
  bool disable_{};
  std::tuple<helpers::Shared<Nodes>...> nodes_;
};

/// ===========================================================================
/// Indentity Helper Function
/// ===========================================================================
// LCOV_EXCL_START
template <class... N>
inline constexpr auto ToEdge(const Edge<N...>& e) {
  return e;
}
// LCOV_EXCL_STOP
}  // namespace vortex::graph
#endif  // VORTEX_BASE_GRAPH_EDGE_HPP
