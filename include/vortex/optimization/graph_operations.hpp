/// ===============================================================================================
/// @file
///
/// @brief vortex.graph.operations component
/// ===============================================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_OPERATIONS_HPP
#define VORTEX_OPTIMIZATION_GRAPH_OPERATIONS_HPP

#include <cstddef>
#include <utility>

namespace vortex::optimization {

/// @brief Alternative operations API for interacting with the Graph class.
/// These free functions helper provide a simplified and convenient way to
/// interact with the graph by abstracting template complexities. They offer an
/// alternative to the member functions of the Graph class, allowing for easier
/// iteration, search, and manipulation of nodes and edges in a more generic
/// context.
///
/// - `for_each`: Iterates through all specified types in a graph, applying a
/// function.
/// - `find_if`: Searches for the first element of a specified type that matches
/// a condition.
/// - `remove_if`: Removes elements from the graph that match a given predicate.
/// - `enable`: Enables elements from the graph that match a specified type.
/// - `enable_if`: Enables elements from the graph that match a specified type
/// and condition.
/// - `disable`: Disables elements from the graph that match a specified type.
/// - `disable_if`: Disables elements from the graph that match a specified type
/// and condition.
/// - `get_node`: Retrieves a node by its index from a specified type.

template <class Types, class Graph, class Fn>
inline auto for_each(Graph& graph, Fn&& func) -> void {
  graph.template apply<Types>(std::forward<Fn>(func));
}

template <class Types, class Graph, class Fn, class Option>
inline auto for_each(Graph& graph, Fn&& func, const Option value) -> void {
  graph.template apply<Types>(std::forward<Fn>(func), value);
}

template <class Types, class Graph, class Fn>
inline auto find_if(Graph& graph, Fn&& func) -> decltype(auto) {
  return graph.template find<Types>(std::forward<Fn>(func));
}

template <class Types, class Graph, class Fn>
inline auto find_if(const Graph& graph, Fn&& func) -> decltype(auto) {
  return graph.template find<Types>(std::forward<Fn>(func));
}

template <class Types, class Graph, class Fn>
inline auto remove_if(Graph& graph, Fn&& func) -> void {
  graph.template destroy<Types>(std::forward<Fn>(func));
}

template <class Types, class Graph>
inline auto enable(Graph& graph) -> void {
  graph.template toggle<Types>(typename Graph::disabled_type{});
}

template <class Types, class Graph, class Key>
inline auto enable(Graph& graph, const Key& key) -> void {
  graph.template toggle<Types>(key, typename Graph::disabled_type{});
}

template <class Types, class Graph>
inline auto disable(Graph& graph) -> void {
  graph.template toggle<Types>(typename Graph::enabled_type{});
}

template <class Types, class Graph, class Key>
inline auto disable(Graph& graph, const Key& key) -> void {
  graph.template toggle<Types>(key, typename Graph::enabled_type{});
}

template <class Types, class Graph, class Fn>
inline auto enable_if(Graph& graph, Fn&& func) -> void {
  graph.template toggle<Types>(std::forward<Fn>(func), typename Graph::disabled_type{});
}

template <class Types, class Graph, class Fn>
inline auto disable_if(Graph& graph, Fn&& func) -> void {
  graph.template toggle<Types>(std::forward<Fn>(func), typename Graph::enabled_type{});
}

template <std::size_t I, class Type>
inline auto get_node(Type& obj) -> decltype(auto) {
  return obj.template node<I>();
}

}  // namespace vortex::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_OPERATIONS_HPP
