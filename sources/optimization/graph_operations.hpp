/// ===========================================================================
/// @file
///
/// @brief vortex.graph.operations component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_OPERATIONS_HPP
#define VORTEX_OPTIMIZATION_GRAPH_OPERATIONS_HPP

#include <cstddef>
#include <utility>

namespace vortex::graph::optimization {

/// @brief Alternative operations API for interacting with the Graph class.
/// These free functions helper provide a simplified and convenient way to
/// interact with the graph by abstracting template complexities. They offer an
/// alternative to the member functions of the Graph class, allowing for easier
/// iteration, search, and manipulation of nodes and edges in a more generic
/// context.
///
/// - `ForEach`: Iterates through all specified types in a graph, applying a
/// function.
/// - `FindIf`: Searches for the first element of a specified type that matches
/// a condition.
/// - `RemoveIf`: Removes elements from the graph that match a given predicate.
/// - `Enable`: Enables elements from the graph that match a specified type.
/// - `EnableIf`: Enables elements from the graph that match a specified type
/// and condition.
/// - `Disable`: Disables elements from the graph that match a specified type.
/// - `DisableIf`: Disables elements from the graph that match a specified type
/// and condition.
/// - `GetNode`: Retrieves a node by its index from a specified type.

template <class Types, class Graph, class Fn>
inline auto ForEach(Graph& graph, Fn&& func) -> void {
  graph.template apply<Types>(std::forward<Fn>(func));
}

template <class Types, class Graph, class Fn, class Option>
inline auto ForEach(Graph& graph, Fn&& func, const Option value) -> void {
  graph.template apply<Types>(std::forward<Fn>(func), value);
}

template <class Types, class Graph, class Fn>
inline auto ForEach(const Graph& graph, Fn&& func) -> void {
  graph.template apply<Types>(std::forward<Fn>(func));
}

template <class Types, class Graph, class Fn, class Option>
inline auto ForEach(const Graph& graph, Fn&& func, const Option value) -> void {
  graph.template apply<Types>(std::forward<Fn>(func), value);
}

template <class Types, class Graph, class Fn>
inline auto FindIf(Graph& graph, Fn&& func) -> decltype(auto) {
  return graph.template find<Types>(std::forward<Fn>(func));
}

template <class Types, class Graph, class Fn>
inline auto FindIf(const Graph& graph, Fn&& func) -> decltype(auto) {
  return graph.template find<Types>(std::forward<Fn>(func));
}

template <class Types, class Graph, class Fn>
inline auto RemoveIf(Graph& graph, Fn&& func) -> void {
  graph.template destroy<Types>(std::forward<Fn>(func));
}

template <class Types, class Graph>
inline auto Enable(Graph& graph) -> void {
  graph.template toggle<Types>(typename Graph::Disabled{});
}

template <class Types, class Graph, class Key>
inline auto Enable(Graph& graph, const Key& key) -> void {
  graph.template toggle<Types>(key, typename Graph::Disabled{});
}

template <class Types, class Graph>
inline auto Disable(Graph& graph) -> void {
  graph.template toggle<Types>(typename Graph::Enabled{});
}

template <class Types, class Graph, class Key>
inline auto Disable(Graph& graph, const Key& key) -> void {
  graph.template toggle<Types>(key, typename Graph::Enabled{});
}

template <class Types, class Graph, class Fn>
inline auto EnableIf(Graph& graph, Fn&& func) -> void {
  graph.template toggle<Types>(std::forward<Fn>(func), typename Graph::Disabled{});
}

template <class Types, class Graph, class Fn>
inline auto DisableIf(Graph& graph, Fn&& func) -> void {
  graph.template toggle<Types>(std::forward<Fn>(func), typename Graph::Enabled{});
}

template <std::size_t I, class Type>
inline auto GetNode(Type& obj) -> decltype(auto) {
  return obj.template node<I>();
}

}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_OPERATIONS_HPP
