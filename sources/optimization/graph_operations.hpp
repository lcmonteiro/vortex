/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
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
inline void ForEach(Graph& graph, Fn&& func) {
  graph.template apply<Types>(std::forward<Fn>(func));
}

template <class Types, class Graph, class Fn, class Option>
inline void ForEach(Graph& graph, Fn&& func, const Option value) {
  graph.template apply<Types>(std::forward<Fn>(func), value);
}

template <class Types, class Graph, class Fn>
inline void ForEach(const Graph& graph, Fn&& func) {
  graph.template apply<Types>(std::forward<Fn>(func));
}

template <class Types, class Graph, class Fn, class Option>
inline void ForEach(const Graph& graph, Fn&& func, const Option value) {
  graph.template apply<Types>(std::forward<Fn>(func), value);
}

template <class Types, class Graph, class Fn>
inline decltype(auto) FindIf(Graph& graph, Fn&& func) {
  return graph.template find<Types>(std::forward<Fn>(func));
}

template <class Types, class Graph, class Fn>
inline decltype(auto) FindIf(const Graph& graph, Fn&& func) {
  return graph.template find<Types>(std::forward<Fn>(func));
}

template <class Types, class Graph, class Fn>
inline void RemoveIf(Graph& graph, Fn&& func) {
  graph.template destroy<Types>(std::forward<Fn>(func));
}

template <class Types, class Graph>
inline void Enable(Graph& graph) {
  graph.template toggle<Types>(typename Graph::Disabled{});
}

template <class Types, class Graph, class Key>
inline void Enable(Graph& graph, const Key& key) {
  graph.template toggle<Types>(key, typename Graph::Disabled{});
}

template <class Types, class Graph>
inline void Disable(Graph& graph) {
  graph.template toggle<Types>(typename Graph::Enabled{});
}

template <class Types, class Graph, class Key>
inline void Disable(Graph& graph, const Key& key) {
  graph.template toggle<Types>(key, typename Graph::Enabled{});
}

template <class Types, class Graph, class Fn>
inline void EnableIf(Graph& graph, Fn&& func) {
  graph.template toggle<Types>(std::forward<Fn>(func), typename Graph::Disabled{});
}

template <class Types, class Graph, class Fn>
inline void DisableIf(Graph& graph, Fn&& func) {
  graph.template toggle<Types>(std::forward<Fn>(func), typename Graph::Enabled{});
}

template <size_t I, class Type>
inline decltype(auto) GetNode(Type& obj) {
  return obj.template node<I>();
}

}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_OPERATIONS_HPP
