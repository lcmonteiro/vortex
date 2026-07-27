/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief g2o_dual.graph_engine.node component
/// ===========================================================================
#ifndef G2O_DUAL_BASE_GRAPH_NODE_HPP
#define G2O_DUAL_BASE_GRAPH_NODE_HPP
#include <memory>
#include <memory_resource>
#include <set>
#include <tuple>

#include "sources/helpers/apply.hpp"
#include "sources/helpers/build.hpp"
#include "sources/helpers/shared.hpp"
namespace graph {
/// ===========================================================================
/// @class Node
/// @brief A graph node that can manage edges of different types.
/// @tparam Edges... - Variadic template representing different edge types the
/// node can manage. The Node class stores edges of various types in sets, one
/// set per edge type. It supports linking, unlinking, and applying functions to
/// all edges or edges of a specific type. Nodes can be enabled/disabled.
/// ===========================================================================
template <class... Edges>
class Node {
  template <class T>
  using SetShared = std::pmr::set<detail::Shared<T>>;
  using TupleSetShared = std::tuple<SetShared<Edges>...>;

 public:
  static constexpr auto N_TYPES = sizeof...(Edges);

  template <class Key>
  Node(const Key&, std::pmr::memory_resource* const memory)
      : disable_{}, edges_{detail::build<TupleSetShared>(memory)} {}

  /// @brief Apply a function to all edges
  /// @tparam F - function type
  /// @param func - function reference
  template <class F>
  void apply(F&& func) {
    detail::apply([&](auto& data) { detail::apply(std::forward<F>(func), data); }, edges_);
  }

  template <class F>
  void apply(F&& func) const {
    detail::apply([&](auto& data) { detail::apply(std::forward<F>(func), data); }, edges_);
  }

  template <class T, class F>
  void apply(F&& func) {
    detail::apply(std::forward<F>(func), std::get<SetShared<T>>(edges_));
  }

  template <class T, class F>
  void apply(F&& func) const {
    detail::apply(std::forward<F>(func), std::get<SetShared<T>>(edges_));
  }

  /// @brief Checks if the node is disable.
  /// @return `true` if disable otherwise `false`.
  bool disable() const { return disable_; }

 protected:
  template <class, class, class>
  friend class Graph;

  /// @brief Disables or enables the node.
  /// @param value If `true`, the node will be disabled; if `false`, it will
  /// be enabled.
  void disable(bool value) { disable_ = value; }

  /// @brief Link and unlink edges
  /// @tparam T - edge type
  /// @param edge
  template <class T>
  void link(const detail::Shared<T>& edge) {
    std::ignore = std::get<SetShared<T>>(edges_).insert(edge);
  }

  template <class T>
  void unlink(const detail::Shared<T>& edge) {
    std::ignore = std::get<SetShared<T>>(edges_).erase(edge);
  }
  template <class T>
  void unlink() {
    std::get<SetShared<T>>(edges_).clear();
  }
  void unlink() {
    detail::apply([&](auto& set) { set.clear(); }, edges_);
  }

 private:
  bool disable_;
  std::tuple<SetShared<Edges>...> edges_;
};

/// ===========================================================================
/// Indentity Helper Function
/// ===========================================================================
// LCOV_EXCL_START
template <class... E>
inline constexpr auto ToNode(const Node<E...>& v) {
  return v;
}
// LCOV_EXCL_STOP

}  // namespace graph
#endif  // G2O_DUAL_BASE_GRAPH_NODE_HPP
