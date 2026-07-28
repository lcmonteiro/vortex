/// ===========================================================================
/// @file
///
/// @brief vortex.graph_engine.node component
/// ===========================================================================
#ifndef VORTEX_BASE_GRAPH_NODE_HPP
#define VORTEX_BASE_GRAPH_NODE_HPP
#include <memory>
#include <memory_resource>
#include <set>
#include <tuple>

#include "helpers/apply.hpp"
#include "helpers/build.hpp"
#include "helpers/shared.hpp"
namespace vortex::graph {
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
  using SetShared = std::pmr::set<helpers::Shared<T>>;
  using TupleSetShared = std::tuple<SetShared<Edges>...>;

 public:
  static constexpr auto N_TYPES = sizeof...(Edges);

  template <class Key>
  Node(const Key&, std::pmr::memory_resource* const memory)
      : disable_{}, edges_{helpers::build<TupleSetShared>(memory)} {}

  /// @brief Apply a function to all edges
  /// @tparam F - function type
  /// @param func - function reference
  template <class F>
  auto apply(F&& func) -> void {
    helpers::apply([&](auto& data) { helpers::apply(std::forward<F>(func), data); }, edges_);
  }

  template <class F>
  auto apply(F&& func) const -> void {
    helpers::apply([&](auto& data) { helpers::apply(std::forward<F>(func), data); }, edges_);
  }

  template <class T, class F>
  auto apply(F&& func) -> void {
    helpers::apply(std::forward<F>(func), std::get<SetShared<T>>(edges_));
  }

  template <class T, class F>
  auto apply(F&& func) const -> void {
    helpers::apply(std::forward<F>(func), std::get<SetShared<T>>(edges_));
  }

  /// @brief Checks if the node is disable.
  /// @return `true` if disable otherwise `false`.
  auto disable() const -> bool { return disable_; }

 protected:
  template <class, class, class>
  friend class Graph;

  /// @brief Disables or enables the node.
  /// @param value If `true`, the node will be disabled; if `false`, it will
  /// be enabled.
  auto disable(bool value) -> void { disable_ = value; }

  /// @brief Link and unlink edges
  /// @tparam T - edge type
  /// @param edge
  template <class T>
  auto link(const helpers::Shared<T>& edge) -> void {
    std::ignore = std::get<SetShared<T>>(edges_).insert(edge);
  }

  template <class T>
  auto unlink(const helpers::Shared<T>& edge) -> void {
    std::ignore = std::get<SetShared<T>>(edges_).erase(edge);
  }
  template <class T>
  auto unlink() -> void {
    std::get<SetShared<T>>(edges_).clear();
  }
  auto unlink() -> void {
    helpers::apply([&](auto& set) { set.clear(); }, edges_);
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

}  // namespace vortex::graph
#endif  // VORTEX_BASE_GRAPH_NODE_HPP
