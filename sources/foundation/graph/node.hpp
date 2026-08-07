/// ===============================================================================================
/// @file
///
/// @brief vortex.graph.node component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_GRAPH_GRAPH_NODE_HPP
#define VORTEX_FOUNDATION_GRAPH_GRAPH_NODE_HPP
#include <memory>
#include <memory_resource>
#include <set>
#include <tuple>

#include "helpers/apply.hpp"
#include "helpers/build.hpp"
#include "helpers/handle.hpp"
namespace vortex::graph {
/// ===============================================================================================
/// @brief A graph node that can manage edges of different types.
///
/// The Node class stores edges of various types in sets, one set per edge type.
/// It supports linking, unlinking, and applying functions to all edges or edges
/// of a specific type. Nodes can be enabled/disabled.
///
/// @tparam Edges Variadic template representing the different edge types the
/// node can manage.
/// ===============================================================================================
template <class... Edges>
class Node {
  template <class T>
  using SetShared = std::pmr::set<helpers::handle<T>>;
  using TupleSetShared = std::tuple<SetShared<Edges>...>;

 public:
  static constexpr auto N_TYPES = sizeof...(Edges);

  template <class Key>
  Node(const Key&, std::pmr::memory_resource* const memory)
      : disable_{}, edges_{helpers::build<TupleSetShared>(memory)} {}

  /// @brief Applies a function to all edges.
  /// @tparam F Function type.
  /// @param func Function reference.
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
  friend class Container;

  /// @brief Disables or enables the node.
  /// @param value If `true`, the node will be disabled; if `false`, it will
  /// be enabled.
  auto disable(bool value) -> void { disable_ = value; }

  /// @brief Links and unlinks edges.
  /// @tparam T Edge type.
  /// @param edge The edge to link.
  template <class T>
  auto link(const helpers::handle<T>& edge) -> void {
    std::ignore = std::get<SetShared<T>>(edges_).insert(edge);
  }

  template <class T>
  auto unlink(const helpers::handle<T>& edge) -> void {
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

/// ===============================================================================================
/// @brief Concept satisfied by any specialization of `Node`.
/// ===============================================================================================
template <class T>
concept node_type = requires { []<class... Ts>(const Node<Ts...>&) {}(std::declval<T>()); };

}  // namespace vortex::graph
#endif  // VORTEX_FOUNDATION_GRAPH_GRAPH_NODE_HPP
