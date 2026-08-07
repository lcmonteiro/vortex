/// ===============================================================================================
/// @file
///
/// @brief vortex.graph component
/// ===============================================================================================
#ifndef VORTEX_FOUNDATION_GRAPH_GRAPH_CONTAINER_HPP
#define VORTEX_FOUNDATION_GRAPH_GRAPH_CONTAINER_HPP

#include <cstddef>
#include <memory_resource>
#include <optional>
#include <tuple>

#include "foundation/graph/config.hpp"
#include "foundation/graph/edge.hpp"
#include "foundation/graph/node.hpp"
#include "helpers/apply.hpp"
#include "helpers/build.hpp"
#include "helpers/handle.hpp"
#include "helpers/memory.hpp"
#include "helpers/move.hpp"
#include "helpers/revision.hpp"
#include "helpers/select.hpp"
#include "helpers/types.hpp"
#include "helpers/utility.hpp"

namespace vortex::graph {
/// ===============================================================================================
/// Helper Types
/// ===============================================================================================
using helpers::handle;
using helpers::optional;
using helpers::revision;
using helpers::type_list;
using helpers::types;

using SharedRevision = handle<helpers::revision>;

/// ===============================================================================================
/// @brief Represents a graph with customizable node and edge types, and
/// flexible configuration options.
///
/// This class provides functionalities to build, manipulate, and query nodes
/// and edges in a graph. Nodes and edges can be enabled, disabled, or toggled
/// between both states. It also supports various operations like size queries,
/// applying functions to nodes/edges, and managing graph revision states.
///
/// @tparam Tnodes The set of node types used in the graph.
/// @tparam Tedges The set of edge types used in the graph.
/// @tparam Config The configuration for memory resource management (default is
/// DefaultConfig).
/// ===============================================================================================
template <class Tnodes, class Tedges, class Config = DefaultConfig>
class Container {
 protected:
  // types
  using Key = typename Config::Key;

  template <class K, class V>
  using Map = typename Config::template Map<K, V>;
  template <class V>
  using Set = typename Config::template Set<V>;

  template <class T>
  using MapShared = Map<Key, handle<T>>;
  template <class T>
  using SetShared = Set<handle<T>>;

  using TupleMapShared = helpers::types_wrap_build_t<std::tuple, MapShared, Tnodes>;
  using TupleSetShared = helpers::types_wrap_build_t<std::tuple, SetShared, Tedges>;

  template <helpers::selection value>
  using SelectionConstant = helpers::selection_constant<value>;
  using Selection = helpers::selection;

 public:
  using Nodes = Tnodes;
  using Edges = Tedges;

  using All = SelectionConstant<Selection::kAll>;
  using Enabled = SelectionConstant<Selection::kLeft>;
  using Disabled = SelectionConstant<Selection::kRight>;
  static constexpr auto kAll = All{};
  static constexpr auto kEnabled = Enabled{};
  static constexpr auto kDisabled = Disabled{};

  /// @brief Container main constructor.
  /// @param memory_resource
  Container(const SharedRevision& revision, std::pmr::memory_resource* const memory_resource)
      : memory_monotonic_{Config::CacheInitSize, memory_resource},
        memory_pool_{{0, Config::CacheBlockMaxSize}, &memory_monotonic_},
        memory_bounded_{&memory_pool_},
        memory_{&memory_bounded_},
        nodes_enable_{helpers::build<TupleMapShared>(memory_resource)},
        nodes_disable_{helpers::build<TupleMapShared>(memory_resource)},
        edges_enable_{helpers::build<TupleSetShared>(memory_resource)},
        edges_disable_{helpers::build<TupleSetShared>(memory_resource)},
        revision_{revision} {}

  explicit Container(std::pmr::memory_resource* const memory_resource)
      : Container(SharedRevision{memory_resource}, memory_resource) {}

  Container(const Container&) = delete;
  Container(Container&&) = delete;
  auto operator=(const Container&) -> Container& = delete;
  auto operator=(Container&&) -> Container& = delete;

  /// @brief Container destructor.
  ~Container() { destroy(); }

  /// @defgroup BuildFunctions Build Functions
  /// @brief Functions to create and initialize nodes or edges in the graph.
  /// These functions manage memory allocation and ensure that newly created
  /// nodes or edges are linked correctly within the graph structure.
  /// The functions also handle the destruction of existing nodes with the same
  /// key to prevent duplicates.
  /// @param T The type of the node or edge being created.
  /// @param A The types of the arguments passed to the constructor of the node
  /// or edge.
  /// @param key The key used to identify the node being built (for node
  /// construction).
  /// @param args The arguments used to initialize the node or edge.
  template <node_type T, class... A>
  auto build(const Key& key, A&&... args) {
    auto node = handle<T>(memory_, std::forward<A>(args)..., key, memory_);
    destroy<T>(key);
    std::ignore = get<T>(nodes_enable_).emplace(key, node);
    revision_->update();
    return node;
  }

  template <edge_type T, class... A>
  auto build(A&&... args) {
    auto edge = handle<T>(memory_, std::forward<A>(args)...);
    edge->apply([&](const auto& v) { v->link(edge); });
    link_edge(edges_enable_, edge);
    revision_->update();
    return edge;
  }

  /// @defgroup SizeFunctions Size Functions
  /// @brief Functions to calculate the sizes of nodes, edges, and types in the
  /// graph.
  /// @param T The type of node or edge to count.
  /// @param value The side value that determines the counting behavior.
  /// This can be one of the enumerated options (enabled, disabled, all).
  /// @param side The `SelectionConstant` that specifies whether to count
  /// enabled, disabled, or all nodes and edges.
  template <node_type T, Selection value = Selection::kAll>
  auto size(const SelectionConstant<value> side = All{}) const {
    std::size_t result{0};
    for (const MapShared<T>& map : select<T>(nodes_enable_, nodes_disable_, side)) {
      result += std::size(map);
    }
    return result;
  }

  template <edge_type T, Selection value = Selection::kAll>
  auto size(const SelectionConstant<value> side = All{}) const {
    std::size_t result{0};
    for (const SetShared<T>& set : select<T>(edges_enable_, edges_disable_, side)) {
      result += std::size(set);
    }
    return result;
  }

  template <type_list T, Selection value = Selection::kAll>
  auto size(const SelectionConstant<value> side = All{}) const {
    return size(T{}, side);
  }

  template <Selection value = Selection::kAll>
  auto size(const SelectionConstant<value> side = All{}) const {
    return size<Nodes>(side) + size<Edges>(side);
  }

  /// @defgroup ApplyFunctions Apply Functions
  /// @brief Functions to apply a given function to nodes and edges in the
  /// graph.
  /// These functions allow users to apply a specified function to all nodes or
  /// edges of a particular type within the graph. The function can be applied
  /// to all enabled and disabled nodes or edges, or selectively based on
  /// specific options.
  /// @param T The type of the element to which the function will be
  /// applied.
  /// @param Fn The type of the function to apply to each element.
  /// @param value The side constant that determines the scope of the
  /// operation.
  /// @param side The constant indicating the selection criteria.
  template <node_type T, class Fn, Selection value = Selection::kAll>
  requires std::is_invocable_v<Fn, handle<T>&>
  auto apply(Fn&& func, const SelectionConstant<value> side = All{}) const -> void {
    auto& ref_func = helpers::lreference<Fn>(std::forward<Fn>(func));
    for (const MapShared<T>& map : select<T>(nodes_enable_, nodes_disable_, side)) {
      helpers::apply(ref_func, map);
    }
  }

  template <edge_type T, class Fn, Selection value = Selection::kAll>
  requires std::is_invocable_v<Fn, handle<T>&>
  auto apply(Fn&& func, const SelectionConstant<value> side = All{}) const -> void {
    auto& ref_func = helpers::lreference<Fn>(std::forward<Fn>(func));
    for (const SetShared<T>& set : select<T>(edges_enable_, edges_disable_, side)) {
      helpers::apply(ref_func, set);
    }
  }

  template <type_list T, class Fn, Selection value = Selection::kAll>
  auto apply(Fn&& func, const SelectionConstant<value> side = All{}) const -> void {
    apply(std::forward<Fn>(func), T{}, side);
  }

  template <class Fn, Selection value = Selection::kAll>
  auto apply(Fn&& func, const SelectionConstant<value> side = All{}) const -> void {
    auto& ref_func = helpers::lreference<Fn>(std::forward<Fn>(func));
    apply<Nodes>(ref_func, side);
    apply<Edges>(ref_func, side);
  }

  /// @defgroup FindFunctions Find Functions
  /// @brief Functions to find nodes in the graph.
  /// The search can be performed based on a specific key or by applying
  /// a predicate to determine the desired node.
  /// @param N The type of the node to search for.
  /// @param Fn The type of the predicate function used for searching.
  /// @param value The side constant that determines the scope of the search.
  /// @param key The key used to locate a specific node.
  /// @param evaluate The predicate function used to evaluate each node during
  /// the search.
  /// @param side The constant indicating the selection criteria for nodes.
  template <node_type N, Selection value = Selection::kAll>
  auto find(const Key& key, const SelectionConstant<value> side = All{}) const -> optional<N> {
    const auto& nodes{select<N>(nodes_enable_, nodes_disable_, side)};
    for (const MapShared<N>& map : nodes) {
      if (auto found = map.find(key); found != std::end(map)) {
        return std::get<handle<N>>(*found);
      }
    }
    return std::nullopt;
  }

  template <node_type N, class Fn, Selection value = Selection::kAll>
  requires std::is_invocable_v<Fn, handle<N>&>
  auto find(Fn&& evaluate, const SelectionConstant<value> side = All{}) const -> optional<N> {
    auto& ref_evaluate = helpers::lreference<Fn>(std::forward<Fn>(evaluate));
    const auto& nodes{select<N>(nodes_enable_, nodes_disable_, side)};
    for (const MapShared<N>& map : nodes) {
      for (auto& node_pair : map) {
        if (ref_evaluate(std::get<1>(node_pair))) {
          return std::get<1>(node_pair);
        }
      }
    }
    return std::nullopt;
  }

  /// @defgroup UnlinkFunctions Unlink Functions
  /// @brief Functions to unlink/delete edges from nodes.
  /// These functions provide utilities for unlinking nodes from the graph
  /// based on their keys or by iterating through all nodes of a specified type.
  /// @param N The type of the node to unlink.
  /// @param E The type of the edge to unlink (if applicable).
  /// @param value The side constant that determines the scope of the
  /// unlinking operation.
  /// @param key The key used to locate a specific node for unlinking.
  /// @param side The constant indicating the selection criteria for nodes.
  template <node_type N, Selection value = Selection::kAll>
  auto unlink(Key key, const SelectionConstant<value> side = All{}) -> void {
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, side)) {
      if (auto iter = map.find(key); iter != map.end()) {
        unlink_node(std::get<handle<N>>(*iter));
        revision_->update();
        return;
      }
    }
  }

  template <node_type N, edge_type E, Selection value = Selection::kAll>
  auto unlink(Key key, const SelectionConstant<value> side = All{}) -> void {
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, side)) {
      if (auto iter = map.find(key); iter != map.end()) {
        unlink_node<E>(std::get<handle<N>>(*iter));
        revision_->update();
        return;
      }
    }
  }

  template <node_type N, type_list E, Selection value = Selection::kAll>
  auto unlink(Key key, const SelectionConstant<value> side = All{}) -> void {
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, side)) {
      if (auto iter = map.find(key); iter != map.end()) {
        unlink_node(std::get<handle<N>>(*iter), E{});
        revision_->update();
        return;
      }
    }
  }

  template <node_type N, Selection value = Selection::kAll>
  auto unlink(const SelectionConstant<value> side = All{}) -> void {
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, side)) {
      for (const auto& node : map) {
        unlink_node(std::get<handle<N>>(node));
      }
    }
    revision_->update();
  }

  template <node_type N, edge_type E, Selection value = Selection::kAll>
  auto unlink(const SelectionConstant<value> side = All{}) -> void {
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, side)) {
      for (const auto& node : map) {
        unlink_node<E>(std::get<handle<N>>(node));
      }
    }
    revision_->update();
  }

  template <node_type N, type_list E, Selection value = Selection::kAll>
  auto unlink(const SelectionConstant<value> side = All{}) -> void {
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, side)) {
      for (const auto& node : map) {
        unlink_node(std::get<handle<N>>(node), E{});
      }
    }
    revision_->update();
  }

  /// @defgroup DestroyFunctions Destroy Functions
  /// @brief Functions to destroy nodes in the graph.
  /// These functions provide utilities to remove nodes from the graph based on
  /// their keys or by applying a predicate function.
  /// @param N The type of the node to be destroyed.
  /// @param Fn The type of the predicate function used for selective
  /// destruction.
  /// @param value The side constant that determines the scope of the
  /// destruction operation.
  /// @param key The key used to locate a specific node for destruction.
  /// @param func The predicate function used to evaluate each node during the
  /// destruction process.
  /// @param side The constant indicating the selection criteria for nodes.
  template <node_type N, Selection value = Selection::kAll>
  auto destroy(Key key, const SelectionConstant<value> side = All{}) -> void {
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, side)) {
      if (auto iter = map.find(key); iter != map.end()) {
        unlink_node(std::get<handle<N>>(*iter));
        std::ignore = map.erase(iter);
        revision_->update();
        return;
      }
    }
  }

  template <node_type N, Selection value = Selection::kAll>
  auto destroy(const SelectionConstant<value> side = All{}) -> void {
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, side)) {
      for (const auto& node : map) {
        unlink_node(std::get<handle<N>>(node));
      }
      map.clear();
    }
    revision_->update();
  }

  template <node_type N, class Fn, Selection value = Selection::kAll>
  requires std::is_invocable_v<Fn, handle<N>&>
  auto destroy(Fn&& func, const SelectionConstant<value> side = All{}) -> void {
    auto& ref_func = helpers::lreference<Fn>(std::forward<Fn>(func));
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, side)) {
      for (auto iter = map.begin(); iter != map.end();) {
        if (ref_func(std::get<handle<N>>(*iter))) {
          unlink_node(std::get<handle<N>>(*iter));
          iter = map.erase(iter);
        } else {
          ++iter;
        }
      }
    }
    revision_->update();
  }

  template <Selection value = Selection::kAll>
  auto destroy(const SelectionConstant<value> side = All{}) -> void {
    destroy(Nodes{}, side);
  }

  /// @defgroup ToggleFunctions Toggle Functions
  /// @brief Functions to toggle the state of nodes and edges in the graph.
  /// These functions provide utilities to enable or disable nodes and edges of
  /// specific types within the graph. The toggle operations can be performed
  /// on all nodes or edges, conditionally based on a given function, or on
  /// individual nodes or edges identified by their keys or references.
  /// @param T The type of the node or edge to be toggled.
  /// @param Fn The type of function used for conditional toggling.
  /// @param key The identifier of the node or edge to be toggled.
  /// @param Enabled The constant indicating that the operation will act on
  /// enabled nodes or edges.
  /// @param Disabled The constant indicating that the operation will act on
  /// disabled nodes or edges.
  template <node_type T>
  auto toggle(const Enabled) -> void {
    helpers::move_all(get<T>(nodes_enable_), get<T>(nodes_disable_),
                      [](const auto& node) { node->disable(true); });
    revision_->update();
  }

  template <node_type T>
  auto toggle(const Disabled) -> void {
    helpers::move_all(get<T>(nodes_disable_), get<T>(nodes_enable_),
                      [](const auto& node) { node->disable(false); });
    revision_->update();
  }

  template <edge_type E, node_type T>
  auto toggle(const handle<T>& node, const Enabled) -> void {
    node->template apply<E>([this](const auto& edge) {
      helpers::move(get<E>(edges_enable_), get<E>(edges_disable_), edge,
                    [](const auto& e) { e->disable(true); });
    });
  }

  template <edge_type E, node_type T>
  auto toggle(const handle<T>& node, const Disabled) -> void {
    node->template apply<E>([this](const auto& edge) {
      helpers::move(get<E>(edges_disable_), get<E>(edges_enable_), edge,
                    [](const auto& e) { e->disable(false); });
    });
  }

  template <node_type T, class Fn>
  requires std::is_invocable_v<Fn, handle<T>&>
  auto toggle(Fn&& func, const Enabled) -> void {
    helpers::move_if(get<T>(nodes_enable_), get<T>(nodes_disable_), std::forward<Fn>(func),
                     [](const auto& node) { node->disable(true); });
    revision_->update();
  }

  template <node_type T, class Fn>
  requires std::is_invocable_v<Fn, handle<T>&>
  auto toggle(Fn&& func, const Disabled) -> void {
    helpers::move_if(get<T>(nodes_disable_), get<T>(nodes_enable_), std::forward<Fn>(func),
                     [](const auto& node) { node->disable(false); });
    revision_->update();
  }

  template <node_type T>
  auto toggle(const Key& key, const Enabled) -> void {
    helpers::move(get<T>(nodes_enable_), get<T>(nodes_disable_), key,
                  [](const auto& node) { node->disable(true); });
    revision_->update();
  }

  template <node_type T>
  auto toggle(const Key& key, const Disabled) -> void {
    helpers::move(get<T>(nodes_disable_), get<T>(nodes_enable_), key,
                  [](const auto& node) { node->disable(false); });
    revision_->update();
  }

  template <edge_type T>
  auto toggle(const Enabled) -> void {
    helpers::move_all(get<T>(edges_enable_), get<T>(edges_disable_),
                      [](const auto& edge) { edge->disable(true); });
  }

  template <edge_type T>
  auto toggle(const Disabled) -> void {
    helpers::move_all(get<T>(edges_disable_), get<T>(edges_enable_),
                      [](const auto& edge) { edge->disable(false); });
  }

  template <edge_type T>
  auto toggle(const handle<T>& edge, const Enabled) -> void {
    helpers::move(get<T>(edges_enable_), get<T>(edges_disable_), edge,
                  [](const auto& e) { e->disable(true); });
  }

  template <edge_type T>
  auto toggle(const handle<T>& edge, const Disabled) -> void {
    helpers::move(get<T>(edges_disable_), get<T>(edges_enable_), edge,
                  [](const auto& e) { e->disable(false); });
  }

  template <edge_type T, class Fn>
  requires std::is_invocable_v<Fn, handle<T>&>
  auto toggle(Fn&& func, const Enabled) -> void {
    helpers::move_if(get<T>(edges_enable_), get<T>(edges_disable_), std::forward<Fn>(func),
                     [](const auto& edge) { edge->disable(true); });
  }

  template <edge_type T, class Fn>
  requires std::is_invocable_v<Fn, handle<T>&>
  auto toggle(Fn&& func, const Disabled) -> void {
    helpers::move_if(get<T>(edges_disable_), get<T>(edges_enable_), std::forward<Fn>(func),
                     [](const auto& edge) { edge->disable(false); });
  }

  template <type_list E, Selection value>
  auto toggle(const SelectionConstant<value> side) -> void {
    toggle(E{}, side);
  }

  template <type_list E, class T, Selection value>
  auto toggle(T&& variant, const SelectionConstant<value> side) -> void {
    toggle(E{}, side, std::forward<T>(variant));
  }

  /// @brief Provides the memory resource in use.
  /// @return A memory resource pointer.
  auto memory() const { return memory_; }

 protected:
  template <class T>
  auto& get(TupleMapShared& data) {
    return std::get<MapShared<T>>(data);
  }
  template <class T>
  const auto& get(const TupleMapShared& data) const {
    return std::get<MapShared<T>>(data);
  }

  template <class T>
  auto& get(TupleSetShared& data) {
    return std::get<SetShared<T>>(data);
  }
  template <class T>
  const auto& get(const TupleSetShared& data) const {
    return std::get<SetShared<T>>(data);
  }

  template <class T, Selection value = Selection::kAll>
  auto select(TupleMapShared& enable, TupleMapShared& disable, const SelectionConstant<value> side) {
    return helpers::select(get<T>(enable), get<T>(disable), side);
  }

  template <class T, Selection value = Selection::kAll>
  auto select(const TupleMapShared& enable, const TupleMapShared& disable,
              const SelectionConstant<value> side) const {
    return helpers::cselect(get<T>(enable), get<T>(disable), side);
  }

  template <class T, Selection value = Selection::kAll>
  auto select(TupleSetShared& enable, TupleSetShared& disable, const SelectionConstant<value> side) {
    return helpers::select(get<T>(enable), get<T>(disable), side);
  }
  template <class T, Selection value = Selection::kAll>
  auto select(const TupleSetShared& enable, const TupleSetShared& disable,
              const SelectionConstant<value> side) const {
    return helpers::cselect(get<T>(enable), get<T>(disable), side);
  }

  template <class T>
  auto link_edge(TupleSetShared& edges, const handle<T>& edge) -> void {
    std::ignore = get<T>(edges).insert(edge);
  }

  template <class T>
  auto unlink_edge(TupleSetShared& edges, const handle<T>& edge) -> void {
    std::ignore = get<T>(edges).erase(edge);
  }

  template <class N>
  auto unlink_node(const handle<N>& node) -> void {
    node->apply([&](auto& e) {
      e->apply([&](const auto& n) {
        if (not equal(node, n)) {
          n->unlink(e);
        }
      });
      unlink_edge(edges_enable_, e);
      unlink_edge(edges_disable_, e);
    });
    node->unlink();
  }

  template <class E, class N>
  auto unlink_node(const handle<N>& node) -> void {
    node->template apply<E>([&](auto e) {
      e->apply([&](const auto& n) {
        if (not equal(node, n)) {
          n->unlink(e);
        }
      });
      unlink_edge(edges_enable_, e);
      unlink_edge(edges_disable_, e);
    });
    node->template unlink<E>();
  }

  template <class N, class... T>
  auto unlink_node(const handle<N>& node, types<T...>) -> void {
    (unlink_node<T>(node), ...);
  }

  template <class... T, Selection value>
  auto size(const types<T...>, const SelectionConstant<value> side) const {
    return (size<T>(side) + ...);
  }

  template <class Fn, class... T, Selection value>
  auto apply(Fn&& func, types<T...>, const SelectionConstant<value> side) -> void {
    auto& ref_func = helpers::lreference<Fn>(std::forward<Fn>(func));
    (apply<T>(ref_func, side), ...);
  }

  template <class Fn, class... T, Selection value>
  auto apply(Fn&& func, types<T...>, const SelectionConstant<value> side) const -> void {
    auto& ref_func = helpers::lreference<Fn>(std::forward<Fn>(func));
    (apply<T>(ref_func, side), ...);
  }

  template <class... T, Selection value>
  auto destroy(types<T...>, const SelectionConstant<value> side) -> void {
    (destroy<T>(side), ...);
  }

  template <class... E, Selection value, class... Ts>
  auto toggle(types<E...>, const SelectionConstant<value> side, Ts&&... args) -> void {
    (toggle<E>(helpers::lreference<Ts>(args)..., side), ...);
  }

  const auto& revision() const { return revision_; }

 private:
  std::pmr::monotonic_buffer_resource memory_monotonic_;
  std::pmr::unsynchronized_pool_resource memory_pool_;
  helpers::bounded_memory_resource<Config::CacheBlockMaxSize> memory_bounded_;
  std::pmr::memory_resource* memory_;
  TupleMapShared nodes_enable_;
  TupleMapShared nodes_disable_;
  TupleSetShared edges_enable_;
  TupleSetShared edges_disable_;
  SharedRevision revision_;
};
}  // namespace vortex::graph
#endif  // VORTEX_FOUNDATION_GRAPH_GRAPH_CONTAINER_HPP
