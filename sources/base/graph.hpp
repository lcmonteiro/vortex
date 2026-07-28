/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.graph_engine component
/// ===========================================================================
#ifndef VORTEX_BASE_GRAPH_HPP
#define VORTEX_BASE_GRAPH_HPP
#include <memory>
#include <memory_resource>
#include <optional>
#include <tuple>

#include "base/graph_config.hpp"
#include "base/graph_edge.hpp"
#include "base/graph_node.hpp"
#include "helpers/apply.hpp"
#include "helpers/build.hpp"
#include "helpers/memory.hpp"
#include "helpers/move.hpp"
#include "helpers/revision.hpp"
#include "helpers/select.hpp"
#include "helpers/shared.hpp"
#include "helpers/traits.hpp"
#include "helpers/types.hpp"

namespace vortex::graph {
/// ===========================================================================
/// Helper Types
/// ===========================================================================
using detail::Revision;
using detail::Shared;
using detail::Types;

template <class T>
using OptionalShared = std::optional<Shared<T>>;

using SharedRevision = Shared<detail::Revision>;

/// ===========================================================================
/// Helpers Functions
/// ===========================================================================
// LCOV_EXCL_START
template <class... T>
inline constexpr auto ToTypes(const Types<T...>& t) {
  return t;
}
// LCOV_EXCL_STOP

/// ===========================================================================
/// @class Graph
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
/// ===========================================================================
template <class Tnodes, class Tedges, class Config = DefaultConfig>
class Graph {
 protected:
  /// concepts
  template <class T>
  using if_node = trait::if_valid<decltype(ToNode(std::declval<T>()))>;

  template <class T>
  using if_edge = trait::if_valid<decltype(ToEdge(std::declval<T>()))>;

  template <class T>
  using if_types = trait::if_valid<decltype(ToTypes(std::declval<T>()))>;

  template <class F, class N>
  using if_predicate = trait::if_valid<std::enable_if_t<std::is_invocable_v<F, Shared<N>&>>>;

  /// types
  using Key = typename Config::Key;

  template <class K, class V>
  using Map = typename Config::template Map<K, V>;
  template <class V>
  using Set = typename Config::template Set<V>;

  template <class T>
  using MapShared = Map<Key, Shared<T>>;
  template <class T>
  using SetShared = Set<Shared<T>>;

  using TupleMapShared = detail::TypesWrapBuild<std::tuple, MapShared, Tnodes>;
  using TupleSetShared = detail::TypesWrapBuild<std::tuple, SetShared, Tedges>;

  template <detail::Option value>
  using OptionConstant = detail::OptionConstant<value>;
  using Option = detail::Option;

 public:
  using Nodes = Tnodes;
  using Edges = Tedges;

  using All = OptionConstant<Option::kAll>;
  using Enabled = OptionConstant<Option::kLeft>;
  using Disabled = OptionConstant<Option::kRight>;
  static constexpr auto kAll = All{};
  static constexpr auto kEnabled = Enabled{};
  static constexpr auto kDisabled = Disabled{};

  /// @brief Graph main constructor.
  /// @param memory_resource
  Graph(const SharedRevision& revision, std::pmr::memory_resource* const memory_resource)
      : memory_monotonic_{Config::CacheInitSize, memory_resource},
        memory_pool_{{0, Config::CacheBlockMaxSize}, &memory_monotonic_},
        memory_bounded_{&memory_pool_},
        memory_{&memory_bounded_},
        nodes_enable_{detail::build<TupleMapShared>(memory_resource)},
        nodes_disable_{detail::build<TupleMapShared>(memory_resource)},
        edges_enable_{detail::build<TupleSetShared>(memory_resource)},
        edges_disable_{detail::build<TupleSetShared>(memory_resource)},
        revision_{revision} {}

  explicit Graph(std::pmr::memory_resource* const memory_resource)
      : Graph(SharedRevision{memory_resource}, memory_resource) {}

  Graph(const Graph&) = delete;
  Graph(Graph&&) = delete;
  Graph& operator=(const Graph&) = delete;
  Graph& operator=(Graph&&) = delete;

  /// @brief Graph destructor
  ~Graph() { destroy(); }

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
  template <class T, class... A, if_node<T> = 0>
  auto build(const Key& key, A&&... args) {
    auto node = Shared<T>(memory_, std::forward<A>(args)..., key, memory_);
    destroy<T>(key);
    std::ignore = get<T>(nodes_enable_).emplace(key, node);
    revision_->update();
    return node;
  }

  template <class T, class... A, if_edge<T> = 0>
  auto build(A&&... args) {
    auto edge = Shared<T>(memory_, std::forward<A>(args)...);
    edge->apply([&](const auto& v) { v->link(edge); });
    linkEdge(edges_enable_, edge);
    revision_->update();
    return edge;
  }

  /// @defgroup SizeFunctions Size Functions
  /// @brief Functions to calculate the sizes of nodes, edges, and types in the
  /// graph.
  /// @param T The type of node or edge to count.
  /// @param value The option value that determines the counting behavior.
  /// This can be one of the enumerated options (enabled, disabled, all).
  /// @param option The `OptionConstant` that specifies whether to count
  /// enabled, disabled, or all nodes and edges.
  template <class T, Option value = Option::kAll, if_node<T> = 0>
  auto size(const OptionConstant<value> option = All{}) const {
    size_t result{0};
    for (const MapShared<T>& map : select<T>(nodes_enable_, nodes_disable_, option)) {
      result += map.size();
    }
    return result;
  }

  template <class T, Option value = Option::kAll, if_edge<T> = 0>
  auto size(const OptionConstant<value> option = All{}) const {
    size_t result{0};
    for (const SetShared<T>& set : select<T>(edges_enable_, edges_disable_, option)) {
      result += set.size();
    }
    return result;
  }

  template <class T, Option value = Option::kAll, if_types<T> = 0>
  auto size(const OptionConstant<value> option = All{}) const {
    return size(T{}, option);
  }

  template <Option value = Option::kAll>
  auto size(const OptionConstant<value> option = All{}) const {
    return size<Nodes>(option) + size<Edges>(option);
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
  /// @param value The option constant that determines the scope of the
  /// operation.
  /// @param option The constant indicating the selection criteria.
  template <class T, class Fn, Option value = Option::kAll, if_node<T> = 0>
  void apply(Fn&& func, const OptionConstant<value> option = All{}) const {
    auto& ref_func = trait::lreference<Fn>(std::forward<Fn>(func));
    for (const MapShared<T>& map : select<T>(nodes_enable_, nodes_disable_, option)) {
      detail::apply(ref_func, map);
    }
  }

  template <class T, class Fn, Option value = Option::kAll, if_edge<T> = 0>
  void apply(Fn&& func, const OptionConstant<value> option = All{}) const {
    auto& ref_func = trait::lreference<Fn>(std::forward<Fn>(func));
    for (const SetShared<T>& set : select<T>(edges_enable_, edges_disable_, option)) {
      detail::apply(ref_func, set);
    }
  }

  template <class T, class Fn, Option value = Option::kAll, if_types<T> = 0>
  void apply(Fn&& func, const OptionConstant<value> option = All{}) const {
    apply(std::forward<Fn>(func), T{}, option);
  }

  template <class Fn, Option value = Option::kAll>
  void apply(Fn&& func, const OptionConstant<value> option = All{}) const {
    auto& ref_func = trait::lreference<Fn>(std::forward<Fn>(func));
    apply<Nodes>(ref_func, option);
    apply<Edges>(ref_func, option);
  }

  /// @defgroup FindFunctions Find Functions
  /// @brief Functions to find nodes in the graph.
  /// The search can be performed based on a specific key or by applying
  /// a predicate to determine the desired node.
  /// @param N The type of the node to search for.
  /// @param Fn The type of the predicate function used for searching.
  /// @param value The option constant that determines the scope of the search.
  /// @param key The key used to locate a specific node.
  /// @param evaluate The predicate function used to evaluate each node during
  /// the search.
  /// @param option The constant indicating the selection criteria for nodes.
  template <class N, Option value = Option::kAll, if_node<N> = 0>
  OptionalShared<N> find(const Key& key, const OptionConstant<value> option = All{}) const {
    const auto& nodes{select<N>(nodes_enable_, nodes_disable_, option)};
    for (const MapShared<N>& map : nodes) {
      if (auto found = map.find(key); found != std::end(map)) {
        return std::get<Shared<N>>(*found);
      }
    }
    return std::nullopt;
  }

  template <class N, class Fn, Option value = Option::kAll, if_node<N> = 0, if_predicate<Fn, N> = 0>
  OptionalShared<N> find(Fn&& evaluate, const OptionConstant<value> option = All{}) const {
    auto& ref_evaluate = trait::lreference<Fn>(std::forward<Fn>(evaluate));
    const auto& nodes{select<N>(nodes_enable_, nodes_disable_, option)};
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
  /// @brief Functions to unlink unlink/delete edges from nodes.
  /// These functions provide utilities for unlinking nodes from the graph
  /// based on their keys or by iterating through all nodes of a specified type.
  /// @param N The type of the node to unlink.
  /// @param E The type of the edge to unlink (if applicable).
  /// @param value The option constant that determines the scope of the
  /// unlinking operation.
  /// @param key The key used to locate a specific node for unlinking.
  /// @param option The constant indicating the selection criteria for nodes.
  template <class N, Option value = Option::kAll, if_node<N> = 0>
  void unlink(Key key, const OptionConstant<value> option = All{}) {
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, option)) {
      if (auto iter = map.find(key); iter != map.end()) {
        unlinkNode(std::get<Shared<N>>(*iter));
        revision_->update();
        return;
      }
    }
  }

  template <class N, class E, Option value = Option::kAll, if_node<N> = 0, if_edge<E> = 0>
  void unlink(Key key, const OptionConstant<value> option = All{}) {
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, option)) {
      if (auto iter = map.find(key); iter != map.end()) {
        unlinkNode<E>(std::get<Shared<N>>(*iter));
        revision_->update();
        return;
      }
    }
  }

  template <class N, class E, Option value = Option::kAll, if_node<N> = 0, if_types<E> = 0>
  void unlink(Key key, const OptionConstant<value> option = All{}) {
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, option)) {
      if (auto iter = map.find(key); iter != map.end()) {
        unlinkNode(std::get<Shared<N>>(*iter), E{});
        revision_->update();
        return;
      }
    }
  }

  template <class N, Option value = Option::kAll, if_node<N> = 0>
  void unlink(const OptionConstant<value> option = All{}) {
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, option)) {
      for (const auto& node : map) {
        unlinkNode(std::get<Shared<N>>(node));
      }
    }
    revision_->update();
  }

  template <class N, class E, Option value = Option::kAll, if_node<N> = 0, if_edge<E> = 0>
  void unlink(const OptionConstant<value> option = All{}) {
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, option)) {
      for (const auto& node : map) {
        unlinkNode<E>(std::get<Shared<N>>(node));
      }
    }
    revision_->update();
  }

  template <class N, class E, Option value = Option::kAll, if_node<N> = 0, if_types<E> = 0>
  void unlink(const OptionConstant<value> option = All{}) {
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, option)) {
      for (const auto& node : map) {
        unlinkNode(std::get<Shared<N>>(node), E{});
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
  /// @param value The option constant that determines the scope of the
  /// destruction operation.
  /// @param key The key used to locate a specific node for destruction.
  /// @param func The predicate function used to evaluate each node during the
  /// destruction process.
  /// @param option The constant indicating the selection criteria for nodes.
  template <class N, Option value = Option::kAll, if_node<N> = 0>
  void destroy(Key key, const OptionConstant<value> option = All{}) {
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, option)) {
      if (auto iter = map.find(key); iter != map.end()) {
        unlinkNode(std::get<Shared<N>>(*iter));
        std::ignore = map.erase(iter);
        revision_->update();
        return;
      }
    }
  }

  template <class N, Option value = Option::kAll, if_node<N> = 0>
  void destroy(const OptionConstant<value> option = All{}) {
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, option)) {
      for (const auto& node : map) {
        unlinkNode(std::get<Shared<N>>(node));
      }
      map.clear();
    }
    revision_->update();
  }

  template <class N, class Fn, Option value = Option::kAll, if_node<N> = 0, if_predicate<Fn, N> = 0>
  void destroy(Fn&& func, const OptionConstant<value> option = All{}) {
    auto& ref_func = trait::lreference<Fn>(std::forward<Fn>(func));
    for (MapShared<N>& map : select<N>(nodes_enable_, nodes_disable_, option)) {
      for (auto iter = map.begin(); iter != map.end();) {
        if (ref_func(std::get<Shared<N>>(*iter))) {
          unlinkNode(std::get<Shared<N>>(*iter));
          iter = map.erase(iter);
        } else {
          ++iter;
        }
      }
    }
    revision_->update();
  }

  template <Option value = Option::kAll>
  void destroy(const OptionConstant<value> option = All{}) {
    destroy(Nodes{}, option);
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
  /// @param Enabled The constant indicating that the operation will on enabled
  /// nodes or edges.
  /// @param Disabled The constant indicating that the operation will on
  /// disabled nodes or edges.
  template <class T, if_node<T> = 0>
  void toggle(const Enabled) {
    detail::move_all(get<T>(nodes_enable_), get<T>(nodes_disable_),
                     [](const auto& node) { node->disable(true); });
    revision_->update();
  }

  template <class T, if_node<T> = 0>
  void toggle(const Disabled) {
    detail::move_all(get<T>(nodes_disable_), get<T>(nodes_enable_),
                     [](const auto& node) { node->disable(false); });
    revision_->update();
  }

  template <class E, class T, if_node<T> = 0, if_edge<E> = 0>
  void toggle(const Shared<T>& node, const Enabled) {
    node->template apply<E>([this](const auto& edge) {
      detail::move(get<E>(edges_enable_), get<E>(edges_disable_), edge,
                   [](const auto& e) { e->disable(true); });
    });
  }

  template <class E, class T, if_node<T> = 0, if_edge<E> = 0>
  void toggle(const Shared<T>& node, const Disabled) {
    node->template apply<E>([this](const auto& edge) {
      detail::move(get<E>(edges_disable_), get<E>(edges_enable_), edge,
                   [](const auto& e) { e->disable(false); });
    });
  }

  template <class T, class Fn, if_node<T> = 0, if_predicate<Fn, T> = 0>
  void toggle(Fn&& func, const Enabled) {
    detail::move_if(get<T>(nodes_enable_), get<T>(nodes_disable_), std::forward<Fn>(func),
                    [](const auto& node) { node->disable(true); });
    revision_->update();
  }

  template <class T, class Fn, if_node<T> = 0, if_predicate<Fn, T> = 0>
  void toggle(Fn&& func, const Disabled) {
    detail::move_if(get<T>(nodes_disable_), get<T>(nodes_enable_), std::forward<Fn>(func),
                    [](const auto& node) { node->disable(false); });
    revision_->update();
  }

  template <class T, if_node<T> = 0>
  void toggle(const Key& key, const Enabled) {
    detail::move(get<T>(nodes_enable_), get<T>(nodes_disable_), key,
                 [](const auto& node) { node->disable(true); });
    revision_->update();
  }

  template <class T, if_node<T> = 0>
  void toggle(const Key& key, const Disabled) {
    detail::move(get<T>(nodes_disable_), get<T>(nodes_enable_), key,
                 [](const auto& node) { node->disable(false); });
    revision_->update();
  }

  template <class T, if_edge<T> = 0>
  void toggle(const Enabled) {
    detail::move_all(get<T>(edges_enable_), get<T>(edges_disable_),
                     [](const auto& edge) { edge->disable(true); });
  }

  template <class T, if_edge<T> = 0>
  void toggle(const Disabled) {
    detail::move_all(get<T>(edges_disable_), get<T>(edges_enable_),
                     [](const auto& edge) { edge->disable(false); });
  }

  template <class T, if_edge<T> = 0>
  void toggle(const Shared<T>& edge, const Enabled) {
    detail::move(get<T>(edges_enable_), get<T>(edges_disable_), edge,
                 [](const auto& e) { e->disable(true); });
  }

  template <class T, if_edge<T> = 0>
  void toggle(const Shared<T>& edge, const Disabled) {
    detail::move(get<T>(edges_disable_), get<T>(edges_enable_), edge,
                 [](const auto& e) { e->disable(false); });
  }

  template <class T, class Fn, if_edge<T> = 0, if_predicate<Fn, T> = 0>
  void toggle(Fn&& func, const Enabled) {
    detail::move_if(get<T>(edges_enable_), get<T>(edges_disable_), std::forward<Fn>(func),
                    [](const auto& edge) { edge->disable(true); });
  }

  template <class T, class Fn, if_edge<T> = 0, if_predicate<Fn, T> = 0>
  void toggle(Fn&& func, const Disabled) {
    detail::move_if(get<T>(edges_disable_), get<T>(edges_enable_), std::forward<Fn>(func),
                    [](const auto& edge) { edge->disable(false); });
  }

  template <class E, Option value, if_types<E> = 0>
  void toggle(const OptionConstant<value> option) {
    toggle(E{}, option);
  }

  template <class E, class T, Option value, if_types<E> = 0>
  void toggle(T&& variant, const OptionConstant<value> option) {
    toggle(E{}, option, std::forward<T>(variant));
  }

  /// @brief Provides the memory resource in use.
  /// @return a memory resource pointer
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

  template <class T, Option value = Option::kAll>
  auto select(TupleMapShared& enable, TupleMapShared& disable, const OptionConstant<value> option) {
    return detail::select(get<T>(enable), get<T>(disable), option);
  }

  template <class T, Option value = Option::kAll>
  auto select(const TupleMapShared& enable, const TupleMapShared& disable,
              const OptionConstant<value> option) const {
    return detail::cselect(get<T>(enable), get<T>(disable), option);
  }

  template <class T, Option value = Option::kAll>
  auto select(TupleSetShared& enable, TupleSetShared& disable, const OptionConstant<value> option) {
    return detail::select(get<T>(enable), get<T>(disable), option);
  }
  template <class T, Option value = Option::kAll>
  auto select(const TupleSetShared& enable, const TupleSetShared& disable,
              const OptionConstant<value> option) const {
    return detail::cselect(get<T>(enable), get<T>(disable), option);
  }

  template <class T>
  void linkEdge(TupleSetShared& edges, const Shared<T>& edge) {
    std::ignore = get<T>(edges).insert(edge);
  }

  template <class T>
  void unlinkEdge(TupleSetShared& edges, const Shared<T>& edge) {
    std::ignore = get<T>(edges).erase(edge);
  }

  template <class N>
  void unlinkNode(const Shared<N>& node) {
    node->apply([&](auto& e) {
      e->apply([&](const auto& n) {
        if (not equal(node, n)) {
          n->unlink(e);
        }
      });
      unlinkEdge(edges_enable_, e);
      unlinkEdge(edges_disable_, e);
    });
    node->unlink();
  }

  template <class E, class N>
  void unlinkNode(const Shared<N>& node) {
    node->template apply<E>([&](auto e) {
      e->apply([&](const auto& n) {
        if (not equal(node, n)) {
          n->unlink(e);
        }
      });
      unlinkEdge(edges_enable_, e);
      unlinkEdge(edges_disable_, e);
    });
    node->template unlink<E>();
  }

  template <class N, class... T>
  void unlinkNode(const Shared<N>& node, Types<T...>) {
    (unlinkNode<T>(node), ...);
  }

  template <class... T, Option value>
  auto size(const Types<T...>, const OptionConstant<value> option) const {
    return (size<T>(option) + ...);
  }

  template <class Fn, class... T, Option value>
  void apply(Fn&& func, Types<T...>, const OptionConstant<value> option) {
    auto& ref_func = trait::lreference<Fn>(std::forward<Fn>(func));
    (apply<T>(ref_func, option), ...);
  }

  template <class Fn, class... T, Option value>
  void apply(Fn&& func, Types<T...>, const OptionConstant<value> option) const {
    auto& ref_func = trait::lreference<Fn>(std::forward<Fn>(func));
    (apply<T>(ref_func, option), ...);
  }

  template <class... T, Option value>
  void destroy(Types<T...>, const OptionConstant<value> option) {
    (destroy<T>(option), ...);
  }

  template <class... E, Option value, class... Ts>
  void toggle(Types<E...>, const OptionConstant<value> option, Ts&&... args) {
    (toggle<E>(trait::lreference<Ts>(args)..., option), ...);
  }

  const auto& revision() const { return revision_; }

 private:
  std::pmr::monotonic_buffer_resource memory_monotonic_;
  std::pmr::unsynchronized_pool_resource memory_pool_;
  detail::BoundedMemoryResource<Config::CacheBlockMaxSize> memory_bounded_;
  std::pmr::memory_resource* memory_;
  TupleMapShared nodes_enable_;
  TupleMapShared nodes_disable_;
  TupleSetShared edges_enable_;
  TupleSetShared edges_disable_;
  SharedRevision revision_;
};
}  // namespace vortex::graph
#endif  // VORTEX_BASE_GRAPH_HPP
