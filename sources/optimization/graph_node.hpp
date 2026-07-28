/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.graph component
/// ===========================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_NODE_HPP
#define VORTEX_OPTIMIZATION_GRAPH_NODE_HPP

#include "base/graph.hpp"
#include "base/math.hpp"
#include "helpers/compat.hpp"
#include "optimization/graph_config.hpp"

namespace vortex::graph::optimization {

using graph::SharedRevision;
using graph::Types;

/// ===========================================================================
/// @brief Type Container: A utility template to define edges types.
/// ===========================================================================
template <class... Ts>
using Edges = Types<Ts...>;

/// ===========================================================================
/// @brief Node: Represents a graph node used for estimations.
///
/// @tparam Derived      The derived node type.
/// @tparam MaxDimension The maximum dimension of the node.
/// @tparam Type         The data type of the node's estimation.
/// @tparam Edges        The type of edges connected to the node.
/// @tparam Config       Configuration settings for the node.
/// ===========================================================================
template <class Derived, size_t MaxDimension, class Type, class Edges, class Config = DefaultConfig>
class Node : public detail::TypesBuild<graph::Node, Edges> {
 public:
  /// @brief helper alises
  static constexpr size_t kMaxDimension{MaxDimension};

  using Number = typename Config::Number;
  using Key = typename Config::Key;

  template <size_t D>
  using Matrix = math::HybridMatrix<Number, D, kMaxDimension>;
  using Vector = math::HybridVector<Number, kMaxDimension>;
  using Base = detail::TypesBuild<graph::Node, Edges>;
  using Base::Base;

  /// @brief Node constructor.
  /// @param memory Memory resource.
  /// @param key Node identifier.
  /// @param revision Node revision tracker
  Node(const SharedRevision& revision, const Key& key, std::pmr::memory_resource* const memory)
      : Base{key, memory},
        dimension_{kMaxDimension},
        estimation_{},
        backlog_{},
        key_{key},
        revision_{revision} {
    backlog_.emplace_back();
  }
  Node(const Key& key, std::pmr::memory_resource* const memory)
      : Node(SharedRevision{memory}, memory, key) {}

  /// @brief Gets the node dimension.
  /// @return  The current dimension.
  size_t dimension() const { return dimension_; }

  /// @brief Sets the node dimension.
  /// @param value The new dimension.
  void dimension(size_t value) {
    dimension_ = value;
    revision_->update();
  }

  /// @brief Gets estimation.
  /// @return The current estimation.
  const Type& estimation() const { return estimation_; }

  /// @brief Set estimation.
  /// @param value The new estimation.
  void estimation(const Type& value) {
    estimation_ = value;
    self()->postEstimation();
  }

  /// @brief Gets the key (identifier).
  /// @return The identifier value.
  const Key& key() const { return key_; }

  /// @brief Backlog handing functions
  void push() { backlog_.emplace_back(estimation_); }

  void pull() { estimation(backlog_.back()); }

  void revert(size_t n = 1) {
    const auto offset =
        std::min(graph::detail::narrow_cast<std::ptrdiff_t>(n), std::ssize(backlog_) - 1);
    std::ignore = backlog_.erase(std::prev(std::end(backlog_), offset), std::end(backlog_));
    pull();
  }

  /// @brief Graph solver support information.
  /// This has the internal position that should be only accessed by
  /// BlockGraphSolver type
  class Position {
    template <class, class>
    friend class BlockGraphSolver;

   protected:
    auto operator()() { return value_; }

    void operator()(size_t value) { value_ = value; }

   private:
    size_t value_{0};
  } position;

  /// @brief Updates the estimation
  /// @tparam Delta
  /// @param delta to be added to the current estimation
  template <class Delta>
  void updateEstimation(const Delta delta) {
    VORTEX_PRECONDITION(std::size(delta) == dimension(),
                          "delta update size do not match with node dimension");
    estimation(self()->plus(delta));
  }

 protected:
  /// @brief Helper function for casting to derived type.
  /// @return A pointer to derived type.
  Derived* self() { return static_cast<Derived*>(this); }

  /// @brief Runs after a new estimation is set.
  void postEstimation() {}

 private:
  using Backlog = graph::detail::CircularBuffer<Type, Config::BacklogCapacity>;

  size_t dimension_;
  Type estimation_;
  Backlog backlog_;
  Key key_;
  SharedRevision revision_;
};

}  // namespace vortex::graph::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_NODE_HPP
