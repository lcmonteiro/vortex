/// ===============================================================================================
/// @file
///
/// @brief vortex.graph component
/// ===============================================================================================
#ifndef VORTEX_OPTIMIZATION_GRAPH_NODE_HPP
#define VORTEX_OPTIMIZATION_GRAPH_NODE_HPP

#include <cstddef>

#include "foundation/graph.hpp"
#include "foundation/math.hpp"
#include "helpers/buffer.hpp"
#include "helpers/contracts.hpp"
#include "optimization/graph_config.hpp"

namespace vortex::optimization {

/// ===============================================================================================
/// @brief Utility alias to define the edge types of a node.
/// ===============================================================================================
template <class... Ts>
using Edges = helpers::types<Ts...>;

/// ===============================================================================================
/// @brief Represents a graph node used for estimations.
///
/// @tparam Derived      The derived node type.
/// @tparam Dimension    The dimension of the node.
/// @tparam Type         The data type of the node's estimation.
/// @tparam Edges        The type of edges connected to the node.
/// @tparam Config       Configuration settings for the node.
/// ===============================================================================================
template <class Derived, auto Dimension, class Type, class Edges, class Config = default_config>
class Node : public helpers::types_build_t<graph::node, Edges> {
 public:
  /// @brief Helper aliases.
  static constexpr auto kDimension = std::size_t{Dimension};

  using Number = typename Config::number_type;
  using Key = typename Config::key_type;

  template <std::size_t D>
  using Matrix = math::static_matrix<Number, D, kDimension>;
  using Vector = math::static_vector<Number, kDimension>;

  using Base = helpers::types_build_t<graph::node, Edges>;
  using Base::Base;

  /// @brief Node constructor.
  /// @param key Node identifier.
  /// @param memory Memory resource.
  Node(const Key& key, std::pmr::memory_resource* const memory)
      : Base{key, memory}, estimation_{}, backlog_{}, key_{key} {
    push();
  }

  /// @brief Gets the node dimension.
  /// @return The dimension.
  constexpr auto dimension() -> std::size_t { return kDimension; }

  /// @brief Gets estimation.
  /// @return The current estimation.
  auto estimation() const -> const Type& { return estimation_; }

  /// @brief Sets estimation.
  /// @param value The new estimation.
  auto estimation(const Type& value) -> void {
    estimation_ = value;
    self()->postEstimation();
  }

  /// @brief Gets the key (identifier).
  /// @return The identifier value.
  auto key() const -> const Key& { return key_; }

  /// @brief Backlog handling functions.
  auto push() -> void { backlog_.push(estimation_); }
  auto pull() -> void { estimation(backlog_.back()); }

  auto revert(std::size_t n = 1) -> void {
    const std::size_t drop = std::size(backlog_) > 1 ? std::min(n, std::size(backlog_) - 1) : 0;
    for (std::size_t i = 0; i < drop; ++i) {
      backlog_.pop();
    }
    pull();
  }

  /// @brief Graph solver support information.
  /// This has the internal position that should be only accessed by
  /// BlockGraphSolver type.
  class Position {
    template <class, class>
    friend class BlockGraphSolver;

   protected:
    auto operator()() { return value_; }
    auto operator()(std::size_t value) -> void { value_ = value; }

   private:
    std::size_t value_{0};
  } position;

  /// @brief Updates the estimation.
  /// @tparam Delta The delta type.
  /// @param delta The delta to be added to the current estimation.
  template <class Delta>
  auto update(const Delta delta) -> void {
    VORTEX_PRECONDITION(std::size(delta) == dimension(),
                        "delta update size do not match with node dimension");
    estimation(self()->plus(delta));
  }

 protected:
  /// @brief Helper function for casting to derived type.
  /// @return A pointer to derived type.
  auto self() -> Derived* { return static_cast<Derived*>(this); }

  /// @brief Runs after a new estimation is set.
  auto postEstimation() -> void {}

 private:
  using Backlog = helpers::buffer<Type, Config::backlog_capacity>;

  Type estimation_;
  Backlog backlog_;
  Key key_;
};

}  // namespace vortex::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_NODE_HPP
