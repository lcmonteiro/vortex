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
class node : public helpers::types_build_t<graph::node, Edges> {
  using estimation_type = Type;
  using base_type = helpers::types_build_t<graph::node, Edges>;
  using base_type::base_type;

 public:
  /// @brief Helper aliases.
  using number_type = typename Config::number_type;
  using key_type = typename Config::key_type;

  template <std::size_t D>
  using matrix_type = math::static_matrix<number_type, D, Dimension>;
  using vector_type = math::static_vector<number_type, Dimension>;

  /// @brief Node constructor.
  /// @param key Node identifier.
  /// @param memory Memory resource.
  node(const key_type& key, std::pmr::memory_resource* const memory)
      : base_type{key, memory}, estimation_{}, backlog_{}, key_{key} {
    push();
  }

  /// @brief Gets the dimension of the node.
  /// @return The dimension of the node.
  static constexpr auto dimension() -> std::size_t { return Dimension; }

  /// @brief Gets estimation.
  /// @return The current estimation.
  auto estimation() const -> const Type& { return estimation_; }

  /// @brief Sets estimation.
  /// @param value The new estimation.
  auto estimation(const Type& value) -> void {
    estimation_ = value;
    self()->on_estimation_update();
  }

  /// @brief Gets the key (identifier).
  /// @return The identifier value.
  auto key() const -> const key_type& { return key_; }

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
  /// block_graph_solver type.
  class position_accessor {
    template <class, class>
    friend class block_graph_solver;

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

  /// @brief Callback function invoked when the estimation is updated.
  /// This function can be overridden in derived classes to perform custom actions
  /// when the estimation is updated.
  auto on_estimation_update() -> void {}

 private:
  using backlog_type = helpers::buffer<Type, Config::backlog_capacity>;

  estimation_type estimation_;
  backlog_type backlog_;
  key_type key_;
};

}  // namespace vortex::optimization

#endif  // VORTEX_OPTIMIZATION_GRAPH_NODE_HPP
