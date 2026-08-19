/// ===============================================================================================
/// @file
/// @brief Runs one SLAM problem under every linear solver in the tree, to check that the choice
/// of back-end does not change the optimum.
///
/// This is also the only path that reaches `block_graph_solver::update_h_block::symmetric()`:
/// that branch is selected by `LinearSolver::kRequiresFullMatrix`, which only
/// `pcg_linear_solver` sets.
/// ===============================================================================================
#include <gtest/gtest.h>

#include <cstddef>
#include <memory_resource>

#include "optimization/graph.hpp"
#include "optimization/linear_solver_cholesky.hpp"
#include "optimization/linear_solver_default.hpp"
#include "optimization/linear_solver_pcg.hpp"
#include "optimization/types/position.hpp"

namespace {

namespace go = vortex::optimization;

using Position = go::types::position<double>;
using PositionNode = go::types::position_node<double>;
using PositionDistanceEdge = go::types::position_distance_edge<double>;
using PositionLocationEdge = go::types::position_location_edge<double>;

using Nodes = go::nodes<PositionNode>;
using Edges = go::edges<PositionDistanceEdge, PositionLocationEdge>;

/// @brief The stock configuration with only the linear solver swapped out.
template <class LinearSolver>
struct solver_config : go::default_config {
  using linear_solver_type = LinearSolver;
};

template <class LinearSolver>
struct SolverGraph : go::graph<Nodes, Edges, solver_config<LinearSolver>> {
  using Base = go::graph<Nodes, Edges, solver_config<LinearSolver>>;
  using Base::Base;
};

template <class LinearSolver>
class LinearSolverParityTest : public ::testing::Test {};

using LinearSolvers = ::testing::Types<go::default_linear_solver,       //
                                       go::cholesky_linear_solver,      //
                                       go::pcg_linear_solver<double>>;  //
TYPED_TEST_SUITE(LinearSolverParityTest, LinearSolvers);

/// @brief The three-node problem from optimization_test.cpp: a prior pinning p1 to (1,1) and two
/// relative constraints. Every solver must land on p1=(1,1), p2=(2,2), p3=(2,2).
TYPED_TEST(LinearSolverParityTest, GivenTheSameSlamProblem_ExpectTheSameOptimum) {
  using Graph = SolverGraph<TypeParam>;
  using Key = typename Graph::key_type;

  Graph graph{std::pmr::new_delete_resource()};

  auto p1 = graph.template build<PositionNode>(Key{1});
  auto p2 = graph.template build<PositionNode>(Key{2});
  auto p3 = graph.template build<PositionNode>(Key{3});
  auto d1 = graph.template build<PositionDistanceEdge>(p1, p2);
  auto d2 = graph.template build<PositionDistanceEdge>(p2, p3);
  auto l1 = graph.template build<PositionLocationEdge>(p1);

  p1->estimation(Position{0, 0});
  p2->estimation(Position{2, 2});
  p3->estimation(Position{0, 0});

  l1->measurement(Position{1, 1});
  d1->measurement(Position{1, 1});
  d2->measurement(Position{0, 0});

  const auto result = graph.optimize(std::size_t{10});
  ASSERT_TRUE(result.has_value());

  // Loose enough for the iterative back-end, tight enough to catch a wrong optimum.
  constexpr auto kTolerance = 1e-6;
  EXPECT_NEAR(p1->estimation().x, 1.0, kTolerance);
  EXPECT_NEAR(p1->estimation().y, 1.0, kTolerance);
  EXPECT_NEAR(p2->estimation().x, 2.0, kTolerance);
  EXPECT_NEAR(p2->estimation().y, 2.0, kTolerance);
  EXPECT_NEAR(p3->estimation().x, 2.0, kTolerance);
  EXPECT_NEAR(p3->estimation().y, 2.0, kTolerance);

  graph.destroy();
}

/// @brief A converged graph has a chi2 of essentially zero regardless of back-end.
TYPED_TEST(LinearSolverParityTest, GivenAConvergedGraph_ExpectNearZeroChi2) {
  using Graph = SolverGraph<TypeParam>;
  using Key = typename Graph::key_type;

  Graph graph{std::pmr::new_delete_resource()};

  auto p1 = graph.template build<PositionNode>(Key{1});
  auto p2 = graph.template build<PositionNode>(Key{2});
  auto l1 = graph.template build<PositionLocationEdge>(p1);
  auto d1 = graph.template build<PositionDistanceEdge>(p1, p2);

  p1->estimation(Position{5, -3});
  p2->estimation(Position{0, 0});
  l1->measurement(Position{1, 1});
  d1->measurement(Position{1, 1});

  const auto result = graph.optimize(std::size_t{10});
  ASSERT_TRUE(result.has_value());

  EXPECT_NEAR(graph.chi2(), 0.0, 1e-9);

  graph.destroy();
}

}  // namespace
