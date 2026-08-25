/// ===============================================================================================
/// @file
/// @brief Randomized trajectory optimization tests: build a noisy SLAM problem
/// along a sine-wave reference trajectory (random initial poses, noisy
/// absolute-position priors, and noisy relative-distance loop closures) and
/// verify the Levenberg-Marquardt solver converges close to the ground
/// truth trajectory.
///
/// Two scenarios share one problem generator:
/// - a small, fast one that pins the solver's exact behaviour, and
/// - a large one that stresses the whole pipeline -- more poses, denser loop
///   closures, a longer and taller trajectory, and initial guesses much further
///   from the truth.
/// ===============================================================================================
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <memory_resource>
#include <numbers>
#include <random>
#include <ranges>
#include <vector>

#include "tests/fixtures/simple_slam_graph.hpp"
#include "tests/helpers/bounded_arena_resource.hpp"

namespace {

using namespace vortex::test;

/// ===============================================================================================
/// @brief Everything that defines one randomized trajectory problem.
/// ===============================================================================================
struct scenario {
  std::size_t nodes;           //< poses along the reference trajectory
  std::size_t links_per_node;  //< loop closures attempted per pose
  double amplitude;            //< peak height of the reference sine wave
  double width;                //< length of the reference trajectory
  double guess_min;            //< lower corner of the box initial guesses are drawn from
  double guess_max;            //< upper corner of that box
  std::size_t seed;            //< fixes the problem: same seed, same problem, every run
  std::size_t max_iterations;  //< iteration budget handed to optimize()
};

/// @brief Test parameters for the small trajectory optimization test.
constexpr auto kSmall = scenario{
    .nodes = 100,
    .links_per_node = 10,
    .amplitude = 10.0,
    .width = 80.0,
    .guess_min = -10.0,
    .guess_max = 100.0,
    .seed = 1,
    .max_iterations = 15,
};
constexpr auto kExpectedIterations = std::size_t{3};
constexpr auto kMaxErrorDistance = double{3.0};
constexpr auto kArenaCapacity = std::size_t{0x10000010};

/// @brief Test parameters for the large trajectory optimization test.
constexpr auto kLarge = scenario{
    .nodes = 600,
    .links_per_node = 20,
    .amplitude = 25.0,
    .width = 200.0,
    .guess_min = -50.0,
    .guess_max = 250.0,
    .seed = 7,
    .max_iterations = 25,
};

/// @brief Accuracy bounds for the large scenario.
///
/// Measurements carry up to 15% multiplicative noise, so the reachable accuracy is set by the
/// noise rather than by the solver. Both bounds are checked: the mean is what says the trajectory
/// as a whole is recovered, the worst case is what catches a single pose left behind.
constexpr auto kLargeMaxErrorDistance = double{6.0};
constexpr auto kLargeMeanErrorDistance = double{2.0};
/// @brief Arena for the large scenario. It consumes about 332 MiB (measured; the graph's
/// monotonic cache never reclaims, so consumption is the high-water mark). 320 MiB is not enough
/// and 384 MiB only just is, so this leaves roughly half again as headroom -- a debug build hands
/// the arena a null upstream, where coming up short throws rather than quietly growing.
constexpr auto kLargeArenaCapacity = std::size_t{0x20000000};

/// ===============================================================================================
/// @brief The large scenario's system is 1200 dimensions, well past what `default_config` reserves.
///
/// `system_capacity` sizes the solver's up-front reservation for H, x and b, made when the graph
/// is constructed. Left at the default 512, those buffers would instead grow during `optimize()`,
/// where the active memory scope is the graph's own optimization arena -- and the solver outlives
/// that scope. Sizing the reservation to the problem keeps the buffers owned by the resource that
/// will still be current when the solver is destroyed.
/// ===============================================================================================
struct large_config : go::default_config {
  static constexpr auto system_capacity = std::size_t{2048};
};
using LargeSlamGraph = go::graph<Nodes, Edges, large_config>;
using PositionNodeHandles = std::vector<go::handle<PositionNode>>;
using Curve = std::vector<Position>;

/// @brief Builds `size` reference points along a sine-wave trajectory.
auto BuildReferenceTrajectory(std::size_t size, double amplitude, double width) -> Curve {
  auto points = Curve{};
  std::ranges::transform(
      std::views::iota(std::size_t{0}, size),  // Generate indices from 0 to size-1
      std::back_inserter(points),              // Insert results into points
      [amplitude, width, size](std::size_t i) {
        const auto x = static_cast<double>(i) * (width / static_cast<double>(size));
        const auto y = amplitude * std::sin(2 * std::numbers::pi * (x / width));
        return Position{x, y};
      });
  return points;
}

/// @brief Uniform multiplicative noise factor in [0.85, 1.15] per axis.
template <class Generator>
auto NoiseFactor(Generator& generator) -> Position {
  auto distribution = std::uniform_real_distribution<double>{0.85, 1.15};
  return Position{distribution(generator), distribution(generator)};
}

/// @brief Uniform random point within [x_min, x_max] x [y_min, y_max].
template <class Generator>
auto RandomPoint(Generator& generator, double x_min, double x_max, double y_min,
                 double y_max) -> Position {
  auto x_distribution = std::uniform_real_distribution<double>{x_min, x_max};
  auto y_distribution = std::uniform_real_distribution<double>{y_min, y_max};
  return Position{x_distribution(generator), y_distribution(generator)};
}

/// @brief Uniform random node index in [0, num_nodes).
template <class Generator>
auto RandomNodeIndex(Generator& generator, std::size_t num_nodes) -> std::size_t {
  auto distribution = std::uniform_real_distribution<double>{0.0, static_cast<double>(num_nodes)};
  return static_cast<std::size_t>(distribution(generator));
}

/// @brief Populates @p graph with the problem @p setup describes.
/// @return The pose handles, indexed as the reference trajectory is.
template <class Graph>
auto BuildProblem(Graph& graph, const scenario& setup, const Curve& reference)  //
    -> PositionNodeHandles {
  auto generator = std::mt19937{static_cast<std::mt19937::result_type>(setup.seed)};

  auto poses = PositionNodeHandles{};
  poses.reserve(setup.nodes);
  for (std::size_t idx = 0; idx < setup.nodes; ++idx) {
    auto pose =
        poses.emplace_back(graph.template build<PositionNode>(typename Graph::key_type{idx}));
    pose->estimation(
        RandomPoint(generator, setup.guess_min, setup.guess_max, setup.guess_min, setup.guess_max));
    const auto noise = NoiseFactor(generator);
    auto location = graph.template build<PositionLocationEdge>(pose);
    location->measurement(Position{reference[idx].x * noise.x, reference[idx].y * noise.y});
  }

  auto add_noise = [&generator](const Position& p) {
    const auto noise = NoiseFactor(generator);
    return Position{p.x * noise.x, p.y * noise.y};
  };
  for (std::size_t idx = 0; idx < setup.nodes; ++idx) {
    for (std::size_t link = 0; link < setup.links_per_node; ++link) {
      const auto other = RandomNodeIndex(generator, setup.nodes);
      if (idx != other) {
        auto distance = graph.template build<PositionDistanceEdge>(poses[idx], poses[other]);
        distance->measurement(add_noise(reference[other] - reference[idx]));
      }
    }
  }
  return poses;
}

/// @brief Worst and mean distance between the solved poses and the reference trajectory.
struct accuracy {
  double worst{0.0};        //< largest per-axis deviation of any pose
  double mean{0.0};         //< mean euclidean distance over all poses
  std::size_t worst_at{0};  //< index of the pose holding the worst deviation
};

auto Measure(const PositionNodeHandles& poses, const Curve& reference) -> accuracy {
  auto result = accuracy{};
  for (std::size_t idx = 0; idx < std::size(poses); ++idx) {
    const auto& estimation = poses[idx]->estimation();
    const auto deviation = std::max(std::fabs(estimation.x - reference[idx].x),
                                    std::fabs(estimation.y - reference[idx].y));
    if (deviation > result.worst) {
      result.worst = deviation;
      result.worst_at = idx;
    }
    result.mean += std::hypot(estimation.x - reference[idx].x, estimation.y - reference[idx].y);
  }
  result.mean /= static_cast<double>(std::size(poses));
  return result;
}

class OptimizationTrajectoryTest : public ::testing::Test {
 protected:
  using Key = SlamGraph::key_type;
};

/// @brief Builds a randomized, noisy SLAM problem along a sine trajectory and
/// verifies the optimizer converges close to the ground-truth positions.
TEST_F(OptimizationTrajectoryTest, GivenNoisyTrajectory_ExpectConvergenceNearGroundTruth) {
  auto arena = bounded_arena_resource{kArenaCapacity, std::pmr::new_delete_resource()};
  auto g_ = SlamGraph{&arena};

  const auto ref_points = BuildReferenceTrajectory(kSmall.nodes, kSmall.amplitude, kSmall.width);
  const auto poses = BuildProblem(g_, kSmall, ref_points);

  const auto result = g_.optimize(kSmall.max_iterations);
  ASSERT_TRUE(result.has_value());
  // Neither flag: this run does not converge, and it is not cut short by the budget either.
  // Levenberg runs out of retries after three kept updates and stops on its own. The
  // estimates still land within kMaxErrorDistance of ground truth, which is what the checks
  // below verify. Recorded explicitly because the previous std::size_t return could not tell
  // convergence from giving up, and a bare "3" read as success.
  EXPECT_FALSE(result.value().converged);
  EXPECT_FALSE(result.value().truncated);
  EXPECT_LE(result.value().updates, kExpectedIterations);
  for (std::size_t idx = 0; idx < kSmall.nodes; ++idx) {
    EXPECT_NEAR(poses[idx]->estimation().x, ref_points[idx].x, kMaxErrorDistance)
        << "Node index: " << idx << std::endl;
    EXPECT_NEAR(poses[idx]->estimation().y, ref_points[idx].y, kMaxErrorDistance)
        << "Node index: " << idx << std::endl;
  }
  g_.destroy();
}

/// @brief The same problem at scale: 600 poses and about 12,000 loop closures on a longer, taller
/// trajectory, started from initial guesses far outside it.
///
/// Deliberately the expensive test in the suite. Where the small scenario pins exact solver
/// behaviour, this one asks a broader question -- does the pipeline still recover the trajectory
/// when the system is 1200 dimensions and every iteration evaluates twelve thousand
/// dual-number Jacobians -- so it asserts accuracy and budget rather than an exact iteration
/// count, which would only make it brittle.
TEST_F(OptimizationTrajectoryTest, GivenLargeNoisyTrajectory_ExpectConvergenceNearGroundTruth) {
  auto arena = bounded_arena_resource{kLargeArenaCapacity, std::pmr::new_delete_resource()};
  auto g_ = LargeSlamGraph{&arena};

  const auto ref_points = BuildReferenceTrajectory(kLarge.nodes, kLarge.amplitude, kLarge.width);
  const auto poses = BuildProblem(g_, kLarge, ref_points);
  ASSERT_EQ(std::size(poses), kLarge.nodes);

  const auto result = g_.optimize(kLarge.max_iterations);
  ASSERT_TRUE(result.has_value());

  // The budget must not be what stopped it: a truncated run says the scenario outgrew
  // max_iterations and the accuracy checks below would be measuring the budget, not the solver.
  EXPECT_FALSE(result.value().truncated);
  EXPECT_GT(result.value().updates, std::size_t{0});
  EXPECT_LE(result.value().updates, kLarge.max_iterations);

  const auto error = Measure(poses, ref_points);
  EXPECT_LT(error.mean, kLargeMeanErrorDistance)
      << "mean deviation over " << kLarge.nodes << " poses";
  EXPECT_LT(error.worst, kLargeMaxErrorDistance)
      << "worst deviation at node " << error.worst_at << ": estimate ("
      << poses[error.worst_at]->estimation().x << ", " << poses[error.worst_at]->estimation().y
      << ") vs reference (" << ref_points[error.worst_at].x << ", " << ref_points[error.worst_at].y
      << ")";
  g_.destroy();
}

}  // namespace
