/// ===============================================================================================
/// @file
/// @brief Randomized trajectory optimization test: builds a noisy SLAM problem
/// along a sine-wave reference trajectory (random initial poses, noisy
/// absolute-position priors, and noisy relative-distance loop closures) and
/// verifies the Levenberg-Marquardt solver converges close to the ground
/// truth trajectory.
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

/// @brief Test parameters for the randomized trajectory optimization test.
constexpr auto kNumberNodes = std::size_t{100};
constexpr auto kLinksPerNode = std::size_t{10};
constexpr auto kAmplitude = double{10.0};
constexpr auto kWidth = double{80.0};
constexpr auto kInitialGuessMin = double{-10.0};
constexpr auto kInitialGuessMax = double{100.0};
constexpr auto kRandomSeed = std::size_t{1};
constexpr auto kMaxIterations = std::size_t{15};
constexpr auto kExpectedIterations = std::size_t{3};
constexpr auto kMaxErrorDistance = double{3.0};
constexpr auto kArenaCapacity = std::size_t{0x10000010};

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

class OptimizationTrajectoryTest : public ::testing::Test {
 protected:
  using Key = SlamGraph::key_type;
};

/// @brief Builds a randomized, noisy SLAM problem along a sine trajectory and
/// verifies the optimizer converges close to the ground-truth positions.
TEST_F(OptimizationTrajectoryTest, GivenNoisyTrajectory_ExpectConvergenceNearGroundTruth) {
  auto arena = bounded_arena_resource{kArenaCapacity, std::pmr::new_delete_resource()};
  auto g_ = SlamGraph{&arena};

  auto generator = std::mt19937{kRandomSeed};

  const auto ref_points = BuildReferenceTrajectory(kNumberNodes, kAmplitude, kWidth);

  auto generate_pose = [&generator]() {
    return RandomPoint(generator, kInitialGuessMin, kInitialGuessMax, kInitialGuessMin,
                       kInitialGuessMax);
  };
  auto poses = std::vector<go::handle<PositionNode>>{};
  for (std::size_t idx = 0; idx < kNumberNodes; ++idx) {
    auto pose = poses.emplace_back(g_.build<PositionNode>(Key{idx}));
    const auto estimation = generate_pose();
    pose->estimation(estimation);
    const auto noise = NoiseFactor(generator);
    auto location = g_.build<PositionLocationEdge>(pose);
    location->measurement(Position{ref_points[idx].x * noise.x, ref_points[idx].y * noise.y});
  }

  auto add_noise = [&generator](const Position& p) {
    const auto noise = NoiseFactor(generator);
    return Position{p.x * noise.x, p.y * noise.y};
  };
  for (std::size_t idx = 0; idx < kNumberNodes; ++idx) {
    for (std::size_t link = 0; link < kLinksPerNode; ++link) {
      const auto other = RandomNodeIndex(generator, kNumberNodes);
      if (idx != other) {
        auto distance = g_.build<PositionDistanceEdge>(poses[idx], poses[other]);
        const auto measurement = add_noise(ref_points[other] - ref_points[idx]);
        distance->measurement(measurement);
      }
    }
  }

  const auto result = g_.optimize(kMaxIterations);
  ASSERT_TRUE(result.has_value());
  // Neither flag: this run does not converge, and it is not cut short by the budget either.
  // Levenberg runs out of retries after three kept updates and stops on its own. The
  // estimates still land within kMaxErrorDistance of ground truth, which is what the checks
  // below verify. Recorded explicitly because the previous std::size_t return could not tell
  // convergence from giving up, and a bare "3" read as success.
  EXPECT_FALSE(result.value().converged);
  EXPECT_FALSE(result.value().truncated);
  EXPECT_LE(result.value().updates, kExpectedIterations);
  for (std::size_t idx = 0; idx < kNumberNodes; ++idx) {
    EXPECT_NEAR(poses[idx]->estimation().x, ref_points[idx].x, kMaxErrorDistance)
        << "Node index: " << idx << std::endl;
    EXPECT_NEAR(poses[idx]->estimation().y, ref_points[idx].y, kMaxErrorDistance)
        << "Node index: " << idx << std::endl;
  }
  g_.destroy();
}

}  // namespace
