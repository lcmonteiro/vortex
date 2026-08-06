/// ===============================================================================================
/// @file
/// @brief Randomized trajectory optimization test: builds a noisy SLAM problem
/// along a sine-wave reference trajectory (random initial poses, noisy
/// absolute-position priors, and noisy relative-distance loop closures) and
/// verifies the Levenberg-Marquardt solver converges close to the ground
/// truth trajectory.
/// ===============================================================================================
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <memory_resource>
#include <numbers>
#include <random>
#include <vector>

#include "tests/fixtures/simple_slam_graph.hpp"

namespace {

using namespace vortex::test;

constexpr auto kNumberNodes = std::size_t{30};
constexpr auto kLinksPerNode = std::size_t{4};
constexpr auto kAmplitude = double{10.0};
constexpr auto kWidth = double{80.0};
constexpr auto kInitialGuessMin = double{-10.0};
constexpr auto kInitialGuessMax = double{100.0};
constexpr auto kRandomSeed = std::size_t{1};
constexpr auto kMaxIterations = std::size_t{15};
constexpr auto kMaxErrorDistance = double{3.0};

/// @brief Builds `size` reference points along a sine-wave trajectory.
auto BuildReferenceTrajectory(std::size_t size, double amplitude, double width)
    -> std::vector<Position> {
  auto points = std::vector<Position>(size);
  for (std::size_t i = 0; i < size; ++i) {
    points[i].x = static_cast<double>(i) * (width / static_cast<double>(size));
    points[i].y = amplitude * std::sin(2 * std::numbers::pi * (points[i].x / width));
  }
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
auto RandomPoint(Generator& generator, double x_min, double x_max, double y_min, double y_max)
    -> Position {
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
  using Key = SlamGraph::Key;

  auto TearDown() -> void override { g_.destroy(); }

  SlamGraph g_{std::pmr::new_delete_resource()};
};

/// @brief Builds a randomized, noisy SLAM problem along a sine trajectory and
/// verifies the optimizer converges close to the ground-truth positions.
TEST_F(OptimizationTrajectoryTest, GivenNoisyTrajectory_ExpectConvergenceNearGroundTruth) {
  auto generator = std::mt19937{kRandomSeed};
  const auto ref_points = BuildReferenceTrajectory(kNumberNodes, kAmplitude, kWidth);

  auto poses = std::vector<go::OptionalShared<PositionNode>>(kNumberNodes);
  for (std::size_t idx = 0; idx < kNumberNodes; ++idx) {
    poses[idx] = g_.build<PositionNode>(Key{idx});
    (*poses[idx])
        ->estimation(RandomPoint(generator, kInitialGuessMin, kInitialGuessMax, kInitialGuessMin,
                                 kInitialGuessMax));

    const auto noise = NoiseFactor(generator);
    auto location = g_.build<PositionLocationEdge>(*poses[idx]);
    location->measurement(Position{ref_points[idx].x * noise.x, ref_points[idx].y * noise.y});
  }

  for (std::size_t idx = 0; idx < kNumberNodes; ++idx) {
    for (std::size_t link = 0; link < kLinksPerNode; ++link) {
      const auto other = RandomNodeIndex(generator, kNumberNodes);
      if (idx == other) {
        continue;
      }
      const auto noise = NoiseFactor(generator);
      const auto measurement = Position{(ref_points[other].x - ref_points[idx].x) * noise.x,
                                        (ref_points[other].y - ref_points[idx].y) * noise.y};
      auto distance = g_.build<PositionDistanceEdge>(*poses[idx], *poses[other]);
      distance->measurement(measurement);
    }
  }

  const auto result = g_.optimize(kMaxIterations);
  ASSERT_TRUE(result.has_value());

  for (std::size_t idx = 0; idx < kNumberNodes; ++idx) {
    EXPECT_NEAR((*poses[idx])->estimation().x, ref_points[idx].x, kMaxErrorDistance);
    EXPECT_NEAR((*poses[idx])->estimation().y, ref_points[idx].y, kMaxErrorDistance);
  }
}

}  // namespace
