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

/// @brief Portable uniform real in [min, max), independent of which
/// standard library implementation is in use.
///
/// `std::mt19937`'s raw output sequence is fully specified by the standard
/// and identical across implementations for a given seed, but
/// `std::uniform_real_distribution`'s algorithm for mapping that raw output
/// to a floating-point value is implementation-defined -- it differs
/// between e.g. the Microsoft STL and libstdc++/libc++. Since this test
/// pins a fixed seed and asserts on the resulting SLAM problem's
/// convergence, using `std::uniform_real_distribution` would make the test
/// generate a *different* randomized problem (and therefore a different
/// expected outcome) on Windows than on Linux. This bypasses that by only
/// depending on the engine's raw, portable output and plain arithmetic.
template <class Generator>
auto PortableUniformReal(Generator& generator, double min, double max) -> double {
  constexpr auto lo = static_cast<double>(Generator::min());
  constexpr auto hi = static_cast<double>(Generator::max());
  const auto unit = (static_cast<double>(generator()) - lo) / (hi - lo);
  return min + unit * (max - min);
}

/// @brief Uniform multiplicative noise factor in [0.85, 1.15] per axis.
template <class Generator>
auto NoiseFactor(Generator& generator) -> Position {
  return Position{PortableUniformReal(generator, 0.85, 1.15),
                  PortableUniformReal(generator, 0.85, 1.15)};
}

/// @brief Uniform random point within [x_min, x_max] x [y_min, y_max].
template <class Generator>
auto RandomPoint(Generator& generator, double x_min, double x_max, double y_min, double y_max)
    -> Position {
  return Position{PortableUniformReal(generator, x_min, x_max),
                  PortableUniformReal(generator, y_min, y_max)};
}

/// @brief Uniform random node index in [0, num_nodes).
template <class Generator>
auto RandomNodeIndex(Generator& generator, std::size_t num_nodes) -> std::size_t {
  return static_cast<std::size_t>(PortableUniformReal(generator, 0.0, static_cast<double>(num_nodes)));
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
  EXPECT_LE(result.value(), kExpectedIterations);
  for (std::size_t idx = 0; idx < kNumberNodes; ++idx) {
    EXPECT_NEAR(poses[idx]->estimation().x, ref_points[idx].x, kMaxErrorDistance)
        << "Node index: " << idx << std::endl;
    EXPECT_NEAR(poses[idx]->estimation().y, ref_points[idx].y, kMaxErrorDistance)
        << "Node index: " << idx << std::endl;
  }
}

}  // namespace
