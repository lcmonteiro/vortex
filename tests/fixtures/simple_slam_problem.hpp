/// ===========================================================================
/// @file
/// @brief Scalar-generic SLAM problem fixture for vortex.
///
/// This mirrors the classic vortex simple SLAM problem but the user types,
/// `plus()` and `error()` are now generic over the scalar type so that the
/// same cost functions can be evaluated with `double` (residual) and with a
/// dual number (exact Jacobian via forward-mode automatic differentiation).
/// ===========================================================================
#ifndef VORTEX_TESTS_FIXTURES_SIMPLE_SLAM_PROBLEM_HPP
#define VORTEX_TESTS_FIXTURES_SIMPLE_SLAM_PROBLEM_HPP

#include <type_traits>

#include "sources/optimization/graph.hpp"

namespace vortex_test {

namespace go = vortex::graph::optimization;

/// @brief A scalar-generic 2D point / translation.
template <class T>
struct Point2 {
  T x{};
  T y{};
};

using Pointd = Point2<double>;

// Forward declarations of the edge types connected to a Pose.
struct PoseLocation;
struct PoseDistance;

/// @brief A 2D pose vertex (dimension 2, estimation is a Point2<double>).
struct Pose : go::Node<Pose, 2, Pointd, go::Edges<PoseLocation, PoseDistance>> {
  using Base = go::Node<Pose, 2, Pointd, go::Edges<PoseLocation, PoseDistance>>;
  using Base::Base;

  /// @brief Manifold retraction. Generic over the increment scalar type so it
  /// works both for the real optimizer step and for dual-number seeding.
  template <class Delta>
  auto plus(const Delta& delta) {
    using Scalar = std::decay_t<decltype(this->estimation().x + delta[0])>;
    return Point2<Scalar>{this->estimation().x + delta[0], this->estimation().y + delta[1]};
  }
};

/// @brief Relative translation measurement between two poses.
struct PoseDistance : go::Edge<PoseDistance, 2, Pointd, go::Nodes<Pose, Pose>> {
  using Base = go::Edge<PoseDistance, 2, Pointd, go::Nodes<Pose, Pose>>;
  using Base::Base;

  template <class T>
  auto error(const Point2<T>& a, const Point2<T>& b) -> typename Base::template Error<T> {
    return {(b.x - a.x) - this->measurement().x, (b.y - a.y) - this->measurement().y};
  }
};

/// @brief Absolute position (prior) measurement of a single pose.
struct PoseLocation : go::Edge<PoseLocation, 2, Pointd, go::Nodes<Pose>> {
  using Base = go::Edge<PoseLocation, 2, Pointd, go::Nodes<Pose>>;
  using Base::Base;

  template <class T>
  auto error(const Point2<T>& a) -> typename Base::template Error<T> {
    return {a.x - this->measurement().x, a.y - this->measurement().y};
  }
};

/// @brief The optimization graph for this problem.
struct SlamGraph : go::Graph<go::Nodes<Pose>, go::Edges<PoseDistance, PoseLocation>> {
  using Base = go::Graph<go::Nodes<Pose>, go::Edges<PoseDistance, PoseLocation>>;
  using Base::Base;
};

}  // namespace vortex_test

#endif  // VORTEX_TESTS_FIXTURES_SIMPLE_SLAM_PROBLEM_HPP
