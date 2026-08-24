/// ===============================================================================================
/// @file
/// @brief Benchmark for the dual-number forward-mode AD engine.
///
/// Reports, for a set of representative edge shapes, what one residual evaluation costs: wall
/// time, allocation count and allocated bytes. Allocation traffic is reported alongside time
/// because it dominates dual-number evaluation -- a change that halves the allocations per
/// operation shows up here long before it shows up in a profile.
///
/// Every scenario is also checked against a central finite difference before it is timed, so this
/// is a test as well as a benchmark: a representation change that speeds evaluation up while
/// getting a derivative wrong fails rather than posting a good number.
///
/// Run it directly to read the table; `ctest` runs it as a single correctness gate.
///
/// Note the qualified `std::sin` / `std::cos` calls below. The dual overloads live in namespace
/// `std`, so an unqualified call in a scalar-generic residual resolves to `::sin(double)` through
/// the implicit conversion to the underlying scalar and silently drops the derivative.
/// ===============================================================================================
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory_resource>
#include <string>
#include <string_view>
#include <vector>

#include "foundation/dual.hpp"
#include "foundation/graph/config.hpp"
#include "helpers/memory.hpp"
#include "tests/fixtures/simple_slam_graph.hpp"
#include "tests/helpers/counting_resource.hpp"

namespace {

using vortex::dual::number;
using vortex::test::counting_resource;
using Dual = number<double>;

/// ===============================================================================================
/// @brief The arena a dual number really runs on, reproduced outside the graph.
///
/// `graph::storage` hands dual allocations to a small-block pool over a big-block pool over a
/// monotonic buffer, and `graph::optimize` installs it for the duration of a run. Benchmarking
/// against anything else -- `new_delete_resource`, a bare monotonic buffer -- measures a different
/// allocator than the library actually uses.
/// ===============================================================================================
class arena {
  using config = vortex::graph::default_config;

  static constexpr auto big_blocks = std::pmr::pool_options{4, config::cache_big_block_max_size};
  static constexpr auto small_blocks =
      std::pmr::pool_options{config::cache_big_block_max_size / config::cache_small_block_max_size,
                             config::cache_small_block_max_size};

 public:
  auto resource() noexcept -> counting_resource* { return &counter_; }

 private:
  std::pmr::monotonic_buffer_resource cache_{config::cache_init_size,
                                             std::pmr::new_delete_resource()};
  std::pmr::unsynchronized_pool_resource big_pool_{big_blocks, &cache_};
  std::pmr::unsynchronized_pool_resource small_pool_{small_blocks, &big_pool_};
  counting_resource counter_{&small_pool_};
};

/// @brief One scenario's measurement, normalised per evaluation.
struct sample {
  std::string name;
  std::size_t tangent{0};    //< active tangent directions seeded into the edge
  std::size_t rows{0};       //< residual rows
  double nanoseconds{0.0};   //< wall time per evaluation
  double allocations{0.0};   //< pmr allocations per evaluation
  double bytes{0.0};         //< pmr bytes per evaluation
  bool residual_rows{true};  //< false when the scenario is not a single residual evaluation
  bool counted{true};        //< false when the counter is upstream of the arena being exercised
};

/// @brief Times @p body, reporting per-iteration wall time and the allocation traffic it caused.
/// @param counter Resource sitting at the top of the arena, tallying what @p body allocates.
/// @param iterations Timed repetitions; a tenth of them run first as an untimed warm-up.
/// @param body Callable returning a double, accumulated so the work cannot be optimised away.
template <class Fn>
auto measure(counting_resource* const counter, std::size_t iterations, Fn&& body) -> sample {
  auto sink = 0.0;
  for (auto i = std::size_t{0}; i < std::max(iterations / 10, std::size_t{1}); ++i) {
    sink += body();
  }

  counter->reset();
  const auto start = std::chrono::steady_clock::now();
  for (auto i = std::size_t{0}; i < iterations; ++i) {
    sink += body();
  }
  const auto elapsed = std::chrono::steady_clock::now() - start;

  // Consume the sink so the loop above cannot be elided; the value itself is not interesting.
  if (std::isnan(sink)) {
    std::cerr << "benchmark produced NaN\n";
  }

  const auto count = static_cast<double>(iterations);
  return sample{
      .name = {},
      .tangent = 0,
      .rows = 0,
      .nanoseconds = std::chrono::duration<double, std::nano>(elapsed).count() / count,
      .allocations = static_cast<double>(counter->allocations()) / count,
      .bytes = static_cast<double>(counter->bytes()) / count,
  };
}

/// @brief Reads one partial by tangent index, for checking a named direction against a finite
/// difference. `number` has no such lookup -- the optimizer walks derivatives in order -- and an
/// absent direction is one the expression does not depend on, so its derivative is zero.
auto partial(const Dual& n, std::size_t index) -> double {
  const auto& dvalues = n.dvalues();
  const auto it = std::lower_bound(std::cbegin(dvalues), std::cend(dvalues), index,
                                   [](const auto& d, std::size_t i) { return d.index < i; });
  return (it == std::cend(dvalues) or it->index != index) ? 0.0 : it->value;
}

/// @brief Reads a dual's value and every active partial into a single number.
///
/// The benchmark's sink: without touching the derivatives, the compiler is free to drop the
/// gradient half of the computation and the timings become meaningless. Index-weighted so that
/// reordering the partials cannot cancel the sum out.
auto accumulate(const Dual& n) -> double {
  auto sink = n.value();
  for (const auto& [index, value] : n.dvalues()) {
    sink += value * static_cast<double>(index + 1);
  }
  return sink;
}

/// ===============================================================================================
/// Scenarios. Each is a residual written once against a generic scalar, exactly as an edge's
/// `error()` is, and evaluated at the origin of the tangent space.
/// ===============================================================================================

/// @brief Relative translation between two 2D positions -- the shape of `position_distance_edge`.
/// Cheapest realistic edge: two 2-dimensional nodes, no transcendentals.
struct translation_2d {
  static constexpr std::string_view name = "translation_2d";
  static constexpr std::size_t tangent = 4;
  static constexpr std::size_t rows = 2;

  template <class T>
  static auto residual(const std::array<T, tangent>& d) -> std::array<T, rows> {
    const auto ax = 1.0 + d[0], ay = 2.0 + d[1];
    const auto bx = 3.0 + d[2], by = 4.5 + d[3];
    return {(bx - ax) - 1.9, (by - ay) - 2.4};
  }
};

/// @brief Relative SE(2) pose measurement: two 3-dimensional poses, residual rotated into the
/// first pose's frame. Introduces transcendentals and a fully coupled 6-wide tangent space.
struct pose_graph_se2 {
  static constexpr std::string_view name = "pose_graph_se2";
  static constexpr std::size_t tangent = 6;
  static constexpr std::size_t rows = 3;

  template <class T>
  static auto residual(const std::array<T, tangent>& d) -> std::array<T, rows> {
    const auto x1 = 0.7 + d[0], y1 = -0.3 + d[1], t1 = 0.4 + d[2];
    const auto x2 = 2.1 + d[3], y2 = 0.9 + d[4], t2 = 1.1 + d[5];
    const auto c = std::cos(t1), s = std::sin(t1);
    const auto dx = x2 - x1, dy = y2 - y1;
    return {
        (c * dx + s * dy) - 1.3,
        (-1.0 * s * dx + c * dy) - 0.6,
        (t2 - t1) - 0.7,
    };
  }
};

/// @brief Bundle-adjustment reprojection: a 6-dimensional camera and a 3-dimensional landmark,
/// through a perspective divide and a radial distortion polynomial. Long dependency chains, and
/// every intermediate ends up depending on all nine directions.
struct bundle_adjustment {
  static constexpr std::string_view name = "bundle_adjustment";
  static constexpr std::size_t tangent = 9;
  static constexpr std::size_t rows = 2;

  template <class T>
  static auto residual(const std::array<T, tangent>& d) -> std::array<T, rows> {
    const auto r0 = 0.02 + d[0], r1 = -0.01 + d[1], r2 = 0.03 + d[2];
    const auto t0 = 0.5 + d[3], t1 = -0.2 + d[4], t2 = 4.0 + d[5];
    const auto px = 1.0 + d[6], py = 0.5 + d[7], pz = 6.0 + d[8];

    // Small-angle rotation of the landmark into the camera frame, then translation.
    const auto cx = (px + (r1 * pz - r2 * py)) + t0;
    const auto cy = (py + (r2 * px - r0 * pz)) + t1;
    const auto cz = (pz + (r0 * py - r1 * px)) + t2;

    const auto ix = cx / cz, iy = cy / cz;
    const auto rr = ix * ix + iy * iy;
    const auto distortion = 1.0 + 0.1 * rr + 0.01 * (rr * rr);
    return {
        500.0 * (ix * distortion) - 120.0,
        500.0 * (iy * distortion) - 80.0,
    };
  }
};

/// @brief A wide calibration-style edge: three 10-dimensional blocks, each residual row touching
/// only its own block. The case where the tangent space is wide but each row's dependency set is
/// narrow -- what a sparse derivative representation is actually for.
struct wide_calibration {
  static constexpr std::string_view name = "wide_calibration";
  static constexpr std::size_t tangent = 30;
  static constexpr std::size_t rows = 3;
  static constexpr std::size_t block = tangent / rows;

  template <class T>
  static auto residual(const std::array<T, tangent>& d) -> std::array<T, rows> {
    // Each row is built in place: a dual has no default state to fill an array with first.
    const auto row = [&d](std::size_t index) {
      auto accumulator = 0.25 + d[index * block];
      for (auto k = std::size_t{1}; k < block; ++k) {
        accumulator = accumulator + 0.5 * d[index * block + k] * d[index * block + (k - 1)];
      }
      return std::sqrt(accumulator * accumulator + 1.0) - 1.0;
    };
    return [&row]<std::size_t... R>(std::index_sequence<R...>) {
      return std::array<T, rows>{row(R)...};
    }(std::make_index_sequence<rows>{});
  }
};

/// @brief A long chain of operations over a narrow tangent space. Almost no derivative work per
/// operation, so it isolates the fixed per-operation overhead: allocate, copy, free.
struct deep_chain {
  static constexpr std::string_view name = "deep_chain";
  static constexpr std::size_t tangent = 3;
  static constexpr std::size_t rows = 1;
  static constexpr std::size_t depth = 64;

  template <class T>
  static auto residual(const std::array<T, tangent>& d) -> std::array<T, rows> {
    auto x = 0.5 + d[0];
    const auto a = 1.1 + d[1];
    const auto b = 0.9 + d[2];
    for (auto k = std::size_t{0}; k < depth; ++k) {
      x = (x * a + b) / (1.0 + x * x * 0.01);
    }
    return {x};
  }
};

/// @brief Checks every partial of @p Scenario against a central finite difference.
/// @return True when every derivative matches; diagnostics go to stderr otherwise.
template <class Scenario>
auto verify(std::pmr::memory_resource* const resource) -> bool {
  constexpr auto D = Scenario::tangent;
  constexpr auto R = Scenario::rows;
  constexpr auto step = 1e-6;
  constexpr auto tolerance = 1e-5;

  const vortex::helpers::memory_scope scope{resource};
  const auto exact = Scenario::template residual<Dual>(vortex::dual::zeros<double, D>());

  auto ok = true;
  for (auto column = std::size_t{0}; column < D; ++column) {
    auto forward = std::array<double, D>{};
    auto backward = std::array<double, D>{};
    forward[column] = step;
    backward[column] = -step;

    const auto high = Scenario::template residual<double>(forward);
    const auto low = Scenario::template residual<double>(backward);
    for (auto row = std::size_t{0}; row < R; ++row) {
      const auto numeric = (high[row] - low[row]) / (2.0 * step);
      const auto analytic = partial(exact[row], column);
      const auto scale = std::max(1.0, std::fabs(numeric));
      if (std::fabs(numeric - analytic) / scale > tolerance) {
        std::cerr << "FAIL " << Scenario::name << ": d[row " << row << "]/d[" << column
                  << "] analytic " << analytic << " vs numeric " << numeric << '\n';
        ok = false;
      }
    }
  }
  return ok;
}

/// @brief Verifies @p Scenario, then times one full residual evaluation of it.
template <class Scenario>
auto benchmark(std::vector<sample>& samples, std::size_t iterations) -> bool {
  arena memory;
  if (not verify<Scenario>(memory.resource())) {
    return false;
  }

  const vortex::helpers::memory_scope scope{memory.resource()};
  auto result = measure(memory.resource(), iterations, [] {
    const auto out =
        Scenario::template residual<Dual>(vortex::dual::zeros<double, Scenario::tangent>());
    auto sink = 0.0;
    for (const auto& element : out) {
      sink += accumulate(element);
    }
    return sink;
  });

  result.name = std::string{Scenario::name};
  result.tangent = Scenario::tangent;
  result.rows = Scenario::rows;
  samples.push_back(std::move(result));
  return true;
}

/// ===============================================================================================
/// End-to-end scenarios: the same machinery reached through the optimizer rather than called
/// directly, so the per-evaluation cost above can be read against what a real update spends.
/// ===============================================================================================

using vortex::test::Position;
using vortex::test::PositionDistanceEdge;
using vortex::test::PositionLocationEdge;
using vortex::test::PositionNode;
using vortex::test::SlamGraph;

/// @brief One `edge::update()`: residual, chi-squared, and both Jacobian blocks for both nodes.
auto benchmark_edge_update(std::vector<sample>& samples, std::size_t iterations) -> void {
  arena memory;
  SlamGraph graph{std::pmr::new_delete_resource()};
  const auto a = graph.build<PositionNode>(SlamGraph::key_type{1});
  const auto b = graph.build<PositionNode>(SlamGraph::key_type{2});
  const auto edge = graph.build<PositionDistanceEdge>(a, b);
  a->estimation(Position{0.25, -0.5});
  b->estimation(Position{1.75, 0.5});
  edge->measurement(Position{1.0, 1.0});

  const vortex::helpers::memory_scope scope{memory.resource()};
  auto result = measure(memory.resource(), iterations, [&] {
    edge->update();
    return edge->chi2();
  });

  result.name = "edge_update";
  result.tangent = 2 * PositionNode::dimension();
  result.rows = PositionDistanceEdge::dimension();
  samples.push_back(std::move(result));
  graph.destroy();
}

/// @brief A three-node SLAM graph optimized from the same starting estimates every time.
/// Includes solver and bookkeeping, so it shows how much of a run the AD actually accounts for.
auto benchmark_graph_optimize(std::vector<sample>& samples, std::size_t iterations) -> void {
  arena memory;
  SlamGraph graph{memory.resource()};
  const auto p1 = graph.build<PositionNode>(SlamGraph::key_type{1});
  const auto p2 = graph.build<PositionNode>(SlamGraph::key_type{2});
  const auto p3 = graph.build<PositionNode>(SlamGraph::key_type{3});
  const auto d1 = graph.build<PositionDistanceEdge>(p1, p2);
  const auto d2 = graph.build<PositionDistanceEdge>(p2, p3);
  const auto l1 = graph.build<PositionLocationEdge>(p1);
  l1->measurement(Position{1, 1});
  d1->measurement(Position{1, 1});
  d2->measurement(Position{0, 0});

  constexpr auto steps = std::size_t{3};
  auto result = measure(memory.resource(), iterations, [&] {
    p1->estimation(Position{0, 0});
    p2->estimation(Position{2, 2});
    p3->estimation(Position{0, 0});
    const auto outcome = graph.optimize(steps);
    return outcome.has_value() ? p2->estimation().x : 0.0;
  });

  result.name = "graph_optimize_x3";
  result.tangent = 3 * PositionNode::dimension();
  result.rows = 0;
  result.residual_rows = false;
  // The graph builds its own arena on top of this resource, so the counter sees only that arena's
  // refills -- not the per-operation traffic inside it. Reporting a count here would read as zero
  // allocations, which is the opposite of true.
  result.counted = false;
  samples.push_back(std::move(result));
  graph.destroy();
}

auto report(const std::vector<sample>& samples) -> void {
  std::cout << "\ndual-number evaluation cost\n"
            << std::string(78, '-') << '\n'
            << std::left << std::setw(20) << "scenario"  //
            << std::right << std::setw(9) << "tangent"   //
            << std::setw(6) << "rows"                    //
            << std::setw(14) << "ns/eval"                //
            << std::setw(14) << "allocs/eval"            //
            << std::setw(14) << "bytes/eval" << '\n'     //
            << std::string(78, '-') << '\n';
  for (const auto& s : samples) {
    std::cout << std::left << std::setw(20) << s.name     //
              << std::right << std::setw(9) << s.tangent  //
              << std::setw(6);
    if (s.residual_rows) {
      std::cout << s.rows;
    } else {
      std::cout << '-';
    }
    std::cout << std::setw(14) << std::fixed << std::setprecision(0) << s.nanoseconds;
    if (s.counted) {
      std::cout << std::setw(14) << std::setprecision(1) << s.allocations  //
                << std::setw(14) << std::setprecision(0) << s.bytes;
    } else {
      std::cout << std::setw(14) << '-' << std::setw(14) << '-';
    }
    std::cout << '\n';
  }
  std::cout << std::string(78, '-') << '\n';
}

}  // namespace

auto main(int argc, char** argv) -> int {
  // A short run is enough for the correctness gate ctest gets; pass a larger count for figures
  // worth quoting.
  auto iterations = std::size_t{20000};
  for (auto i = 1; i < argc; ++i) {
    const auto argument = std::string_view{argv[i]};
    constexpr auto flag = std::string_view{"--iterations="};
    if (argument.starts_with(flag)) {
      iterations = static_cast<std::size_t>(std::atoll(argument.substr(std::size(flag)).data()));
    } else {
      std::cerr << "usage: " << argv[0] << " [--iterations=N]\n";
      return 2;
    }
  }
  if (iterations == 0) {
    std::cerr << "--iterations must be greater than zero\n";
    return 2;
  }

  auto samples = std::vector<sample>{};
  const auto ok =                                            //
      benchmark<translation_2d>(samples, iterations) and     //
      benchmark<pose_graph_se2>(samples, iterations) and     //
      benchmark<bundle_adjustment>(samples, iterations) and  //
      benchmark<wide_calibration>(samples, iterations) and   //
      benchmark<deep_chain>(samples, iterations);
  if (not ok) {
    std::cerr << "\nderivative verification failed\n";
    return 1;
  }

  benchmark_edge_update(samples, iterations);
  benchmark_graph_optimize(samples, std::max(iterations / 10, std::size_t{1}));

  report(samples);
  return 0;
}
