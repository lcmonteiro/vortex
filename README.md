<p align="center">
  <img src="docs/vortex-logo.png" alt="Vortex — graph optimization engine" width="220">
</p>

# Vortex

Vortex is a **header-only C++20 graph-optimization library** that brings
together compile-time type safety, non-linear least-squares optimization, and
exact automatic differentiation.

Inspired by [g2o](https://github.com/RainerKuemmerle/g2o) and built on
[library-dual](https://github.com/lcmonteiro/library-dual), Vortex uses
forward-mode automatic differentiation to compute exact edge Jacobians directly
from a single scalar-generic `error()` function — eliminating the need to derive
and hand-code Jacobians.

Suitable for graph and factor-graph problems such as SLAM, bundle adjustment,
and sensor calibration.

> 💡 Write your error function once. Vortex gives you the exact Jacobian
> automatically.

---

## Highlights

- **Write the cost once.** A derived edge implements a single templated
  `error(...)`. It is evaluated with `double` to get the residual and with a
  dual number to get the exact Jacobian — no numerical differentiation, no
  hand-written derivatives.
- **Statically typed graph.** Node and edge types, their dimensions, and their
  connectivity are all part of the type system, checked at compile time.
- **Allocation-aware.** Built on `std::pmr` memory resources (monotonic + pool
  + bounded) for predictable, low-overhead allocation.
- **Pluggable solver stack.** Levenberg–Marquardt algorithm, block graph
  solver, and Cholesky / PCG / dense linear back-ends selected through a single
  configuration struct.
- **Header-only.** [Blaze](https://bitbucket.org/blaze-lib/blaze) provides the
  dense linear algebra (backed by LAPACK/BLAS).

---

## Architecture

![docs/architecture.drawio](docs/architecture.drawio.png)

### Source layout

| Path | Responsibility |
| --- | --- |
| [include/vortex/foundation/dual/](include/vortex/foundation/dual/) | Dual-number type (`number<T>`) and math operations for forward-mode automatic differentiation. |
| [include/vortex/foundation/graph/](include/vortex/foundation/graph/) | Core statically-typed graph engine — `graph`, `node`, `edge`, revision tracking, and memory management. |
| [include/vortex/foundation/math/](include/vortex/foundation/math/) | Dense linear-algebra wrappers over [Blaze](https://bitbucket.org/blaze-lib/blaze) (matrix/vector types, inversion, and solvers). |
| [include/vortex/foundation/types/](include/vortex/foundation/types/) | Small supporting containers (e.g. `vector_set`). |
| [include/vortex/optimization/](include/vortex/optimization/) | Optimizer layer: `optimize()`, Levenberg–Marquardt algorithm, block graph solver, and the Cholesky/PCG/default linear solvers. Edges compute exact Jacobians via dual numbers. |
| [include/vortex/helpers/](include/vortex/helpers/) | Compile-time utilities — type lists, apply/invoke, shared/pmr helpers, traits. |
| [tests/](tests/) | GoogleTest unit and end-to-end tests, including a scalar-generic SLAM fixture. |

> `include/vortex/foundation/` groups the core modeling modules (`dual`, `graph`, `math`, `types`).

---

## How automatic differentiation works

Each derived edge implements **one** scalar-generic residual function:

```cpp
namespace go = vortex::optimization;

struct PositionDistanceEdge
    : go::edge<PositionDistanceEdge, 2, Position<double>,
               go::nodes<PositionNode, PositionNode>> {
  using edge::edge;

  template <class T>
  auto error(const Position<T>& a, const Position<T>& b) -> error_vector<T> {
    return {(b.x - a.x) - this->measurement().x,
            (b.y - a.y) - this->measurement().y};
  }
};
```

The library ships this edge as
[`position_distance_edge`](include/vortex/optimization/types/position.hpp),
templated on the scalar type so it works at any precision.

- Evaluated with `T = double` → the **residual** used to compute `chi²`.
- Evaluated with `T = dual::number<double>` → the residual carries its
  **exact partial derivatives**. The optimizer seeds one node's tangent
  increment with independent dual variables and reads the Jacobian directly
  from the dual residual (see `edge::update()` in [include/vortex/optimization/graph_edge.hpp](include/vortex/optimization/graph_edge.hpp)).

No finite differences, no manually maintained Jacobian blocks.

---

## Getting started

### Prerequisites

- A **C++20** compiler.
- **CMake ≥ 3.24**.
- **LAPACK** and **BLAS** development libraries (used by the Blaze dense
  solvers). On Debian/Ubuntu:

  ```bash
  sudo apt-get install liblapack-dev libblas-dev
  ```

Blaze and GoogleTest are fetched automatically via CMake `FetchContent`.

### Build & test

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

To build the library without tests:

```bash
cmake -S . -B build -DVORTEX_BUILD_TESTS=OFF
cmake --build build -j
```

### Use it in your project

The library exports the target `vortex::vortex`:

```cmake
add_subdirectory(vortex)          # or FetchContent
target_link_libraries(my_app PRIVATE vortex::vortex)
```

```cpp
#include "vortex.h"   // pulls in vortex::optimization
```

---

## Minimal example

A tiny 2D pose-graph SLAM problem: three positions, one prior, two relative
constraints. Full code lives in
[tests/fixtures/simple_slam_graph.hpp](tests/fixtures/simple_slam_graph.hpp)
and [tests/optimization_test.cpp](tests/optimization_test.cpp).

```cpp
#include <memory_resource>
#include "tests/fixtures/simple_slam_graph.hpp"

using namespace vortex::test;

SlamGraph g{std::pmr::new_delete_resource()};

// Vertices (2D positions) and factors (edges)
auto p1 = g.build<PositionNode>(SlamGraph::key_type{1});
auto p2 = g.build<PositionNode>(SlamGraph::key_type{2});
auto p3 = g.build<PositionNode>(SlamGraph::key_type{3});
auto d1 = g.build<PositionDistanceEdge>(*p1, *p2);
auto d2 = g.build<PositionDistanceEdge>(*p2, *p3);
auto l1 = g.build<PositionLocationEdge>(*p1);

// Initial estimates + measurements
p1->estimation(Position{0, 0});
p2->estimation(Position{2, 2});
p3->estimation(Position{0, 0});
l1->measurement(Position{1, 1});   // prior:      p1 = (1,1)
d1->measurement(Position{1, 1});   // relative:   p2 - p1 = (1,1)
d2->measurement(Position{0, 0});   // relative:   p3 - p2 = (0,0)

// Optimize (Levenberg–Marquardt); returns helpers::expected<summary, algorithm_error>
const auto result = g.optimize(/*iterations=*/10);
if (result) {
  // Converges to p1=(1,1), p2=(2,2), p3=(2,2)
  const auto& [updates, converged, truncated] = result.value();
}
```

### Defining your own problem

1. **Node** — subclass `go::node<Derived, Dim, EstimationType, go::edges<...>>`
   and implement a scalar-generic `plus(delta)` manifold retraction.
2. **Edge** — subclass `go::edge<Derived, Dim, MeasurementType, go::nodes<...>>`
   and implement a scalar-generic `error(...)` returning `error_vector<T>`.
3. **Graph** — subclass `go::graph<go::nodes<...>, go::edges<...>>`.
4. Build nodes/edges, set estimations & measurements, call `optimize()`.

---

## Configuration

Solver behaviour is selected through a configuration struct (see
[include/vortex/optimization/graph_config.hpp](include/vortex/optimization/graph_config.hpp)).
The defaults are:

| Component | Default |
| --- | --- |
| Scalar `Number` | `double` |
| Algorithm | `levenberg_algorithm` |
| Graph solver | `block_graph_solver` |
| Linear solver | `default_linear_solver` (also available: `Cholesky`, `PCG`) |
| `system_capacity` | `0x200` |

Provide your own struct deriving from `vortex::optimization::default_config` and
pass it as the third template parameter of `go::graph` to swap any of these.

---

## Testing

```bash
ctest --test-dir build --output-on-failure
```

- [tests/dual_test.cpp](tests/dual_test.cpp) — verifies AD rules (product,
  quotient, trig, `atan2`, chain rule) and checks the Jacobian against central
  differences.
- [tests/optimization_test.cpp](tests/optimization_test.cpp) — end-to-end
  optimization that exercises the dual-number Jacobians on the SLAM fixture.

---

## Acknowledgements
- **[g2o](https://github.com/RainerKuemmerle/g2o)** — inspiration for the
  graph optimization engine.
- **[library-dual](https://github.com/lcmonteiro/library-dual)** — forward-mode
  automatic differentiation (dual numbers).
- **[Blaze](https://bitbucket.org/blaze-lib/blaze)** — high-performance C++
  dense linear algebra.
- **[GoogleTest](https://github.com/google/googletest)** — unit testing.
