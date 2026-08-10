# Vortex

**A header-only C++20 graph-optimization library with exact, automatically
differentiated Jacobians.**

Vortex (build target `vortex`) is a compile-time, type-safe non-linear
least-squares optimizer for graph/factor-graph problems such as SLAM, bundle
adjustment and sensor calibration. It combines two ideas:

- a **graph optimization engine** inspired by the classic
  [g2o](https://github.com/RainerKuemmerle/g2o) framework, and
- **forward-mode automatic differentiation** (dual numbers) from
  [library-dual](https://github.com/lcmonteiro/library-dual), so edge
  Jacobians are computed *exactly* from a single scalar-generic `error()`
  function instead of being derived and hand-coded.

> 💡 Write your error function **once**, and the exact **Jacobian** comes for **free**.

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
- **Header-only core** with a thin static-library shim; [Blaze](https://bitbucket.org/blaze-lib/blaze)
  provides the dense linear algebra (backed by LAPACK/BLAS).

---

## Architecture

![docs/architecture.drawio](docs/architecture.drawio.png)

### Source layout

| Path | Responsibility |
| --- | --- |
| [sources/foundation/dual/](sources/foundation/dual/) | Dual-number type (`number<T>`) and math operations for forward-mode automatic differentiation (ported from `b2o`). |
| [sources/foundation/graph/](sources/foundation/graph/) | Core statically-typed graph engine — `Graph`, `Node`, `Edge`, revision tracking, and memory management (from `vortex`). |
| [sources/foundation/math/](sources/foundation/math/) | Dense linear-algebra wrappers over [Blaze](https://bitbucket.org/blaze-lib/blaze) (matrix/vector types, inversion, and solvers). |
| [sources/foundation/types/](sources/foundation/types/) | Small supporting containers (e.g. `vector_set`). |
| [sources/optimization/](sources/optimization/) | Optimizer layer: `optimize()`, Levenberg–Marquardt algorithm, block graph solver, and the Cholesky/PCG/default linear solvers. Edges compute exact Jacobians via dual numbers. |
| [sources/helpers/](sources/helpers/) | Compile-time utilities — type lists, apply/invoke, shared/pmr helpers, traits. |
| [tests/](tests/) | GoogleTest unit and end-to-end tests, including a scalar-generic SLAM fixture. |

> `sources/foundation/` groups the core modeling modules (`dual`, `graph`, `math`, `types`).

---

## How automatic differentiation works

Each derived edge implements **one** scalar-generic residual function, e.g.
[PositionDistanceEdge](sources/optimization/types/position.hpp):

```cpp
struct PositionDistanceEdge
    : go::edge<PositionDistanceEdge, 2, Position, go::Nodes<PositionNode, PositionNode>> {
  using Base::Base;

  template <class A, class B>
  auto error(const Position<A>& a, const Position<B>& b) -> Base::error_vector_type<A, B> {
    return {(b.x - a.x) - this->measurement().x,
            (b.y - a.y) - this->measurement().y};
  }
};
```

- Evaluated with `A = B = double` → the **residual** used to compute `chi²`.
- Evaluated with `A = B = dual::number<double>` → the residual carries its
  **exact partial derivatives**. The optimizer seeds one node's tangent
  increment with independent dual variables and reads the Jacobian directly
  from the dual residual (see `Edge::jacobian()` in [sources/optimization/graph_edge.hpp](sources/optimization/graph_edge.hpp)).

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
#include "vortex.hpp"   // pulls in graph::optimization
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
auto p1 = g.build<PositionNode>(SlamGraph::Key{1});
auto p2 = g.build<PositionNode>(SlamGraph::Key{2});
auto p3 = g.build<PositionNode>(SlamGraph::Key{3});
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

// Optimize (Levenberg–Marquardt); returns std::expected<std::size_t, AlgorithmError>
const auto result = g.optimize(/*iterations=*/10);
if (result) {
  // Converges to p1=(1,1), p2=(2,2), p3=(2,2)
}
```

### Defining your own problem

1. **Node** — subclass `go::Node<Derived, Dim, EstimationType, go::Edges<...>>`
   and implement a scalar-generic `plus(delta)` manifold retraction.
2. **Edge** — subclass `go::edge<Derived, Dim, MeasurementType, go::Nodes<...>>`
   and implement a scalar-generic `error(...)` returning `Base::error_vector_type<T>`.
3. **Graph** — subclass `go::Graph<go::Nodes<...>, go::Edges<...>>`.
4. Build nodes/edges, set estimations & measurements, call `optimize()`.

---

## Configuration

Solver behaviour is selected through a configuration struct (see
[sources/optimization/graph_config.hpp](sources/optimization/graph_config.hpp)).
The defaults are:

| Component | Default |
| --- | --- |
| Scalar `Number` | `double` |
| Algorithm | `LevenbergAlgorithm` |
| Graph solver | `BlockGraphSolver` |
| Linear solver | `DefaultLinearSolver` (also available: `Cholesky`, `PCG`) |
| `system_capacity` | `0x200` |

Provide your own struct deriving from `vortex::optimization::default_config` and
pass it as the third template parameter of `go::Graph` to swap any of these.

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
