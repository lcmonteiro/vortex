/// ===============================================================================================
/// @file
/// @brief Unit tests for the graph optimization linear solvers
/// (`DefaultLinearSolver`, `cholesky_linear_solver`).
/// ===============================================================================================
#include <gtest/gtest.h>

#include <cstddef>

#include "foundation/math.hpp"
#include "optimization/linear_solver_cholesky.hpp"
#include "optimization/linear_solver_default.hpp"

namespace {

using Vector = vortex::math::dynamic_vector<double>;
using Matrix = vortex::math::dynamic_matrix<double>;

void ClearUpper(Matrix& matrix) {
  for (std::size_t i = 0; i < matrix.rows(); ++i) {
    for (std::size_t j = i + 1; j < matrix.columns(); ++j) {
      matrix(i, j) = 0;
    }
  }
}

TEST(DefaultLinearSolver, GivenPositiveDefinedMatrixExpectSuccess) {
  // random positive defined matrix
  auto coefs = Matrix{{{10, 1, 2, 1, 0, 0},
                       {1, 11, 1, 3, 1, 0},
                       {2, 1, 12, 2, 1, 1},
                       {1, 3, 2, 13, 2, 1},
                       {0, 1, 1, 2, 14, 2},
                       {0, 0, 1, 1, 2, 15}}};
  // random column Matrix
  auto input = Vector{19, 28, 37, 46, 55, 64};
  auto output = Vector();
  auto result = Vector();
  output = coefs * input;

  ClearUpper(coefs);

  auto solver = vortex::optimization::DefaultLinearSolver{};
  EXPECT_EQ(solver.solve(coefs, output, result), true);
  EXPECT_EQ(input, result);
}

TEST(DefaultLinearSolver, GivenZeroMatrixExpectFailed) {
  // zero matrix
  auto coefs = Matrix{{{0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0}}};
  // random column Matrix
  auto input = Vector{19, 28, 37, 46, 55, 64};
  auto output = Vector();
  auto result = Vector();
  output = coefs * input;

  auto solver = vortex::optimization::DefaultLinearSolver{};
  EXPECT_EQ(solver.solve(coefs, output, result), false);
}

TEST(CholeskyLinearSolver, GivenPositiveDefinedMatrixExpectSuccess) {
  // random positive defined matrix
  auto coefs = Matrix{{{10, 1, 2, 1, 0, 0},
                       {1, 11, 1, 3, 1, 0},
                       {2, 1, 12, 2, 1, 1},
                       {1, 3, 2, 13, 2, 1},
                       {0, 1, 1, 2, 14, 2},
                       {0, 0, 1, 1, 2, 15}}};

  // random column Matrix
  auto input = Vector{19, 28, 37, 46, 55, 64};
  auto output = Vector();
  auto result = Vector();
  output = coefs * input;
  ClearUpper(coefs);

  auto solver = vortex::optimization::cholesky_linear_solver{};
  EXPECT_EQ(solver.solve(coefs, output, result), true);
  EXPECT_EQ(input, result);
}

TEST(CholeskyLinearSolver, GivenZeroMatrixExpectFailed) {
  // zero matrix
  auto coefs = Matrix{{{0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0}}};
  // random column Matrix
  auto input = Vector{19, 28, 37, 46, 55, 64};
  auto output = Vector();
  auto result = Vector();
  output = coefs * input;

  auto solver = vortex::optimization::cholesky_linear_solver{};
  EXPECT_EQ(solver.solve(coefs, output, result), false);
}

}  // namespace
