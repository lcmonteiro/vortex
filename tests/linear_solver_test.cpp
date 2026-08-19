/// ===============================================================================================
/// @file
/// @brief Unit tests for the graph optimization linear solvers
/// (`default_linear_solver`, `cholesky_linear_solver`).
/// ===============================================================================================
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

#include "foundation/math.hpp"
#include "optimization/linear_solver_cholesky.hpp"
#include "optimization/linear_solver_default.hpp"
#include "optimization/linear_solver_pcg.hpp"

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

  auto solver = vortex::optimization::default_linear_solver{};
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

  auto solver = vortex::optimization::default_linear_solver{};
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

/// ===============================================================================================
/// PCG
///
/// Unlike the two direct solvers, pcg_linear_solver is iterative and declares
/// kRequiresFullMatrix, so it is handed a fully populated symmetric matrix rather than just the
/// lower triangle. It also treats `x` as its starting guess and never resizes it.
/// ===============================================================================================

TEST(PcgLinearSolver, GivenPositiveDefinedMatrixExpectSuccess) {
  // random positive defined matrix, left fully populated on purpose
  auto coefs = Matrix{{{10, 1, 2, 1, 0, 0},
                       {1, 11, 1, 3, 1, 0},
                       {2, 1, 12, 2, 1, 1},
                       {1, 3, 2, 13, 2, 1},
                       {0, 1, 1, 2, 14, 2},
                       {0, 0, 1, 1, 2, 15}}};
  // random column Matrix
  auto input = Vector{19, 28, 37, 46, 55, 64};
  auto output = Vector();
  output = coefs * input;

  // PCG starts from x and never resizes it, so it has to be sized up front.
  auto result = Vector(coefs.rows(), 0.0);

  auto solver = vortex::optimization::pcg_linear_solver<double>{};
  EXPECT_EQ(solver.solve(coefs, output, result), true);

  // PCG stops on a relative reduction of the preconditioned residual r.z, not on an absolute
  // error in x, so it is judged on the residual it actually controls. At the default
  // ResidualExponent of -6 the error in x is around 1e-1 on this system; asserting anything
  // tighter here would be testing the conditioning, not the solver.
  const Vector residual = output - (coefs * result);
  EXPECT_LT(std::sqrt(blaze::dot(residual, residual)),
            1e-2 * std::sqrt(blaze::dot(output, output)));
}

TEST(PcgLinearSolver, GivenATighterResidualLimitExpectAMoreAccurateSolution) {
  auto coefs = Matrix{{{10, 1, 2, 1, 0, 0},
                       {1, 11, 1, 3, 1, 0},
                       {2, 1, 12, 2, 1, 1},
                       {1, 3, 2, 13, 2, 1},
                       {0, 1, 1, 2, 14, 2},
                       {0, 0, 1, 1, 2, 15}}};
  auto input = Vector{19, 28, 37, 46, 55, 64};
  auto output = Vector();
  output = coefs * input;
  auto result = Vector(coefs.rows(), 0.0);

  // Driving ResidualExponent down to -14 takes the solution to machine precision, which is
  // what makes the template parameter worth having.
  auto solver = vortex::optimization::pcg_linear_solver<double, -14>{};
  EXPECT_EQ(solver.solve(coefs, output, result), true);

  for (std::size_t i = 0; i < std::size(input); ++i) {
    EXPECT_NEAR(result[i], input[i], 1e-9) << "component " << i;
  }
}

TEST(PcgLinearSolver, GivenAnIterationBudgetTooSmallToConvergeExpectFailed) {
  auto coefs = Matrix{{{10, 1, 2, 1, 0, 0},
                       {1, 11, 1, 3, 1, 0},
                       {2, 1, 12, 2, 1, 1},
                       {1, 3, 2, 13, 2, 1},
                       {0, 1, 1, 2, 14, 2},
                       {0, 0, 1, 1, 2, 15}}};
  auto input = Vector{19, 28, 37, 46, 55, 64};
  auto output = Vector();
  output = coefs * input;
  auto result = Vector(coefs.rows(), 0.0);

  // One iteration cannot reach a 1e-6 relative residual on this system.
  auto solver = vortex::optimization::pcg_linear_solver<double, -6, 1>{};
  EXPECT_EQ(solver.solve(coefs, output, result), false);
}

TEST(LinearSolvers, ExpectOnlyPcgToRequireTheFullMatrix) {
  // block_graph_solver reads this flag to decide whether to mirror each Hessian block across
  // the diagonal or to accumulate only the lower triangle, so it is part of the solver contract.
  EXPECT_FALSE(vortex::optimization::default_linear_solver::kRequiresFullMatrix);
  EXPECT_FALSE(vortex::optimization::cholesky_linear_solver::kRequiresFullMatrix);
  EXPECT_TRUE(vortex::optimization::pcg_linear_solver<double>::kRequiresFullMatrix);
}

}  // namespace
