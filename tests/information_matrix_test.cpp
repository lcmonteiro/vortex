/// ===============================================================================================
/// @file
/// @brief Unit tests for the edge information matrix variants (identity,
/// diagonal, symmetric) in `optimization::variants::InformationVariant`.
/// ===============================================================================================
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory_resource>

#include "foundation/math.hpp"
#include "optimization/graph.hpp"

namespace {

namespace go = vortex::graph::optimization;
namespace math = vortex::math;

/// @brief Dimension for Nodes and Edges.
constexpr std::uint8_t kDimension = 2;

// Edges forward declaration
struct EdgeWithIdentity;
struct EdgeWithDiagonal;
struct EdgeWithSymmetric;
using Edges = go::Edges<EdgeWithIdentity, EdgeWithDiagonal, EdgeWithSymmetric>;

// Nodes Definition
using EstimationType = std::uint8_t;

template <class Derived>
using Node1Base = go::Node<Derived, kDimension, EstimationType, Edges>;

struct Node1 : Node1Base<Node1> {
  using Base = Node1Base<Node1>;
  using Base::Base;
};

// Edges Definition
using MeasurementType = std::uint8_t;

template <class Derived>
using EdgeWithIdentityBase =
    go::Edge<Derived, kDimension, MeasurementType, go::Nodes<Node1, Node1>>;

struct EdgeWithIdentity : EdgeWithIdentityBase<EdgeWithIdentity> {
  using Base = EdgeWithIdentityBase<EdgeWithIdentity>;
  using Base::Base;

  /// @brief Information matrix type
  static constexpr std::size_t kInformation = go::variants::kIdentityMatrix;
};

template <class Derived>
using EdgeWithDiagonalBase =
    go::Edge<Derived, kDimension, MeasurementType, go::Nodes<Node1, Node1>>;

struct EdgeWithDiagonal : EdgeWithDiagonalBase<EdgeWithDiagonal> {
  using Base = EdgeWithDiagonalBase<EdgeWithDiagonal>;
  using Base::Base;

  /// @brief Information matrix type
  static constexpr std::size_t kInformation = go::variants::kDiagonalMatrix;
};

template <class Derived>
using EdgeWithSymmetricBase =
    go::Edge<Derived, kDimension, MeasurementType, go::Nodes<Node1, Node1>>;

struct EdgeWithSymmetric : EdgeWithSymmetricBase<EdgeWithSymmetric> {
  using Base = EdgeWithSymmetricBase<EdgeWithSymmetric>;
  using Base::Base;

  /// @brief Information matrix type
  static constexpr std::size_t kInformation = go::variants::kSymmetricMatrix;
};

// Graph Definition
struct Graph : go::Graph<go::Nodes<Node1>, Edges> {
  using Base = go::Graph<go::Nodes<Node1>, Edges>;
  using Base::Base;
};

struct InformationMatrixTestFixture : public ::testing::Test {
 protected:
  void TearDown() override { g_.destroy(); }

  Graph g_{std::pmr::new_delete_resource()};
};

TEST_F(InformationMatrixTestFixture, InformationMatrix_Identity_Get) {
  // Given
  auto n1 = g_.build<Node1>(Graph::Key{1});
  auto n2 = g_.build<Node1>(Graph::Key{2});
  auto edge_with_identity = g_.build<EdgeWithIdentity>(n1, n2);

  // When
  auto& information = edge_with_identity->information();

  // Then
  math::DiagonalMatrix<EdgeWithIdentity::Matrix> expected_information{};
  math::diagonal(expected_information) = math::StaticVector{1, 1};

  EXPECT_EQ(information, expected_information);
}

TEST_F(InformationMatrixTestFixture, InformationMatrix_Diagonal_GetInitial) {
  // Given
  auto n1 = g_.build<Node1>(Graph::Key{1});
  auto n2 = g_.build<Node1>(Graph::Key{2});
  auto edge_with_diagonal = g_.build<EdgeWithDiagonal>(n1, n2);

  // When
  auto& information = edge_with_diagonal->information();

  // Then
  math::DiagonalMatrix<EdgeWithDiagonal::Matrix> expected_information{};
  math::diagonal(expected_information) = math::StaticVector{1, 1};

  EXPECT_EQ(information, expected_information);
}

TEST_F(InformationMatrixTestFixture, InformationMatrix_Diagonal_SetWithMatrix) {
  // Given
  auto n1 = g_.build<Node1>(Graph::Key{1});
  auto n2 = g_.build<Node1>(Graph::Key{2});
  auto edge_with_diagonal = g_.build<EdgeWithDiagonal>(n1, n2);

  math::DiagonalMatrix<EdgeWithDiagonal::Matrix> new_information{};
  math::diagonal(new_information) = math::StaticVector{2, 2};

  // When
  edge_with_diagonal->information(new_information);

  // Then
  auto& retrieved_information = edge_with_diagonal->information();

  EXPECT_EQ(new_information, retrieved_information);
}

TEST_F(InformationMatrixTestFixture, InformationMatrix_Diagonal_SetWithVector) {
  // Given
  auto n1 = g_.build<Node1>(Graph::Key{1});
  auto n2 = g_.build<Node1>(Graph::Key{2});
  auto edge_with_diagonal = g_.build<EdgeWithDiagonal>(n1, n2);

  math::StaticVector new_information_vector{2, 3};

  // When
  edge_with_diagonal->information(new_information_vector);

  // Then
  math::DiagonalMatrix<EdgeWithDiagonal::Matrix> expected_information_matrix{};
  math::diagonal(expected_information_matrix) = new_information_vector;

  auto& retrieved_information_matrix = edge_with_diagonal->information();

  EXPECT_EQ(expected_information_matrix, retrieved_information_matrix);
}

TEST_F(InformationMatrixTestFixture, InformationMatrix_Diagonal_SetWithValues) {
  // Given
  auto n1 = g_.build<Node1>(Graph::Key{1});
  auto n2 = g_.build<Node1>(Graph::Key{2});
  auto edge_with_diagonal = g_.build<EdgeWithDiagonal>(n1, n2);

  math::StaticVector new_information_vector{2.0, 3.0};

  // When
  edge_with_diagonal->information(2.0, 3.0);

  // Then
  math::DiagonalMatrix<EdgeWithDiagonal::Matrix> expected_information_matrix{};
  math::diagonal(expected_information_matrix) = new_information_vector;

  auto& retrieved_information_matrix = edge_with_diagonal->information();

  EXPECT_EQ(expected_information_matrix, retrieved_information_matrix);
}

TEST_F(InformationMatrixTestFixture, InformationMatrix_Symmetric_Get) {
  // Given
  auto n1 = g_.build<Node1>(Graph::Key{1});
  auto n2 = g_.build<Node1>(Graph::Key{2});
  auto edge_with_diagonal = g_.build<EdgeWithSymmetric>(n1, n2);

  EdgeWithSymmetric::Matrix expected_information{{1, 0}, {0, 1}};

  // When
  auto& retrieved_information = edge_with_diagonal->information();

  // Then
  EXPECT_EQ(expected_information, retrieved_information);
}

TEST_F(InformationMatrixTestFixture, InformationMatrix_Symmetric_Set) {
  // Given
  auto n1 = g_.build<Node1>(Graph::Key{1});
  auto n2 = g_.build<Node1>(Graph::Key{2});
  auto edge_with_diagonal = g_.build<EdgeWithSymmetric>(n1, n2);

  math::SymmetricMatrix<EdgeWithSymmetric::Matrix> new_information{{1, 2}, {2, 1}};

  // When
  edge_with_diagonal->information(new_information);

  // Then
  auto& retrieved_information = edge_with_diagonal->information();

  EXPECT_EQ(new_information, retrieved_information);
}

}  // namespace
