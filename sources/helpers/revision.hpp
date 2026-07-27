/// ===========================================================================
/// @file
/// @copyright Copyright (C) 2024, Bayerische Motoren Werke Aktiengesellschaft
/// (BMW AG)
///
/// @brief vortex.helpers.revision component
/// ===========================================================================
#ifndef VORTEX_HELPERS_REVISION_HPP
#define VORTEX_HELPERS_REVISION_HPP
#include <cstddef>
namespace vortex::graph {
namespace detail {

/// Revision Type
/// @brief This struct is used to manage revision numbers for objects.
struct Revision {
  Revision() = default;
  ~Revision() = default;
  Revision(const Revision&) = delete;
  Revision& operator=(const Revision&) = delete;
  Revision(Revision&&) = default;
  Revision& operator=(Revision&&) = default;

  /// @brief Increments the revision number by one.
  /// This function should be called whenever the state of the associated
  /// object changes and a new revision is created.
  void update() { ++revision_; }

  /// @brief Resets the revision number to match another `Revision` object.
  /// This function can be used to synchronize the revision state with
  /// another object.
  /// @param other The `Revision` object whose revision number will be copied.
  void reset(const Revision& other) { revision_ = other.revision_; }

  /// @brief Checks if the revision number is equal to that of another
  /// `Revision`.
  /// @param other The `Revision` object to compare with.
  /// @return `true` if the revision numbers are equal, `false` otherwise.
  bool equal(const Revision& other) const { return revision_ == other.revision_; }

 private:
  size_t revision_;
};
}  // namespace detail
}  // namespace vortex::graph
#endif  // VORTEX_HELPERS_REVISION_HPP
