/// ===============================================================================================
/// @file
///
/// @brief vortex.helpers.revision component
/// ===============================================================================================
#ifndef VORTEX_HELPERS_REVISION_HPP
#define VORTEX_HELPERS_REVISION_HPP
#include <cstddef>
namespace vortex::helpers {

/// @brief This struct is used to manage revision numbers for objects.
class revision {
 public:
  /// @brief Default constructor initializes the revision number to zero.
  revision() = default;
  ~revision() = default;
  revision(const revision&) = delete;
  revision(revision&&) = default;

  /// @brief Assignment operators are deleted to prevent copying of revision
  auto operator=(const revision&) -> revision& = delete;
  auto operator=(revision&&) -> revision& = default;

  /// @brief Increments the revision number by one.
  /// This function should be called whenever the state of the associated
  /// object changes and a new revision is created.
  auto update() -> void { ++revision_; }

  /// @brief Resets the revision number to match another `revision` object.
  /// This function can be used to synchronize the revision state with
  /// another object.
  /// @param other The `revision` object whose revision number will be copied.
  auto reset(const revision& other) -> void { revision_ = other.revision_; }

  /// @brief Checks if the revision number is equal to that of another
  /// `revision`.
  /// @param other The `revision` object to compare with.
  /// @return `true` if the revision numbers are equal, `false` otherwise.
  auto equal(const revision& other) const -> bool { return revision_ == other.revision_; }

 private:
  std::size_t revision_;
};
}  // namespace vortex::helpers
#endif  // VORTEX_HELPERS_REVISION_HPP
