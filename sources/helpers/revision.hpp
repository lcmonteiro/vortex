/// ===========================================================================
/// @file
///
/// @brief vortex.helpers.revision component
/// ===========================================================================
#ifndef VORTEX_HELPERS_REVISION_HPP
#define VORTEX_HELPERS_REVISION_HPP
#include <cstddef>
namespace vortex::helpers {

/// @brief This struct is used to manage revision numbers for objects.
struct Revision {
  Revision() = default;
  ~Revision() = default;
  Revision(const Revision&) = delete;
  Revision(Revision&&) = default;

  auto operator=(const Revision&) -> Revision& = delete;
  auto operator=(Revision&&) -> Revision& = default;

  /// @brief Increments the revision number by one.
  /// This function should be called whenever the state of the associated
  /// object changes and a new revision is created.
  auto update() -> void { ++revision_; }

  /// @brief Resets the revision number to match another `Revision` object.
  /// This function can be used to synchronize the revision state with
  /// another object.
  /// @param other The `Revision` object whose revision number will be copied.
  auto reset(const Revision& other) -> void { revision_ = other.revision_; }

  /// @brief Checks if the revision number is equal to that of another
  /// `Revision`.
  /// @param other The `Revision` object to compare with.
  /// @return `true` if the revision numbers are equal, `false` otherwise.
  auto equal(const Revision& other) const -> bool { return revision_ == other.revision_; }

 private:
  size_t revision_;
};
}  // namespace vortex::helpers
#endif  // VORTEX_HELPERS_REVISION_HPP
