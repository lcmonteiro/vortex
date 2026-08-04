/// ===========================================================================
/// @file
/// @brief Minimal `expected<T, E>` / `unexpected<E>`, standing in for
/// `std::expected` / `std::unexpected`.
///
///
/// @note Unlike `std::expected`, `value()`/`error()` do not throw on misuse;
/// calling `value()` without first checking `has_value()` (or vice versa) is
/// undefined behavior, same as dereferencing an empty `std::optional`. Every
/// call site in this codebase already checks before accessing either.
/// ===========================================================================
#ifndef VORTEX_HELPERS_EXPECTED_HPP
#define VORTEX_HELPERS_EXPECTED_HPP

#include <optional>
#include <utility>

namespace vortex::helpers {

/// @brief Wraps an error value so it can be distinguished from a success
/// value at `expected`'s construction site (mirrors `std::unexpected`).
template <class E>
struct unexpected {
  explicit unexpected(E error) : error_{std::move(error)} {}

  auto error() & -> E& { return error_; }
  auto error() const& -> const E& { return error_; }
  auto error() && -> E&& { return std::move(error_); }

  E error_;
};

template <class E>
unexpected(E) -> unexpected<E>;

/// @brief Holds either a `T` or an `E`, mirroring the subset of
/// `std::expected<T, E>` this codebase actually uses. See file-level comment
/// for the `void` specialization and rationale.
template <class T, class E>
class expected {
 public:
  expected(T value) : value_{std::move(value)} {}
  expected(unexpected<E> error) : error_{std::move(error).error()} {}

  auto has_value() const noexcept -> bool { return value_.has_value(); }
  explicit operator bool() const noexcept { return has_value(); }

  auto value() & -> T& { return *value_; }
  auto value() const& -> const T& { return *value_; }
  auto value() && -> T&& { return std::move(*value_); }

  auto error() & -> E& { return *error_; }
  auto error() const& -> const E& { return *error_; }
  auto error() && -> E&& { return std::move(*error_); }

 private:
  std::optional<T> value_;
  std::optional<E> error_;
};

/// @brief `expected<void, E>` specialization: success carries no payload, so
/// there's nothing to store but the error (`std::optional<void>` is
/// ill-formed, hence this can't share the primary template's body).
template <class E>
class expected<void, E> {
 public:
  expected() = default;
  expected(unexpected<E> error) : error_{std::move(error).error()} {}

  auto has_value() const noexcept -> bool { return not error_.has_value(); }
  explicit operator bool() const noexcept { return has_value(); }

  void value() const { /* no payload; presence already asserted by the caller via has_value() */ }

  auto error() & -> E& { return *error_; }
  auto error() const& -> const E& { return *error_; }
  auto error() && -> E&& { return std::move(*error_); }

 private:
  std::optional<E> error_;
};

}  // namespace vortex::helpers

#endif  // VORTEX_HELPERS_EXPECTED_HPP
