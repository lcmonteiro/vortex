/// ===========================================================================
/// @file
///
/// @brief vortex.types.vector_set component
/// ===========================================================================
#ifndef VORTEX_TYPES_VECTOR_SET_HPP
#define VORTEX_TYPES_VECTOR_SET_HPP
#include <algorithm>
#include <iterator>
#include <utility>
#include <vector>

#include "helpers/contracts.hpp"

namespace vortex::graph {

/// @brief Unique-element container that iterates in insertion order.
///
/// Drop-in replacement for the ordered @c std::pmr::set that the graph engine
/// uses for its edge containers (@c Config::Set). Iteration follows insertion
/// order, which is deterministic and independent of the memory addresses of the
/// stored elements. This removes the address-based ordering of @c
/// std::pmr::set, which is not reproducible when the backing memory resource is
/// exceeded.
///
/// @tparam Value The element type stored in the set (e.g. @c Shared<Edge>).
template <typename Value>
class VectorSet : private std::pmr::vector<Value> {
  using Base = std::pmr::vector<Value>;

 public:
  /// @brief @c key_type must equal @c value_type so the engine's set-like trait
  /// detection treats this container as a set and not a map.
  using value_type = Value;
  using key_type = Value;
  using size_type = typename Base::size_type;
  using iterator = typename Base::iterator;
  using const_iterator = typename Base::const_iterator;

  /// @brief Inherit the base (memory-resource / allocator) constructors.
  using Base::Base;

  /// @brief Re-expose the base operations the engine uses directly (the base
  /// by-iterator and by-range @c erase are valid because @c Value is
  /// move-assignable).
  using Base::begin;
  using Base::clear;
  using Base::end;
  using Base::erase;
  using Base::size;

  /// @brief Inserts a value at the end, preserving insertion order.
  /// @param value The value to insert.
  /// @return A pair mirroring @c std::set::insert; the bool is always true (the
  /// caller guarantees uniqueness).
  auto insert(const value_type& value) -> std::pair<iterator, bool> {
    VORTEX_PRECONDITION(find(value) == Base::end(), "value already present in set");
    Base::push_back(value);
    return {std::prev(Base::end()), true};
  }

  /// @brief Appends the range [first, last), preserving order.
  /// @tparam InputIt Input iterator type.
  /// @param first Begin of the range to append.
  /// @param last End of the range to append.
  template <class InputIt>
  auto insert(InputIt first, InputIt last) -> void {
    std::for_each(first, last, [this](const value_type& value) { std::ignore = insert(value); });
  }

  /// @brief Constructs a value in place at the end, preserving insertion order.
  /// @tparam Args Constructor argument types for @c value_type.
  /// @param args Constructor arguments forwarded to @c value_type.
  /// @return A pair mirroring @c std::set::emplace; the bool is always true.
  template <class... Args>
  auto emplace(Args&&... args) -> std::pair<iterator, bool> {
    std::ignore = Base::emplace_back(std::forward<Args>(args)...);
    return {std::prev(Base::end()), true};
  }

  /// @brief Erases the element equal to @p value, if present.
  /// @param value The value to erase.
  /// @return The number of elements removed (0 or 1), mirroring @c std::set.
  /// @note Survivors are relocated with move-assignment (order-preserving);
  /// relocating @c Shared by move steals its pointer, avoiding the atomic
  /// reference-count churn of a copy-shift.
  auto erase(const value_type& value) -> size_type {
    const auto last = std::remove(Base::begin(), Base::end(), value);
    const auto removed = std::distance(last, Base::end());
    std::ignore = Base::erase(last, Base::end());
    return static_cast<size_type>(removed);
  }

  /// @brief Finds the element equal to @p value.
  /// @param value The value to search for.
  /// @return An iterator to the element, or @c end() if not present.
  auto find(const value_type& value) -> iterator { return std::find(Base::begin(), Base::end(), value); }

  /// @brief Swaps contents with another set.
  /// @param other The set to swap with.
  auto swap(VectorSet& other) -> void { Base::swap(other); }
};

}  // namespace vortex::graph

#endif  // VORTEX_TYPES_VECTOR_SET_HPP
