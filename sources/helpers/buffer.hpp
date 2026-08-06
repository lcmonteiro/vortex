/// ===========================================================================
/// @file
///
/// @brief vortex.helpers.buffer component
/// ===========================================================================
#ifndef VORTEX_HELPERS_BUFFER_HPP
#define VORTEX_HELPERS_BUFFER_HPP

#include <array>
#include <cstddef>
#include <type_traits>
#include <utility>

namespace vortex::helpers {

/// @brief Fixed-capacity ring buffer that retains the most recent @p Capacity
/// elements.
///
/// Elements are appended at the "head" (newest) end. Once the buffer is full,
/// each new element evicts the oldest one, so the buffer always holds at most
/// @p Capacity elements. Access is newest-oriented: back() returns the most
/// recently pushed element and pop() removes it, which makes the container
/// convenient as a bounded history / undo stack.
///
/// Single-threaded only — no locking, no atomics.
///
/// @tparam T Element type.
/// Must be default-constructible because the backing store is std::array<T,Capacity>, 
/// which value-initializes every slot up front.
/// @tparam Capacity Maximum number of retained elements. Must be greater than 0.
template <class T, std::size_t Capacity>
class buffer {
  static_assert(Capacity > 0, "Capacity must be greater than 0");
  static_assert(std::is_default_constructible_v<T>,
                "buffer<T, Capacity> requires T to be default-constructible "
                "because storage is std::array<T, Capacity>; use a raw-storage "
                "implementation if T has no default constructor.");

 public:
  /// @brief Unsigned type used for sizes and internal indices.
  using size_type = std::size_t;

  /// @brief Default constructor, copy/move constructors, and assignment operators.
  buffer() = default;
  buffer(const buffer&) = default;
  buffer(buffer&&) = default;
  auto operator=(const buffer&) -> buffer& = default;
  auto operator=(buffer&&) -> buffer& = default;
  ~buffer() = default;

  /// @brief Checks whether the buffer holds no elements.
  /// @return `true` if the buffer is empty, `false` otherwise.
  auto empty() const noexcept -> bool { return size_ == 0; }

  /// @brief Returns the number of elements currently retained.
  /// @return The element count, always in the range [0, Capacity].
  auto size() const noexcept -> size_type { return size_; }

  /// @brief Constructs a new element at the head of the buffer
  /// element if the buffer is already full.
  /// @tparam Args Constructor argument types for T.
  /// @param args Arguments forwarded to T's constructor.
  template <class... Args>
  auto push(Args&&... args) -> void {
    data_[head_] = T(std::forward<Args>(args)...);
    advance_head();
  }

  /// @brief Removes the newest element. Does nothing if the buffer is empty.
  auto pop() -> void {
    if (not empty()) {
      head_ = prev(head_);
      --size_;
    }
  }

  /// @brief Accesses the newest (most recently pushed) element.
  /// @return A const reference to the newest element.
  /// @note The behaviour is undefined if the buffer is empty.
  auto back() const noexcept -> const T& { return data_[prev(head_)]; }

 protected:
  /// @brief Compile-time constant `1` in the buffer's size type.
  static constexpr auto one = size_type{1};

  /// @brief Returns the ring index immediately following @p index.
  /// @param index A valid ring index.
  /// @return `(index + 1) % Capacity`.
  static constexpr auto next(size_type index) noexcept -> size_type {
    return (index + one) % Capacity;
  }

  /// @brief Returns the ring index immediately preceding @p index.
  /// @param index A valid ring index.
  /// @return `(index + Capacity - 1) % Capacity`.
  static constexpr auto prev(size_type index) noexcept -> size_type {
    return (index + Capacity - one) % Capacity;
  }

  /// @brief Advances the head after a push, growing the size or, when the
  /// buffer is full, evicting the oldest element by advancing the tail.
  auto advance_head() noexcept -> void {
    head_ = next(head_);
    if (size_ == Capacity) {
      tail_ = next(tail_);
    } else {
      ++size_;
    }
  }

 private:
  std::array<T, Capacity> data_{};
  size_type head_ = 0;  // index where the next push will write (one past newest)
  size_type tail_ = 0;  // index of the oldest live element
  size_type size_ = 0;
};

}  // namespace vortex::helpers

#endif  // VORTEX_HELPERS_BUFFER_HPP
