/// ===============================================================================================
/// @file
///
/// @brief vortex.helpers.indices component
///
/// Short spellings of the standard index-sequence types, for the pack-expansion idiom where a
/// sequence is generated and immediately destructured by a lambda or an overload.
/// ===============================================================================================
#ifndef VORTEX_HELPERS_INDICES_HPP
#define VORTEX_HELPERS_INDICES_HPP

#include <cstddef>
#include <utility>

namespace vortex::helpers {

/// @brief Creates a compile-time sequence of indices from 0 to N-1.
/// @tparam N The number of indices in the sequence.
template <std::size_t N>
using make_indices = std::make_index_sequence<N>;

/// @brief A compile-time sequence of the given indices, for deducing a pack from make_indices.
/// @tparam I The indices carried by the sequence.
template <std::size_t... I>
using indices = std::index_sequence<I...>;

}  // namespace vortex::helpers

#endif  // VORTEX_HELPERS_INDICES_HPP
