/// ===========================================================================
/// @file
///
/// @brief vortex.helpers.contracts component
///
/// Runtime contract checks used across the vortex helpers.
/// ===========================================================================
#ifndef VORTEX_HELPERS_CONTRACTS_HPP
#define VORTEX_HELPERS_CONTRACTS_HPP

#include <cassert>

/// @brief Runtime precondition check.
/// Replaces the former `VORTEX_PRECONDITION` macro. In debug builds it
/// behaves like an assertion; the message is kept for readability.
#define VORTEX_PRECONDITION(condition, message) assert((condition) && (message))

/// @brief Runtime invariant check, for conditions other than preconditions.
#define VORTEX_ASSERT(condition, message) assert((condition) && (message))

#endif  // VORTEX_HELPERS_CONTRACTS_HPP
