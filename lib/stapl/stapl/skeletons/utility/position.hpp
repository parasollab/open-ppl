/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_POSITION_HPP
#define STAPL_SKELETONS_UTILITY_POSITION_HPP

namespace stapl {
namespace skeletons {

//////////////////////////////////////////////////////////////////////
/// @brief Direction type to identify the type of filtering to be used.
/// @li @c direction0 - indicates the direction of the flow is along
///        the first dimension.
/// @li @c direction1 - indicates the direction of the flow is along
///        the second dimension.
/// @li @c direction2 - indicates the direction of the flow is along
///        the third dimension.
/// @li @c direction3 - indicates the direction of the flow is along
///        the fourth dimension.
/// @li @c direction4 - indicates the direction of the flow is along
///        the fifth dimension.
//////////////////////////////////////////////////////////////////////
enum class direction
  : std::size_t {direction0 = 0, direction1, direction2, direction3,
                 direction4};


//////////////////////////////////////////////////////////////////////
/// @brief Corner type used in the wavefront parametric dependency.
/// @li @c first  - indicates that a given point is on the lower boundary
///                 of a given dimension
/// @li @c middle - indicates that a given point is not on the boundaries
///                 of a given dimension
/// @li @c last   - indicates that a given point is on the upper boundary
///                 of a dimension
//////////////////////////////////////////////////////////////////////
enum class position : std::size_t {first, last, middle};

//////////////////////////////////////////////////////////////////////
/// @brief Determines the enum item related to a given position based
/// on the lower and upper bound of that dimension.
///
/// @note The bounds are inclusive.
///
/// @param index       current index being evaluated
/// @param lower_bound the inclusive lower bound of the given dimension
/// @param upper_bound the inclusive upper bound of the given dimension
///
/// @return the corresponding enum values for the given index
//////////////////////////////////////////////////////////////////////
constexpr skeletons::position
find_position(std::size_t index,
              std::size_t lower_bound, std::size_t upper_bound) noexcept
{
  return index == lower_bound ? position::first :
                                (index == upper_bound ? position::last :
                                                        position::middle);
}

//////////////////////////////////////////////////////////////////////
/// @brief Computes the switch case possibilities for a point in
/// the wavefront.
///
/// The enum class has 3 options: first, last, and middle. Two bits
/// are needed to consider all cases.
//////////////////////////////////////////////////////////////////////
constexpr std::size_t condition_value(skeletons::position pos0,
                                      skeletons::position pos1)
{
  return (static_cast<std::size_t>(pos0)) +
         (static_cast<std::size_t>(pos1) << 2);
}

//////////////////////////////////////////////////////////////////////
/// @brief The special case that no filtering function is defined.
///
/// The enum class has 3 options: first, last, and middle. Two bits
/// are needed to consider all cases.
//////////////////////////////////////////////////////////////////////
constexpr std::size_t condition_value(skeletons::position pos0,
                                      skeletons::position pos1,
                                      skeletons::position pos2)
{
  return (static_cast<std::size_t>(pos0)) +
         (static_cast<std::size_t>(pos1) << 2) +
         (static_cast<std::size_t>(pos2) << 4);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_POSITION_hpp
