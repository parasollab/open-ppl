/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_MAP_REDUCE_HPP
#define STAPL_SKELETONS_FUNCTIONAL_MAP_REDUCE_HPP

#include "zip_reduce.hpp"

namespace stapl {
namespace skeletons {
namespace result_of {

template<typename MapOp, typename RedOp>
using map_reduce = result_of::zip_reduce<1, MapOp, RedOp>;

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief As its name implies it creates a map reduce skeleton by
/// piping the result of a @c map skeleton to a @c reduce skeleton.
///
/// @param map_op the operation to be applied in the @c map step
/// @param red_op the operation to be applied in the @c reduce step
///
/// @return a map-reduce skeleton
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename MapOp, typename RedOp>
result_of::map_reduce<MapOp, RedOp>
map_reduce(MapOp&& map_op, RedOp&& red_op)
{
  return skeletons::zip_reduce<1>(
           std::forward<MapOp>(map_op),
           std::forward<RedOp>(red_op));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_MAP_REDUCE_HPP
