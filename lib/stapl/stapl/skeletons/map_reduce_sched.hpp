/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_MAP_REDUCE_SCHED_HPP
#define STAPL_SKELETONS_MAP_REDUCE_SCHED_HPP

#include <stapl/utility/use_default.hpp>
#include <stapl/skeletons/functional/zip.hpp>
#include <stapl/skeletons/transformations/coarse/zip.hpp>
#include <stapl/views/metadata/coarseners/all_but_last.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Construct and execute a PARAGRAPH that will perform a map
/// operation, applying the fine-grain work function to the element of
/// the views provided.  The scheduler provided will be used by the
/// executor processing the tasks of the PARAGRAPH as it is executed.
///
/// @param scheduler Scheduler to employ in the executor processing the
///                  PARAGRAPH.
/// @param map_op    Fine-grain map work function.
/// @param view      One or more views to process with the map work function.
///
/// @ingroup skeletonsExecutable
//////////////////////////////////////////////////////////////////////
template<typename Scheduler, typename MapOp, typename ReduceOp, typename ...V>
inline
typename
stapl::result_of::map_reduce<stapl::use_default, MapOp, ReduceOp, V...>::type
map_reduce_sched(Scheduler const& scheduler, MapOp const& map_op,
    ReduceOp const& reduce_op, V&&... v)
{
  typedef typename
    stapl::result_of::map_reduce<
      stapl::use_default, MapOp, ReduceOp, V...>::type val_t;

  return map_reduce_helper::map_reduce_impl<val_t, default_coarsener>(
           skeletons::coarse(
             skeletons::zip_reduce<sizeof...(V)>(map_op, reduce_op)
           ),
           scheduler,
           std::forward<V>(v)...);
}

} // namespace stapl

#endif // STAPL_SKELETONS_MAP_REDUCE_SCHED_HPP
