/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

/// @todo These wrapper functions are a temporary fix to solve coarsening bug,
/// and should be removed once coarnsening is fixed.

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_NC_MAP_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_NC_MAP_HPP

#include <stapl/skeletons/utility/tags.hpp>

namespace stapl {

/////////////////////////////////////////////////////////////////
/// @brief @ref map_reduce() variant that does not coarsen the input
/// views if size of @p view0 is less than the number of locations.
///
/// @note This is needed to make map-reduce work with views with
/// sizes less than the number of locations until the metadata
/// reported is corrected to exclude empty base containers.
////////////////////////////////////////////////////////////////
template<typename Functor1, typename Functor2, typename View0, typename ...View>
typename Functor2::result_type nc_map_reduce(Functor1 const& func1,
  Functor2 const& func2, View0 const& view0, View const&... view)
{
  if (view0.size() > view0.get_num_locations()) {
    return map_reduce(func1, func2, view0, view...);
  } else {
    return map_reduce<skeletons::tags::no_coarsening>(func1, func2, view0,
      view...);
  }
}


/////////////////////////////////////////////////////////////////
/// @brief @ref map_func() variant that does not coarsen the input
/// views if size of @p view0 is less than the number of locations.
///
/// @note This is needed to make map-func work with views with
/// sizes less than the number of locations until the metadata
/// reported is corrected to exclude empty base containers.
////////////////////////////////////////////////////////////////
template<typename Functor, typename View0, typename ...View>
void nc_map_func(Functor const& func, View0 const& view0, View const&... view)
{
  if (view0.size() > view0.get_num_locations())
    map_func(func, view0, view...);
  else
    map_func<skeletons::tags::no_coarsening>(func, view0, view...);
}


/////////////////////////////////////////////////////////////////
/// @brief @ref accumulate() variant that does not coarsen the input
/// views if size of @p view is less than the number of locations.
///
/// @note This is needed to make accumulate work with views with
/// sizes less than the number of locations until the metadata
/// reported is corrected to exclude empty base containers.
////////////////////////////////////////////////////////////////
template<typename View, typename Oper>
typename View::value_type nc_accumulate(View const& view,
                                        typename View::value_type init,
                                        Oper oper)
{
  using namespace skeletons;

  if (view.size() > view.get_num_locations())
    return oper(init, stapl::reduce(view, oper));
  else
    return oper(init, stapl::reduce<tags::no_coarsening>(view, oper));

}


/////////////////////////////////////////////////////////////////
/// @brief @ref accumulate() variant that does not coarsen the input
/// views if size of @p view is less than the number of locations.
///
/// @note This is needed to make accumulate work with views with
/// sizes less than the number of locations until the metadata
/// reported is corrected to exclude empty base containers.
////////////////////////////////////////////////////////////////
template<typename View>
typename View::value_type nc_accumulate(View const& view,
                                        typename View::value_type init)
{
  return nc_accumulate(view, init, stapl::plus<typename View::value_type>());
}


/////////////////////////////////////////////////////////////////
/// @brief @ref partial_sum() variant that does not coarsen the input
/// views if size of @p view0 is less than the number of locations.
///
/// @note This is needed to make partial-sum work with views with
/// sizes less than the number of locations until the metadata
/// reported is corrected to exclude empty base containers.
////////////////////////////////////////////////////////////////
template<typename View0, typename View1>
void
nc_partial_sum(View0 const& view0, View1 const& view1, bool shift)
{
  using namespace skeletons;
  if (view0.size() > view0.get_num_locations()) {
    stapl::partial_sum(view0, view1, shift);
  } else {
    scan<tags::no_coarsening>(view0, view1,
      stapl::plus<typename View0::value_type>(), shift);
  }
}

/////////////////////////////////////////////////////////////////
/// @brief combination of @ref nc_partial_sum and @ref nc_accumulate
///
/// @note This is needed to make partial_sum_accumulate work with
/// views with sizes less than the number of locations until
/// locations until the metadata reported is corrected to exclude
/// empty base containers.
////////////////////////////////////////////////////////////////
template <typename View0, typename View1>
typename View0::value_type
nc_partial_sum_accumulate(
  View0 const& view0, View1 const& view1, typename View0::value_type init_value,
  bool shift)
{
  using namespace skeletons;
  if (view0.size() > view0.get_num_locations()) {
    return stapl::partial_sum_accumulate(view0, view1, init_value, shift);
  } else {
    return scan_reduce<tags::no_coarsening>(view0, view1,
      stapl::plus<typename View0::value_type>(), shift);
  }
}


}

#endif
