/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_MAP_HPP
#define STAPL_SKELETONS_MAP_HPP

#include <stapl/skeletons/functional/zip.hpp>
#include <stapl/skeletons/transformations/coarse/zip.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/views/metadata/coarseners/null.hpp>
#include <stapl/views/metadata/coarseners/default.hpp>
#include <stapl/views/metadata/coarseners/multiview.hpp>

namespace stapl {

namespace map_func_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Reflects the correct span for a map instantiated with a
///        view set.
//////////////////////////////////////////////////////////////////////
template<typename... Views>
struct select_span
{
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of dimensions of the first view
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename... Ts>
  static constexpr std::size_t first(T&& t, Ts&&...)
  {
    return t;
  }

  using type = skeletons::spans::blocked<
    first(dimension_traits<typename std::decay<Views>::type>::type::value...)
  >;
};

//////////////////////////////////////////////////////////////////////
/// @brief Construct and execute a PARAGRAPH that will perform a
/// fine-grained map operation, applying the fine-grain work function
/// to the elements of the views provided.
///
/// @param map_op Fine-grain map work function.
/// @param view   One or more views to process with the map work function.
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template<typename MapOp, typename ...V>
inline void
map_func(skeletons::tags::with_coarsened_wf, MapOp const& map_op,
         V&&... views)
{
  using span_t = typename select_span<V...>::type;

  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    skeletons::zip<sizeof...(V)>(
      map_op, skeletons::skeleton_traits<span_t>()),
    std::forward<V>(views)...);
}

//////////////////////////////////////////////////////////////////////
/// @brief Construct and execute a PARAGRAPH that will perform a
/// fine-grained map operation, applying the fine-grain work function
/// to the element of the views provided.
///
/// @param map_op Fine-grain map work function.
/// @param views  One or more views to process with the map work function.
///
/// @ingroup skeletonsExecutableInternal
//////////////////////////////////////////////////////////////////////
template<typename MapOp, typename ...V>
inline void
map_func(skeletons::tags::no_coarsening, MapOp const& map_op, V&&... views)
{
  using span_t = typename select_span<V...>::type;

  skeletons::execute(
    skeletons::default_execution_params(),
    skeletons::zip<sizeof...(V)>(
      map_op, skeletons::skeleton_traits<span_t>()),

    std::forward<V>(views)...);
}

}

//////////////////////////////////////////////////////////////////////
/// @brief Construct and execute a PARAGRAPH that will perform a map operation,
/// applying the fine-grain work function to the element of the views
/// provided.
///
/// @param map_op Fine-grain map work function.
/// @param views  One or more views to process with the map work function.
///
/// @ingroup skeletonsExecutable
//////////////////////////////////////////////////////////////////////
template<typename MapOp, typename ...V>
inline void
map_func(MapOp const& map_op, V&&... views)
{
  using span_t = typename map_func_impl::select_span<V...>::type;

  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    skeletons::coarse(skeletons::zip<sizeof...(V)>(
      map_op, skeletons::skeleton_traits<span_t>())),
    std::forward<V>(views)...);
}


template<typename Tag, typename MapOp, typename ...V>
inline void
map_func(MapOp const& map_op, V&&... views)
{
  map_func_impl::map_func(Tag(), map_op, std::forward<V>(views)...);
}

} // namespace stapl

#endif // STAPL_SKELETONS_MAP_HPP
