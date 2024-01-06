/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_TASK_CREATION_H
#define STAPL_PARAGRAPH_TASK_CREATION_H

#include <stapl/paragraph/view_operations/subview_type.hpp>
#include <stapl/paragraph/view_operations/fast_view.hpp>
#include <stapl/paragraph/view_operations/localization.hpp>
#include <stapl/paragraph/tasks/return_promotion.hpp>
#include <boost/mpl/has_xxx.hpp>

namespace stapl {

namespace detail {

BOOST_MPL_HAS_XXX_TRAIT_DEF(deferred_localizable)

//////////////////////////////////////////////////////////////////////
/// @brief Checks single view paramter to determine if it has marked
/// itself as deferred localizable.
//////////////////////////////////////////////////////////////////////
template<typename ViewParam>
using is_deferred_localizable =
  std::integral_constant<bool,
    has_deferred_localizable<
      typename get_fast_view_type<
        typename std::decay<ViewParam>::type
      >::type
    >::value
  >;


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction which computes the number of views in the variadic
/// type pack which have signalled via type reflection that they are
/// deferred localizable.
//////////////////////////////////////////////////////////////////////
template<typename ...Elements>
struct count_deferred_localizable
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Recursive case when there's at least one element remaining in list.
//////////////////////////////////////////////////////////////////////
template<typename Head, typename... Tail>
struct count_deferred_localizable<Head, Tail...>
  : public std::integral_constant<
      size_t,
      is_deferred_localizable<Head>::value ? 1 : 0
        + count_deferred_localizable<Tail...>::value
    >
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Base case of recursion for metafunction.
//////////////////////////////////////////////////////////////////////
template<>
struct count_deferred_localizable<>
  : std::integral_constant<size_t, 0>
{ };


template<typename SchedulerEntry,
         typename Migratable,
         typename Persistent,
         typename SchedulerInfoParam,
         typename WFParam,
         typename ...ViewParams>
paragraph_impl::task_base_intermediate<SchedulerEntry>*
create_task(detail::edge_local_notifier_base*,
            tg_callback const&,
            detail::edge_entry_base*, SchedulerInfoParam&&,
            WFParam&&, ViewParams&&...);

} // namespace detail

} // namespace stapl

#endif // STAPL_PARAGRAPH_TASK_CREATION_H
