/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_WF_ITER_COMPARE_HPP
#define STAPL_SKELETONS_UTILITY_WF_ITER_COMPARE_HPP

#include <type_traits>
#include <stapl/views/metadata/infinite_helpers.hpp>
#include <stapl/paragraph/view_operations/subview_type.hpp>
#include <stapl/utility/pack_ops.hpp>

namespace stapl {

/////////////////////////////////////////////////////////////////////
/// @brief Helper class template that performs iterator comparison against
/// an end iterator.  The iterators are selected from a single view of
/// a list of input views.  The first view with a finite domain is
/// selected for use in the comparison.
///
/// @ingroup skeletonsUtilities
/////////////////////////////////////////////////////////////////////
template <typename... V>
class wf_iter_compare
{
private:
  typedef typename first_finite_domain_index<tuple<V...>>::type   index_t;
  typedef typename std::remove_reference<
    typename pack_ops::pack_element<index_t::value, V...>::type
  >::type::const_iterator                                         cond_iter_t;

  const cond_iter_t m_end_iter;

public:
  explicit wf_iter_compare(V const&... v, bool check_end = false)
    : m_end_iter( pack_ops::get<index_t::value>(v...).end() )
  {
    if (check_end)
    {
      stapl_assert(
        cond_iter_t(pack_ops::get<index_t::value>(v...).begin())
          != m_end_iter,
        "wf_iter_compare encountered empty view"
      );
    }
  }

  bool operator()(
    typename std::remove_reference<V>::type::const_iterator const&... iter)
    const
  {
    return m_end_iter != pack_ops::get<index_t::value>(iter...);
  }
};


/////////////////////////////////////////////////////////////////////
/// @brief Helper class template that dispatches call to
/// @p get_num_local_subviews to the first view parameter that has
/// a finite domain.
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template <typename... V>
class part_id_size
{
private:
   typedef typename first_finite_domain_index<
      tuple<typename paragraph_impl::subview_type<V>::type...>>::type index_t;

public:
   static size_t invoke(V const&... v)
   {
     return pack_ops::get<index_t::value>(v...).get_num_local_subviews();
   }
};

} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_WF_ITER_COMPARE_HPP
