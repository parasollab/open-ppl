/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_FAST_VIEW_HPP
#define STAPL_PARAGRAPH_FAST_VIEW_HPP

#include <boost/mpl/has_xxx.hpp>

namespace stapl {

BOOST_MPL_HAS_XXX_TRAIT_DEF(fast_view_type)

//////////////////////////////////////////////////////////////////////
/// @brief Computes view type used if view localization (i.e. is_local())
///   succeeds during PARAGRAPH task creation.
/// @ingroup pgViewOps
///
/// @param View Type of view passed to PARAGRAPH at task creation (@p add_task)
///
/// Reflects @p View::fast_view_type if definition exists.  Otherwise,
/// @p View itself is returned (signifying no localization transformation
/// exists to apply).
//////////////////////////////////////////////////////////////////////
template <typename View, bool = has_fast_view_type<View>::value>
struct get_fast_view_type
{
  typedef typename View::fast_view_type type;
};


template <typename View>
struct get_fast_view_type<View, false>
{
  typedef View type;
};

} // namespace stapl

#endif // STAPL_PARAGRAPH_FAST_VIEW_HPP
