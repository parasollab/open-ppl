/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MULTIARRAY_TRAVERSALS_HPP
#define STAPL_CONTAINERS_MULTIARRAY_TRAVERSALS_HPP

#include <boost/mpl/int.hpp>

#include <stapl/utility/tuple.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Typedef for 2D row-major traversal for a @ref multiarray.
/// @ingroup pmultiarrayManip
//////////////////////////////////////////////////////////////////////
typedef tuple<
  boost::mpl::int_<1>,
  boost::mpl::int_<0>
>                         row_major;

//////////////////////////////////////////////////////////////////////
/// @brief Typedef for 2D column-major traversal for a @ref multiarray.
/// @ingroup pmultiarrayManip
//////////////////////////////////////////////////////////////////////
typedef tuple<
  boost::mpl::int_<0>,
  boost::mpl::int_<1>
>                         column_major;

//////////////////////////////////////////////////////////////////////
/// @brief Typedef for 3D traversal for a @ref multiarray, where the
/// X-Y plane is traversed first in the 2D row-major order, then
/// the Z-component is traversed.
/// @ingroup pmultiarrayManip
//////////////////////////////////////////////////////////////////////
typedef tuple<
  boost::mpl::int_<2>,
  boost::mpl::int_<1>,
  boost::mpl::int_<0>
>                         xyplane_major;


//////////////////////////////////////////////////////////////////////
/// @brief Typedef for 3D traversal for a @ref multiarray, where the
/// X-Z plane is traversed first in the 2D X-major order, then
/// the Y-component is traversed.
/// @ingroup pmultiarrayManip
//////////////////////////////////////////////////////////////////////
typedef tuple<
  boost::mpl::int_<2>,
  boost::mpl::int_<0>,
  boost::mpl::int_<1>
>                         xzplane_major;

} // namespace stapl

#endif
