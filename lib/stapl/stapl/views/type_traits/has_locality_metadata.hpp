/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TYPE_TRAITS_HAS_LOCALITY_METADATA_HPP
#define STAPL_VIEWS_TYPE_TRAITS_HAS_LOCALITY_METADATA_HPP

#include <boost/mpl/has_xxx.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Type checker to determine if a given object provides a
///        functor to extract the locality metadata associated with
///        the object.
//////////////////////////////////////////////////////////////////////
BOOST_MPL_HAS_XXX_TRAIT_DEF(loc_dist_metadata)

} // namespace detail

} //namespace stapl

#endif /* STAPL_VIEWS_TYPE_TRAITS_HAS_LOCALITY_METADATA_HPP */
