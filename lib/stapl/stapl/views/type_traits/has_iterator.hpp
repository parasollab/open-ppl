/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TYPE_TRAITS_HAS_ITERATOR_HPP
#define STAPL_VIEWS_TYPE_TRAITS_HAS_ITERATOR_HPP

#include <boost/mpl/has_xxx.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Type checker to determine if a given object provides an
///        iterator type definition.
//////////////////////////////////////////////////////////////////////
BOOST_MPL_HAS_XXX_TRAIT_DEF(iterator)

} // stapl namespace

#endif /* STAPL_VIEWS_TYPE_TRAITS_HAS_ITERATOR_HPP */
