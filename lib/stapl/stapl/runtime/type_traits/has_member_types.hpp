/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_HAS_MEMBER_TYPES_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_HAS_MEMBER_TYPES_HPP

#include <boost/mpl/has_xxx.hpp>

namespace stapl {

namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief Returns if @c T::member_types is defined.
///
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
BOOST_MPL_HAS_XXX_TRAIT_DEF(member_types)

} // namespace runtime

} // namespace stapl

#endif
