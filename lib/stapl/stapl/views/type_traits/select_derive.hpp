/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TYPE_TRAITS_SELECT_DERIVE_HPP
#define STAPL_VIEWS_TYPE_TRAITS_SELECT_DERIVE_HPP

#include <stapl/utility/use_default.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Determine the most derived class based on the given @c
///        Derived type.
//////////////////////////////////////////////////////////////////////
template<typename Derived, typename Default>
struct select_derived
{
  typedef Derived type;
};


template<typename Default>
struct select_derived<use_default, Default>
{
  typedef Default type;
};

} // namespace stapl

#endif // STAPL_VIEWS_TYPE_TRAITS_SELECT_DERIVE_HPP
