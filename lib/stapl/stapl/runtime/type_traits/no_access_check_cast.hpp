/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_NO_ACCESS_CHECK_CAST_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_NO_ACCESS_CHECK_CAST_HPP

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Cast function to break access checking.
///
/// @ingroup runtimeTypeTraits
//////////////////////////////////////////////////////////////////////
template<typename T, typename U>
T& no_access_check_cast(U& u) noexcept
{
  return (T&)u;
}


//////////////////////////////////////////////////////////////////////
/// @copydoc no_access_check_cast(U&)
///
/// @ingroup runtimeTypeTraits
//////////////////////////////////////////////////////////////////////
template<typename T, typename U>
T const& no_access_check_cast(U const& u) noexcept
{
  return (T const&)u;
}

} // namespace runtime

} //  namespace stapl

#endif
