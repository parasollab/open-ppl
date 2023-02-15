/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_INVOKE_ARG_HPP
#define STAPL_UTILITY_INVOKE_ARG_HPP

#include <boost/type_traits/remove_reference.hpp>
#include <boost/type_traits/remove_const.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Wrapper used to specify the argument type during a function
///        call declaration.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template <typename Arg>
struct invoke_arg
  : public boost::remove_const<
      typename boost::remove_reference<Arg>::type
    >
{ };

} // namespace stapl

#endif
