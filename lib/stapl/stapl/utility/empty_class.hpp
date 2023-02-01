/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_EMPTY_CLASS_HPP
#define STAPL_UTILITY_EMPTY_CLASS_HPP

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Empty class.
///
/// @ref empty_class is used with when invoking workfunctions returning void
/// to avoid code duplication in @p Task.
///
/// It is also used in @ref stapl::detail::directory_request_base as trivial
/// hook type when not using intrusive containers.
///
/// @sa result_storage_mf, directory_request_base
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
struct empty_class
{
  template<typename ...Args>
  empty_class(Args&&...)
  { }
};

} // namespace stapl

#endif // STAPL_UTILITY_EMPTY_CLASS_HPP
