/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_PROXY_ACCESSOR_HPP
#define STAPL_VIEWS_PROXY_ACCESSOR_HPP

#include <boost/utility/result_of.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Helper class to access the private methods of the given
///        accessor.
///
/// The given accessor should have declared as a friend this class.
//////////////////////////////////////////////////////////////////////
class accessor_core_access
{
public:
  accessor_core_access(void) = delete;

  template<typename A, typename F>
  static void
  apply_set(A const& a, F const& f)
  {
    a.apply_set(f);
  }

  template<typename A, typename F>
  static
  typename boost::result_of<F(typename A::value_type)>::type
  apply_get(A const& a, F const& f)
  {
    return a.apply_get(f);
  }

  template<typename A, typename T>
  static void
  write(A& a, T const& val)
  {
    a.write(val);
  }

  template<typename A>
  static bool is_local(A const& a)
  {
    return a.is_local();
  }
}; // class accessor_core_access

} // namespace stapl

#endif // STAPL_VIEWS_PROXY_ACCESSOR_HPP
