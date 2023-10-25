/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TYPE_TRAITS_IS_PROXY_HPP
#define STAPL_VIEWS_TYPE_TRAITS_IS_PROXY_HPP

#include <boost/mpl/bool.hpp>

namespace stapl {

template<typename T, typename A>
class proxy;

//////////////////////////////////////////////////////////////////////
/// @brief Type checker to determine if a variable is an instantiation
///        of proxy.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct is_proxy
  : public boost::mpl::false_
{ };

template<typename T, typename A>
struct is_proxy<proxy<T,A> >
  : public boost::mpl::true_
{ };

} // namespace stapl

#endif // STAPL_VIEWS_TYPE_TRAITS_IS_PROXY_HPP
