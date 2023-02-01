/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_RETYPE_HPP
#define STAPL_VIEWS_RETYPE_HPP

#include <boost/static_assert.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Helper class to change the derived type of the given
///        @tparam View type to the given @tparam Child type.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Child>
struct inherit_retype
{
  BOOST_STATIC_ASSERT(sizeof(View) == 0);
};


template<template<typename, typename...> class View, typename C, typename Child>
struct inherit_retype<View<C>, Child>
{
  typedef View<
    C,
    typename view_traits<View<C>>::domain_type,
    typename view_traits<View<C>>::map_function,
    Child
  > type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization to change the derived type of the given view.
//////////////////////////////////////////////////////////////////////
template<template<typename,typename,typename,typename> class VT,
         typename C, typename D, typename F, typename Derived, typename Child>
struct inherit_retype<VT<C,D,F,Derived>, Child>
{
  typedef VT<C,D,F,Child> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper class to change the mapping function type of the
///        given @tparam View to the specified new mapping function type
///        @tparam NF. Base case.
//////////////////////////////////////////////////////////////////////
template<typename View, typename NF>
struct mapfunc_retype
{
  BOOST_STATIC_ASSERT(sizeof(View) == 0);
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for @ref mapfunc_retype to change the
///        mapping function from the given @tparam View type to the
///        specified new mapping function type @tparam NF.
//////////////////////////////////////////////////////////////////////
template<template<typename,typename,typename,typename> class VT,
         typename C, typename D, typename F, typename Derived, typename NF>
struct mapfunc_retype<VT<C,D,F,Derived>, NF>
{
  typedef VT<C,D,NF,Derived> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper class to change the domain type of the given @tparam View
///        to the specified new domain type @tparam NewDom. Base case.
//////////////////////////////////////////////////////////////////////
template<typename View, typename NewDom>
struct domain_retype
{
  BOOST_STATIC_ASSERT(sizeof(View) == 0);
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for @ref domain_retype to change the domain
///        from the given @tparam View type to the specified new
///        domain type @tparam NF.
//////////////////////////////////////////////////////////////////////
template<template<typename, typename, typename, typename> class V,
         typename C, typename D, typename M, typename De, typename NewDom>
struct domain_retype<V<C, D, M, De>, NewDom>
{
  typedef V<C, NewDom, M, De> type;
};

} //namespace stapl

#endif // STAPL_VIEWS_RETYPE_HPP
