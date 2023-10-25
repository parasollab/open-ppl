/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_VIEW_TRAITS_HPP
#define STAPL_VIEWS_VIEW_TRAITS_HPP

#include <boost/mpl/bool.hpp>
#include <boost/static_assert.hpp>

#include <stapl/views/proxy.h>
#include <stapl/views/type_traits/is_view.hpp>
#include <stapl/views/type_traits/is_proxy.hpp>
#include <stapl/views/mapping_functions/identity.hpp>

#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/containers/type_traits/is_container.hpp>

#include <stapl/views/proxy/stl_vector_accessor.hpp>

namespace stapl {

namespace view_impl {

template<typename View>
class gid_accessor;

template<typename C>
struct is_stl_vector
  : public boost::mpl::false_
{ };


template<typename T, typename A>
struct is_stl_vector<std::vector<T, A> >
  : public boost::mpl::true_
{ };

} // namespace view_impl


template<typename C,
          bool = view_impl::is_stl_vector<C>::value,
          bool = is_container<typename container_traits<C>::value_type>::value>
struct extract_reference_type
{
  typedef typename C::reference type;
};


template<typename C>
struct extract_reference_type<C, true, true>
{
  BOOST_STATIC_ASSERT_MSG(
    sizeof(C) == 0,
    "detected both stl vector and value type of stapl container"
   );
};


template<typename C>
struct extract_reference_type<C, true, false>
{
  typedef proxy<typename C::value_type, stl_vector_accessor<C> > type;
};


template<typename C,
          bool = view_impl::is_stl_vector<C>::value,
          bool = is_container<typename container_traits<C>::value_type>::value>
struct extract_const_reference_type
{
  typedef typename C::const_reference type;
};


template<typename C>
struct extract_const_reference_type<C, true, true>
{
  BOOST_STATIC_ASSERT_MSG(
    sizeof(C) == 0,
    "detected both stl vector and value type of stapl container"
   );
};


template<typename C>
struct extract_const_reference_type<C, true, false>
{
  typedef proxy<typename C::value_type, stl_vector_const_accessor<C> > type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction to reflect identity mapping function if
///  the passed view type parameter is actually a container, and hence
///  does not define this type (and is by definition identity).
//////////////////////////////////////////////////////////////////////
template<typename View, typename Index, bool = is_view<View>::value>
struct compute_mf_type
{
  typedef typename View::map_function type;
};


template<typename View, typename Index>
struct compute_mf_type<View, Index, false>
{
  typedef f_ident<Index> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Define traits for views and containers that do not conform
///   to the V<C,D,F,Derived> template parameter convention.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct view_traits_impl
{
  typedef View                                          container;
  typedef typename container::reference                 reference;
  typedef typename container::const_reference           const_reference;
  typedef typename View::value_type                     value_type;
  typedef typename View::gid_type                       index_type;
  typedef typename View::domain_type                    domain_type;
  typedef typename compute_mf_type<
    View, typename domain_type::index_type>::type       map_function;
};


//////////////////////////////////////////////////////////////////////
/// @brief Reflects common trait types based on explicitly passed
///  container, domain, mapping function, and derived parameter types.
//////////////////////////////////////////////////////////////////////
template<typename C, typename D, typename F, typename Derived>
struct default_view_traits
{
  typedef typename container_traits<C>::value_type             value_type;
  typedef typename extract_reference_type<C>::type             reference;
  typedef typename extract_const_reference_type<C>::type       const_reference;
  typedef C                                                    container;
  typedef F                                                    map_function;
  typedef D                                                    domain_type;
  typedef typename D::index_type                               index_type;
  typedef Derived                                              derived_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper structure to extract the types provided by the given
///        view type.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct view_traits
  : view_traits_impl<View>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Helper structure to extract the types provided by the given
///   view type.  Conditional inheritance guards against templates
///   matching this signature that aren't views and hence don't conform
///   to their template parameter convention
///   (i.e., @ref multiarray_base_container).
//////////////////////////////////////////////////////////////////////
template<template<typename,typename,typename,typename> class VT,
         typename C, typename D, typename F, typename Derived>
struct view_traits<VT<C,D,F,Derived>>
  : std::conditional<std::is_class<C>::value,
                     default_view_traits<C, D, F, Derived>,
                     view_traits_impl<VT<C,D,F,Derived>>>::type
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Macro used to define the types required for the given view type.
//////////////////////////////////////////////////////////////////////
#define STAPL_VIEW_REFLECT_TRAITS(view_type)                                \
  typedef typename view_traits<view_type>::value_type      value_type;      \
  typedef typename view_traits<view_type>::reference       reference;       \
  typedef typename view_traits<view_type>::const_reference const_reference; \
  typedef typename view_traits<view_type>::map_function    map_function;    \
  typedef typename view_traits<view_type>::domain_type     domain_type;     \
  typedef typename view_traits<view_type>::index_type      index_type;      \
  typedef typename view_traits<view_type>::map_function    map_func_type;   \
  typedef typename view_traits<view_type>::container       view_container_type;

} // namespace stapl

#endif // STAPL_VIEWS_VIEW_TRAITS_HPP
