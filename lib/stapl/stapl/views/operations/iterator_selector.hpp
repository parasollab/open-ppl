/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_ITERATOR_SELECTOR_HPP
#define STAPL_VIEWS_ITERATOR_SELECTOR_HPP

#include <stapl/views/type_traits/has_iterator.hpp>
#include <stapl/views/type_traits/is_domain_sparse.hpp>
#include <stapl/views/type_traits/is_identity.hpp>
#include <stapl/views/type_traits/is_view.hpp>
#include <stapl/views/type_traits/has_vertex_property.hpp>

#include <stapl/views/operations/view_iterator.hpp>
#include <stapl/views/operations/const_view_iterator.hpp>

#include <boost/mpl/eval_if.hpp>
#include <boost/mpl/identity.hpp>
#include <boost/mpl/logical.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper to return the iterator type provided for the
///        @p View's container.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct get_iterator
{ using type = typename view_traits<View>::container::iterator; };


//////////////////////////////////////////////////////////////////////
/// @brief Helper to return the const_iterator type provided for the
///        @p View's container.
//////////////////////////////////////////////////////////////////////
template <typename View>
struct get_const_iterator
{ using type = typename view_traits<View>::container::const_iterator; };


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to determine the type of iterator to use
///        based on the View type and its value type T
//////////////////////////////////////////////////////////////////////
template <typename View,
          typename T,
          typename Category = std::forward_iterator_tag>
struct iterator_selector
{
  using type = typename boost::mpl::eval_if<
    /* if */ boost::mpl::and_<
               boost::mpl::not_<
                 is_domain_sparse<typename view_traits<View>::domain_type>>,
               is_identity<typename view_traits<View>::map_function>,
               has_iterator<typename view_traits<View>::container>>,
    /*then*/ get_iterator<View>,  // uses the container's iterator type
    /*else*/ boost::mpl::eval_if_c<
               /* if */ is_view<typename view_traits<View>::value_type>::value,
               /*then*/ boost::mpl::identity<detail::view_iterator<View>>,
               /*else*/ boost::mpl::eval_if<
                 /* if */ boost::mpl::and_<
                            is_container<typename view_traits<View>::container>,
                            has_iterator<typename view_traits<View>::container>,
                            std::is_same<
                              typename view_traits<View>::domain_type,
                              typename container_traits<
                                typename view_traits<View>::container
                              >::domain_type>,
                            has_vertex_property<
                              typename view_traits<View>::container>>,
                          get_iterator<View>,
                 /*else*/ boost::mpl::identity<index_iterator<View, Category>>
               >>>::type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to determine the type of iterator to use
///        based on the View type and its value type T
//////////////////////////////////////////////////////////////////////
template <typename View,
          typename T,
          typename Category = std::forward_iterator_tag>
struct const_iterator_selector
{
  using type = typename boost::mpl::eval_if<
    /* if */ boost::mpl::and_<
               boost::mpl::not_<
                 is_domain_sparse<typename view_traits<View>::domain_type> >,
               is_identity<typename view_traits<View>::map_function>,
               has_iterator<typename view_traits<View>::container> >,
    /*then*/ get_iterator<View>,  // uses the container's iterator type
    /*else*/ boost::mpl::eval_if_c<
               /* if */ is_view<typename view_traits<View>::value_type>::value,
               /*then*/ boost::mpl::identity<detail::const_view_iterator<View>>,
               /*else*/ boost::mpl::eval_if<
                 /* if */ boost::mpl::and_<
                            is_container<typename view_traits<View>::container>,
                            has_iterator<typename view_traits<View>::container>,
                            std::is_same<
                              typename view_traits<View>::domain_type,
                              typename container_traits<
                                typename view_traits<View>::container
                              >::domain_type>,
                            has_vertex_property<
                              typename view_traits<View>::container>>,
                          get_iterator<View>,
                /*else*/ boost::mpl::identity<
                           const_index_iterator<View, Category>>
               >>>::type;
};

} // namespace detail

} // namespace stapl

#endif // STAPL_VIEWS_ITERATOR_SELECTOR_HPP
