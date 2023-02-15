/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_TRAITS_HPP
#define STAPL_CONTAINERS_TRAITS_HPP

#include <vector>
#include <list>

#include <boost/mpl/bool.hpp>

#include <stapl/containers/type_traits/is_container.hpp>
#include <stapl/domains/indexed_fwd.hpp>
#include <stapl/views/type_traits/has_domain.hpp>

namespace stapl {

template <typename T, typename Dom>
struct dom1D;

template <typename Iterator>
class seqDom;

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to retrieve associated types for containers.
/// This class is similar in spirit to the STL's iterator_traits.
///
/// @tparam Container The container for which to query.
//////////////////////////////////////////////////////////////////////
template <typename Container>
struct container_traits
{
  typedef typename Container::gid_type        gid_type;
  typedef typename Container::value_type      value_type;
  typedef typename Container::domain_type     domain_type;
  typedef typename Container::reference       reference;
  typedef typename Container::const_reference const_reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for std::vector
///
/// @see container_traits
//////////////////////////////////////////////////////////////////////
template<typename T>
struct container_traits<std::vector<T>>
{
  typedef size_t                                   gid_type;
  typedef T                                        value_type;
  typedef indexed_domain<gid_type>                 domain_type;
  typedef std::vector<T>                           container_type;
  typedef typename container_type::iterator        iterator;
  typedef typename container_type::reference       reference;
  typedef typename container_type::const_reference const_reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for proxy<std::vector<T>, Accessor>>
///
/// @see container_traits
//////////////////////////////////////////////////////////////////////
template<typename T, typename Accessor>
struct container_traits<proxy<std::vector<T>, Accessor>>
{
  typedef size_t                                   gid_type;
  typedef T                                        value_type;
  typedef indexed_domain<gid_type>                 domain_type;
  typedef proxy<std::vector<T>, Accessor>          container_type;
  typedef typename container_type::iterator        iterator;
  typedef typename container_type::reference       reference;
  typedef typename container_type::const_reference const_reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for std::list
///
/// @see container_traits
//////////////////////////////////////////////////////////////////////
template<typename T>
struct container_traits<std::list<T> >
{
  typedef std::list<T>                       container_type;
  typedef typename container_type::iterator  gid_type;
  typedef T                                  value_type;
  typedef typename container_type::reference reference;
  typedef typename container_type::const_reference const_reference;
  typedef typename container_type::iterator  iterator;
  typedef dom1D<iterator, seqDom<iterator> > domain_type;
};


namespace view_impl {


//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction returning @p true_ if @p Traits reflects
///   the type @p enable_view_reference
//////////////////////////////////////////////////////////////////////
template<typename Traits, typename Void = void>
struct has_enable_view_reference
  : boost::mpl::false_
{ };


template<typename Traits>
struct has_enable_view_reference<Traits, typename Traits::enable_view_reference>
  : boost::mpl::true_
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Reflecting whether for a given type @p T, a view should be
///   used as a reference to it (in place of a proxy).  True if @p T
///   is a container and its traits class reflects @p enable_view_reference.
//////////////////////////////////////////////////////////////////////
template<typename T, bool = is_container<T>::value>
struct use_view_as_reference
  : boost::mpl::false_
{ };


template<typename T>
struct use_view_as_reference<T, true>
  : has_enable_view_reference<container_traits<T> >
{ };


template <typename Domain, typename Container>
Domain get_domain_helper(const Container& container, boost::mpl::true_)
{
  return container.domain();
}


template <typename Domain, typename Container>
Domain get_domain_helper(const Container& container, boost::mpl::false_)
{
  return Domain(container.size());
}

} // namespace view_impl


template <typename Container>
typename container_traits<Container>::domain_type
get_domain(const Container& container)
{
  typedef typename container_traits<Container>::domain_type  domain_type;
  return view_impl::get_domain_helper<domain_type>(
       container,typename has_domain_type<Container>::type());
}

} // namespace stapl


#endif // STAPL_CONTAINERS_TRAITS_HPP
