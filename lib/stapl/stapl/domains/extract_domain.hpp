/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_DOMAINS_EXTRACT_DOMAIN_HPP
#define STAPL_DOMAINS_EXTRACT_DOMAIN_HPP

#include <stapl/domains/domains.hpp>
#include <stapl/views/type_traits/has_domain.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to extract the domain associated with the
///        given @c container.
///
/// Specialization used when the container provides domain
/// information.
//////////////////////////////////////////////////////////////////////
template<typename Container,
         bool b_has_domain = has_domain_type<Container>::value>
struct extract_domain
{
  typedef typename Container::domain_type result_type;

  result_type operator()(Container* container) const
  {
    return container->domain();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to extract the domain associated with the
///        given @c container.
///
/// Specialization used when the container does not specify domain
/// information. A domain based on iterators is returned.
/// @todo Replace seqDom/dom1D with iterator_domain
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct extract_domain<Container, false>
{
  typedef typename Container::iterator    iterator;
  typedef seqDom<iterator>                dom_t;
  typedef dom1D<iterator, dom_t>          result_type;

  result_type operator()(Container* container) const
  {
    if (container->size() > 0) {
      return result_type(
        dom_t(container->begin(),--(container->end()), true, container->size())
      );
    }

    // else
    return result_type(
      dom_t(container->begin(), container->begin(), true, 0)
    );
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to extract the domain associated with the
///        given @c container.
///
/// Specialization used when the container is a std::list.
/// @todo Replace seqDom/dom1D with iterator_domain
/// @todo Probably move these STL specific specializations to a
/// different file.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct extract_domain<std::list<T>, false>
{
private:
  typedef typename std::list<T>::iterator     iterator;
  typedef seqDom<iterator>                     dom_t;

public:
  typedef dom1D<iterator,dom_t>               result_type;

  result_type operator()(std::list<T>* ct)
  {
    if (ct->size() > 0)
    {
      return result_type(
        dom_t(ct->begin(), --(ct->end()), true, ct->size())
      );
    }

    // else
    return result_type(
      dom_t(ct->begin(), ct->begin(), true, 0)
    );
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to extract the domain associated with the
///        given @c container.
///
/// Specialization used when the container is a std::vector.
/// @todo Probably move these STL specific specializations to a
/// different file.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct extract_domain<std::vector<T>, false>
{
  typedef indexed_domain<size_t> result_type;

  result_type operator()(std::vector<T>* container) const
  {
    return result_type(container->size());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to extract the domain associated with the
///        given @c container.
///
/// Specialization used when the container is a std::string.
/// @todo Probably move these STL specific specializations to a
/// different file.
//////////////////////////////////////////////////////////////////////
template<>
struct extract_domain<std::string, false>
{
  typedef indexed_domain<size_t> result_type;

  result_type operator()(std::string* container) const
  {
    return result_type(container->size());
  }
};

} // namespace detail

} // namespace stapl

#endif // STAPL_DOMAINS_EXTRACT_DOMAIN_HPP
