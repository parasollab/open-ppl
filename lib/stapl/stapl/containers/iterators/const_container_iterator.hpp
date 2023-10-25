/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_CONST_CONTAINER_ITERATOR_HPP
#define STAPL_CONTAINERS_CONST_CONTAINER_ITERATOR_HPP

#include <stapl/views/iterator/iterator_facade.h>
#include "const_container_iterator_fwd.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Const_iterator that is used to globally traverse a container.
///
/// @tparam Container Type of the container
/// @tparam Accessor Accessor type used for references (proxies)
/// @tparam Category Iterator category of this iterator. By default, it's
/// a forward iterator.
///
/// @see proxy
//////////////////////////////////////////////////////////////////////
template<typename Container,
         typename Accessor,
         typename Category>
class const_container_iterator
  : public iterator_facade<
            const_container_iterator <Container, Accessor, Category>,
            Accessor,
            Category>
{
  friend class stapl::iterator_core_access;

  typedef Accessor                                        accessor_t;
  typedef iterator_facade<const_container_iterator,
                          accessor_t, Category>           base_type;
  typedef typename base_type::difference_type             diff_t;

  typedef typename Container::domain_type                 domain_type;

public:
  typedef typename container_traits<Container>::gid_type  index_type;

private:
  Container const*    m_container;
  index_type          m_index;
  domain_type const   m_domain;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Create an invalid const_iterator.
  //////////////////////////////////////////////////////////////////////
  const_container_iterator(void)
    : m_container(0), m_index(), m_domain()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a global const_iterator over a given container starting at
  /// a specified GID.
  /// @param container Pointer to the container
  /// @param index Index into the container
  //////////////////////////////////////////////////////////////////////
  const_container_iterator(Container const* container, index_type index)
    : m_container(container), m_index(index), m_domain(container->domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a global const_iterator over a given container starting at
  /// a specified GID with a specific domain.
  /// @param container Pointer to the container
  /// @param domain The domain that this iterator will traverse
  /// @param index Index into the container
  //////////////////////////////////////////////////////////////////////
  const_container_iterator(Container const* container,
                           domain_type const& domain, index_type index)
    : m_container(container), m_index(index), m_domain(domain)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Assignment operator for a global const_iterator over a given
  ///        container. If the destination iterator is invalid, the data
  ///        members are directly copied; otherwise, the two iterators must
  ///        reside in the same container and domain.
  /// @param other The iterator being copied
  /// @todo  The assertion check for the domain members does not prove that
  ///        the two domains are identical. The domains need an equality
  ///        operator to provide a stronger test.
  //////////////////////////////////////////////////////////////////////
  void operator=(const_container_iterator const& other)
  {
    if (m_container == NULL) {
      m_container = other.m_container;
      m_index = other.m_index;
      *const_cast<domain_type*>(&this->m_domain) = other.m_container->domain();
    }
    else {
      stapl_assert(m_domain.size() == other.m_domain.size() &&
        m_domain.contains(other.m_index) &&
        m_container->get_rmi_handle() == other.m_container->get_rmi_handle(),
        "The iterators do not reside in the same container or domain.");
      m_index = other.m_index;
    }
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Create an accessor to be used in a reference for the value that
  /// the const_iterator is currently pointing to.
  //////////////////////////////////////////////////////////////////////
  accessor_t access(void) const
  {
    return accessor_t(m_container, m_index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Increment the global const_iterator by one position by
  /// advancing its domain.
  //////////////////////////////////////////////////////////////////////
  void increment(void)
  {
    m_index = m_domain.advance(m_index,1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Decrement the global const_iterator by one position by
  /// advancing its domain in the reverse direction.
  //////////////////////////////////////////////////////////////////////
  void decrement(void)
  {
    m_index = m_domain.advance(m_index,-1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Advance the global const_iterator forward or backward by a specific
  /// amount by advancing its domain.
  /// @param n The amount by which to advance the const_iterator
  //////////////////////////////////////////////////////////////////////
  void advance(diff_t n)
  {
    m_index = m_domain.advance(m_index,n);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compare equality of two iterators by comparing their indices.
  /// @param rhs The other iterator to compare against
  /// @return Whether or not these two iterators point to the same value
  //////////////////////////////////////////////////////////////////////
  bool equal(const_container_iterator const& rhs) const
  {
    return m_index == rhs.m_index;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc equal
  //////////////////////////////////////////////////////////////////////
  template <typename OtherContainer>
  bool equal(const_container_iterator<
             OtherContainer, Category> const& rhs) const
  {
    return m_index == rhs.m_index;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the distance between two iterators by evaluating
  /// the distance of their indices.
  /// @param rhs The other iterator to compare against
  /// @return How far apart the iterators are
  //////////////////////////////////////////////////////////////////////
  diff_t distance_to(const_container_iterator const& rhs) const
  {
    return m_domain.distance(m_index,rhs.m_index);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Determine whether this iterator precedes another in a proper
  /// traversal of the domain.
  /// @param rhs The other iterator to compare against
  /// @return Whether or not this iterator precedes the other
  //////////////////////////////////////////////////////////////////////
  bool less_than(const_container_iterator const& rhs) const
  {
    return m_domain.less_than(m_index,rhs.m_index);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Serialization for this global const_iterator
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.member(m_container);
    t.member(m_index);
    t.member(m_domain);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the index (GID) of the const_iterator
  //////////////////////////////////////////////////////////////////////
  index_type index(void) const
  {
    return m_index;
  }

}; // class const_container_iterator

template <typename C,typename A, typename Cat>
typename container_traits<C>::gid_type
gid_of(const_container_iterator<C,A,Cat> const& it)
{
  return it.index();
}

} // stapl namespace

#endif /* STAPL_CONTAINERS_CONST_CONTAINER_ITERATOR_HPP */
