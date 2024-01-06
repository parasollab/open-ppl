/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_LIST_LIST_BASE_CONTAINER_HPP
#define STAPL_CONTAINERS_LIST_LIST_BASE_CONTAINER_HPP

#include <stapl/containers/base/bc_base.hpp>
#include <stapl/containers/list/list_gid.hpp>
#include <stapl/containers/iterators/local_iterator.hpp>
#include <stapl/containers/iterators/const_local_iterator.hpp>
#include <stapl/containers/distribution/base_container_metadata.hpp>
#include <stapl/containers/type_traits/define_value_type.hpp>
#include "traits/base_container_traits.hpp"

#include <stapl/domains/list_bc_domain.hpp>

namespace stapl {

template<typename Traits>
struct list_base_container;


template<typename Traits>
struct container_traits<list_base_container<Traits> >
{
  typedef typename Traits::value_type                      value_type;
  typedef typename Traits::container_type                  container_type;
  typedef local_iterator<list_base_container<Traits> >     iterator;
  typedef list_gid<iterator, list_base_container<Traits> > gid_type;
  typedef list_bc_domain<
    list_base_container<Traits>,
    gid_generator<gid_type,list_base_container<Traits> >
  >                                                        domain_type;
  typedef local_accessor<list_base_container<Traits> >     accessor_t;
  typedef proxy<value_type, accessor_t>                    reference;
  typedef const_local_accessor<list_base_container<Traits> > const_accessor_t;
  typedef proxy<value_type, const_accessor_t>              const_reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines the base container used for @ref list.
/// @ingroup plistDist
/// @tparam Traits A traits class that defines customizable components
///                of the base container, such as the domain type and
///                storage type.
//////////////////////////////////////////////////////////////////////
template<typename Traits>
struct list_base_container
  : public bc_base
{
  typedef typename Traits::value_type                          value_type;
  // backend container
  typedef typename Traits::container_type                      container_type;
  typedef local_accessor<list_base_container>                  accessor_t;
  typedef proxy<value_type,accessor_t>                         reference;
  typedef const_local_accessor<list_base_container>            const_accessor_t;
  typedef proxy<value_type,const_accessor_t>                   const_reference;
  typedef local_iterator<list_base_container>                  iterator;
  typedef const_local_iterator<list_base_container>            const_iterator;
  typedef list_gid<iterator, list_base_container>              gid_type;
  typedef list_bc_domain<
    list_base_container,
    gid_generator<gid_type,list_base_container> >              domain_type;
  typedef size_t                                               cid_type;

protected:
  container_type m_data;
  domain_type    m_domain;

public:
  list_base_container(location_type loc)
    : m_domain(*this, gid_generator<gid_type, list_base_container>(this, loc))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a list base container with a given @p size
  ///        using a default constructed value.
  //////////////////////////////////////////////////////////////////////
  list_base_container(size_t size, location_type loc)
    : m_data(size, typename Traits::stored_type()),
      m_domain(*this, gid_generator<gid_type, list_base_container>(this, loc))
  {
    if (size==0) m_data.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a list base container with @p size elements and
  ///        initializes them with @p value.
  //////////////////////////////////////////////////////////////////////
  list_base_container(size_t size, value_type const& value, location_type loc)
    : m_data(size, typename Traits::stored_type(value)),
      m_domain(*this, gid_generator<gid_type, list_base_container>(this, loc))
  {
    if (size==0) m_data.clear();
  }

  list_base_container(list_base_container const& other)
    : m_domain(
       *this,
       gid_generator<gid_type, list_base_container>
         (this, other.domain().generator().location())
      )
  {
    this->m_data.resize(other.size());
    this->m_data = other.m_data;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an iterator to a specific @p gid of the list.
  ////////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return gid.m_pointer;
  }

  iterator begin(void)
  {
    return iterator(m_data.begin(),this);
  }

  iterator end(void)
  {
    return iterator(m_data.end(),this);
  }

  const_iterator begin(void) const
  {
    return const_iterator(m_data.begin(),this);
  }

  const_iterator end(void) const
  {
    return const_iterator(m_data.end(),this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Verify if this method is used.
  //////////////////////////////////////////////////////////////////////
  const_reference get_element(gid_type const& gid) const
  {
    return *(gid.m_pointer);
  }

  value_type get_element(gid_type const& gid)
  {
    return *(gid.m_pointer);
  }

  void set_element(gid_type const& gid,
                   typename Traits::stored_type const& value)
  {
    *(gid.m_pointer) = value;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator for the element referenced by the
  ///        given @p gid.
  //////////////////////////////////////////////////////////////////////
  iterator find(gid_type const& gid)
  {
    return gid.m_pointer;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator for the element referenced by the
  ///        given @p gid.
  /// @todo Verify who needs this method.
  //////////////////////////////////////////////////////////////////////
  const_iterator find_element(gid_type const& gid) const
  {
    return gid.m_pointer;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Search if the given @p gid is contained between the range
  ///        defined with the iterators @p first and @p last.
  //////////////////////////////////////////////////////////////////////
  bool search(gid_type const& first, gid_type const& last, gid_type const& gid)
  {
    typename gid_type::pointer_type it = first.m_pointer;

    while (it != last.m_pointer)
    {
      if (gid.m_pointer == it)
        return true;

      ++it;
    }

    if (gid.m_pointer == it)
      return true;

    return false;
  }

  void push_back(value_type const& value)
  {
    this->m_data.push_back(value);
  }

  void pop_back(void)
  {
    this->m_data.pop_back();
  }

  void push_front(value_type const& value)
  {
    this->m_data.push_front(value);
  }

  void pop_front(void)
  {
    this->m_data.pop_front();
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Verify the implementation of this method.
  //////////////////////////////////////////////////////////////////////
  reference front(void)
  {
    return this->m_data.front();
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Verify the implementation of this method.
  //////////////////////////////////////////////////////////////////////
  const_reference front(void) const
  {
    return this->m_data.front();
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Verify the implementation of this method.
  //////////////////////////////////////////////////////////////////////
  reference back(void)
  {
    return this->m_data.back();
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Verify the implementation of this method.
  //////////////////////////////////////////////////////////////////////
  const_reference back(void) const
  {
    return this->m_data.back();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given @p value before the element referenced
  ///        by the @p gid.
  /// @todo Should this method return an iterator?
  //////////////////////////////////////////////////////////////////////
  void insert(gid_type const& gid, value_type const& value)
  {
    if (gid.m_base_container == nullptr)
       this->m_data.push_back(value);
    else
       this->m_data.insert(gid.m_pointer.operator->(), value);
  }

  void remove(gid_type const& gid)
  {
    this->m_data.erase(gid.m_pointer.operator->());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Splits the list at position @p pos and inserts the
  ///        sublist defined by the iterators @p first and @p last
  ///        between the resulting lists.
  ///
  /// @todo Verify if this is the correct interface and implement this
  ///       method.
  //////////////////////////////////////////////////////////////////////
  void splice(iterator const& it, list_base_container* nc,
              iterator const& first, iterator const& last)
  {
    abort("splice not implemented");
    // this->m_data.splice(it, nc->m_data, first, last);
  }

  container_type& container(void)
  {
    return m_data;
  }

  domain_type const& domain(void) const
  {
    return m_domain;
  }

  size_t size(void) const
  {
    return this->m_data.size();
  }

  bool empty(void) const
  {
    return this->m_data.empty();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes all the elements of this base container.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->m_data.clear();
  }

  void define_type(typer &t)
  {
    t.base<bc_base>(*this);
    t.member(this->m_data);
  }
}; // struct list_base_container

} // namespace stapl

#endif // STAPL_CONTAINERS_LIST_LIST_BASE_CONTAINER_HPP
