/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SET_BASE_CONTAINER_HPP
#define STAPL_CONTAINERS_SET_BASE_CONTAINER_HPP

#include <stapl/domains/iterator_domain.hpp>
#include <stapl/containers/base/bc_base.hpp>
#include <stapl/containers/iterators/local_iterator.hpp>
#include <stapl/containers/iterators/const_local_iterator.hpp>
#include <stapl/containers/set/base_container_traits.hpp>

namespace stapl {

template<typename Key, typename Compare, typename Traits>
struct set_base_container;


template<typename Key, typename Compare, typename Traits>
struct container_traits<set_base_container<Key, Compare, Traits> >
{
  typedef Key                                            value_type;
  typedef typename Traits::domain_type                   domain_type;
  typedef Key                                            gid_type;
  typedef typename Traits::container_type                container_type;

  typedef local_accessor<
    set_base_container<Key, Compare, Traits>
  >                                                      accessor_t;
  typedef proxy<value_type, accessor_t>                  reference;

  typedef const_local_accessor<
    set_base_container<Key, Compare, Traits>
  >                                                      const_accessor_t;
  typedef proxy<value_type, const_accessor_t>            const_reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief Base container used for @ref set container.
/// @ingroup psetDist
///
/// @tparam Key     Type of key objects.
/// @tparam Compare Comparison function object type.
/// @tparam Traits  A traits class that defines customizable components
///                 of the base container, such as the domain type and
///                 storage type.
/// @todo Instance of the compare class should be passed to this class
/// @todo Add const reference and iterators, move constructor
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Compare, typename Traits>
struct set_base_container
  : public bc_base
{
  /// Backend container
  typedef typename Traits::container_type             container_type;
  typedef Key                                         key_type;
  typedef Key                                         value_type;
  typedef Compare                                     compare_type;
  typedef key_type                                    gid_type;
  typedef size_t                                      cid_type;
  typedef typename Traits::domain_type                domain_type;

  typedef local_accessor<set_base_container>          accessor_t;
  typedef proxy<value_type, accessor_t>               reference;
  typedef local_iterator<set_base_container>          iterator;

  typedef const_local_accessor<set_base_container>    const_accessor_t;
  typedef proxy<value_type, const_accessor_t>         const_reference;
  typedef const_local_iterator<set_base_container>    const_iterator;

protected:
  /// @todo why domain is ignored?
  //  domain_type     m_domain;
  /// The underlying raw data
  container_type  m_data;
  /// This base container's id
  cid_type        m_cid;

public:

  //////////////////////////////////////////////////////////////////////
  /// @todo Make sure this default constructor is needed. Instead,
  /// move semantics constructors and assign operators need to be added.
  //////////////////////////////////////////////////////////////////////
  set_base_container(void)
    : m_cid()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @todo Make sure this default constructor is needed.
  //////////////////////////////////////////////////////////////////////
  set_base_container(cid_type const& cid)
    : m_cid(cid)
  { }

  ~set_base_container(void)
  {
    bc_base_impl::cleanup_elements<value_type>::apply(m_data,
                                                      this->m_defer_cleanup);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the element corresponding to a specific GID. The element
  /// must be present in this base container.
  /// @param key The id of the element for which we want to read
  /// the value.
  /// @return A copy of the element.
  //////////////////////////////////////////////////////////////////////
  value_type get_element(key_type const& key)
  {
    typename container_type::iterator it = this->m_data.find(key);
    if (it!=this->m_data.end())
      return *it;
    stapl_assert(false, "key does not exist in map base container");
    return value_type();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the element corresponding to a specific GID.
  ///
  /// This method is used in the redistribution of composed containers.
  /// It is needed to allow the distributor object to get the instance of
  /// the @ref container_wrapper_ref for a container instance on one location
  /// and send it to another location where it will be placed in a base
  /// container by calling set_element. @ref set_element only accepts
  /// instances of the stored type.
  ///
  /// @param gid The id associated with the element for which we want to read
  /// the value.
  ///
  /// @return A copy of the stored instance of the element.
  //////////////////////////////////////////////////////////////////////
  value_type get_stored_element(key_type const& key)
  {
    typename container_type::iterator it = this->m_data.find(key);

    if (it!=this->m_data.end())
      return *it;

    stapl_assert(false, "key does not exist in map base container");

    return value_type();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Attempts to insert an element into the @c set_base_container.
  /// The element is inserted if it is unique in the set.
  ///
  /// @param  val Element to be inserted.
  ///
  /// @return @c true if the element was actually inserted.
  //////////////////////////////////////////////////////////////////////
  bool insert(value_type const& val)
  {
    typedef typename container_type::const_iterator iterator;
    std::pair<iterator, bool> op = this->m_data.insert(val);
    return op.second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert a <Key,Mapped> pair in the container, or apply a functor
  /// to the element if it is already in the container.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void insert_fn(value_type const& val, Functor const& f)
  {
    auto ret = this->m_data.insert(val);

    if (!ret.second)
      f(*ret.first);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a local reference to a specific element of the
  /// base container.
  ///
  /// @param key The GID for which to create the reference
  /// @return A proxy of the value at gid
  ////////////////////////////////////////////////////////////////////////
  reference make_reference(key_type const& key)
  {
    return reference(accessor_t(this, m_data.find(key)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a local const_reference to a specific element of the
  /// base container.
  ///
  /// @param gid The GID for which to create the reference
  /// @return A proxy of the value at gid
  ////////////////////////////////////////////////////////////////////////
  const_reference make_reference(key_type const& key) const
  {
    return const_reference(const_accessor_t(this, m_data.find(key)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a local iterator to a specific element of the base
  /// container.
  ///
  /// @param key The GID for which to create the iterator
  /// @return An iterator of the value at gid
  ////////////////////////////////////////////////////////////////////////
  iterator make_iterator(key_type const& key)
  {
    return iterator(m_data.find(key), this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a local iterator to a specific element of the base
  /// container.
  ///
  /// @param gid The GID for which to create the iterator
  /// @return An iterator of the value at gid
  ////////////////////////////////////////////////////////////////////////
  const_iterator make_iterator(key_type const& key) const
  {
    return const_iterator(m_data.find(key), this);
  }

  container_type& container(void)
  {
    return m_data;
  }

  container_type const& container(void) const
  {
    return m_data;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of elements in this base container.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return this->m_data.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return whether or not this base container is empty.
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return (this->m_data.size()==0);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clear this base container by removing all of its elements.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->m_data.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the base container's ID.
  ///
  /// @todo Remove getters that begin with get_.
  //////////////////////////////////////////////////////////////////////
  cid_type get_cid(void) const
  {
    return m_cid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the base container's ID.
  //////////////////////////////////////////////////////////////////////
  cid_type const& cid(void) const
  {
    return m_cid;
  }

  void define_type(typer& t)
  {
    t.base<bc_base>(*this);
    t.member(m_data);
    t.member(m_cid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a domain over all elements of the base container.
  //////////////////////////////////////////////////////////////////////
  domain_type domain(void) const
  {
    return domain_type(this->m_data);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element with specified GID.
  /// @param gid GID of the element on which to apply the functor
  /// @param f Functor to apply
  /// @note This function is required by the associative_distribution::insert
  ///   even if the functor called is doing nothing on the element.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void apply_set(gid_type const& gid, Functor const& f)
  {
    f(*(m_data.find(gid)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element with specified GID.
  /// @param gid GID of the element on which to apply the functor
  /// @param f Functor to apply
  /// @return The result of the functor.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type
  apply_get(gid_type const& gid, Functor const& f)
  {
    return f(*(m_data.find(gid)));
  }
}; // struct set_base_container

} // namespace stapl

#endif // STAPL_CONTAINERS_SET_BASE_CONTAINER_HPP
