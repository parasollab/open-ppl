/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_MULTISET_BASE_CONTAINER_HPP
#define STAPL_CONTAINERS_UNORDERED_MULTISET_BASE_CONTAINER_HPP

#include <stapl/domains/iterator_domain.hpp>
#include <stapl/containers/base/bc_base.hpp>
#include <stapl/containers/iterators/local_iterator.hpp>
#include <stapl/containers/map/functional.hpp>
#include "multi_key.hpp"

namespace stapl {

template<typename Key, typename Traits>
struct unordered_multiset_base_container;

template<typename Key, typename Traits>
struct container_traits<unordered_multiset_base_container<Key, Traits> >
{
  typedef typename Traits::container_type              container_type;
  typedef typename container_type::value_type          value_type;
  typedef Key                                          key_type;
  typedef key_type                                     mapped_type;
  typedef multi_key<Key>                               gid_type;
  typedef gid_type                                     stored_type;
  typedef iterator_domain<
            container_type,
            domain_impl::f_deref_gid<gid_type> >       domain_type;

  typedef local_accessor<
    unordered_multiset_base_container<Key, Traits> >   accessor_t;
  typedef proxy<key_type, accessor_t>                  reference;
};

//////////////////////////////////////////////////////////////////////
/// @brief The unordered multiset base container
/// @ingroup punorderedmultisetDist
/// @tparam Key Type of the key.
/// @tparam Traits Traits type. Specifies the internal configuration of the
///   container
/// @todo add move semantic constructor and assign operation.
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Traits>
struct unordered_multiset_base_container
  : public bc_base
{
public:
  typedef typename Traits::container_type                   container_type;
  typedef Key                                               key_type;
  typedef key_type                                          mapped_type;
  typedef typename container_type::value_type               value_type;
  typedef multi_key<Key>                                    gid_type;
  typedef gid_type                                          stored_type;
  typedef typename container_type::hasher                   hasher;
  typedef typename container_type::key_equal                key_equal;
  typedef size_t                                            cid_type;
  typedef iterator_domain<
            container_type,
            domain_impl::f_deref_gid<gid_type> >            domain_type;

  typedef local_accessor<unordered_multiset_base_container> accessor_t;
  typedef proxy<value_type,accessor_t>                      reference;
  typedef local_iterator<
            unordered_multiset_base_container,
            accessor_t>                                     iterator;

  typedef const_local_accessor<
    unordered_multiset_base_container<key_type, Traits> >   const_accessor_t;
  typedef proxy<value_type, const_accessor_t>               const_reference;
  typedef const_local_iterator<
            unordered_multiset_base_container,
            const_accessor_t>                               const_iterator;

protected:
  /// @brief Storage for elements assigned to the base container.
  ///   The type of the storage is specified in the Traits parameter.
  container_type m_data;
  cid_type       m_cid;

public:
  ///////////////////////////////////////////////////////////////////
  /// @brief Create a base container with default ID and hash and
  ///   predicate functions
  /// @param hf Optional hash function provided by the user
  /// @param eq Optional predicate function provided by the user
  ///////////////////////////////////////////////////////////////////
  unordered_multiset_base_container(
      hasher const& hf    = hasher(),
      key_equal const& eq = key_equal())
    : m_data(boost::unordered::detail::default_bucket_count,hf,eq),
      m_cid(0)
  { }

  ///////////////////////////////////////////////////////////////////
  /// @brief Create a base container with a specific ID and default
  ///   hash and predicate functions
  /// @param cid An ID for the base container
  /// @param hf Optional hash function provided by the user
  /// @param eq Optional predicate function provided by the user
  ///////////////////////////////////////////////////////////////////
  unordered_multiset_base_container(
      cid_type cid,
      hasher const& hf    = hasher(),
      key_equal const& eq = key_equal())
    : m_data(boost::unordered::detail::default_bucket_count,hf,eq),
      m_cid(cid)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a reference to the data
  /// @todo This function should either be removed or modified to prevent
  ///  access to private data members. The local accessor classes, and
  ///  any other classes that invoke this method, will need to be modified.
  //////////////////////////////////////////////////////////////////////
  container_type& container(void)
  {
    return m_data;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const reference to the data
  /// @todo This function should either be removed or modified to prevent
  ///  access to private data members. The local accessor classes, and
  ///  any other classes that invoke this method, will need to be modified.
  //////////////////////////////////////////////////////////////////////
  container_type const& container(void) const
  {
    return m_data;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the base container ID
  //////////////////////////////////////////////////////////////////////
  cid_type const& cid(void) const
  {
    return m_cid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Access the element associated with the given GID
  /// @param gid The GID of the element
  /// @return The element associated with @p gid if it exists; otherwise
  ///   an invalid value
  //////////////////////////////////////////////////////////////////////
  value_type get_element(const gid_type gid)
  {
    typename container_type::iterator it = this->m_data.find(gid);
    if (it != this->m_data.end())
      return (*it).first;
    stapl_assert(false, "Key does not exist in the base container");
    return index_bounds<value_type>::invalid();
  }

  ////////////////////////////////////////////////////////////////////////
  /// @brief Apply a work function to the specified element
  /// @param gid The GID of the target element
  /// @param f The user-supplied work function to apply
  ////////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void apply(gid_type const& gid, Functor const& f)
  {
    f(*(m_data.find(gid)));
  }

  ///////////////////////////////////////////////////////////////////////
  /// @brief Apply a work function to the specified element
  /// @param gid The GID of the target element
  /// @param f The user-supplied work function to apply
  /// @return The result of applying the work function to the specified element
  ///////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type
  apply_get(gid_type const& gid, Functor const& f)
  {
    return f(*(m_data.find(gid)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert an element into the container.
  /// @param gid The GID to be inserted.
  /// @return Return true if it was inserted
  //////////////////////////////////////////////////////////////////////
  bool insert(gid_type const& gid)
  {
    this->m_data.insert(gid);
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erase an element from the container asynchronously.
  /// @param gid The GID to be deleted.
  //////////////////////////////////////////////////////////////////////
  void erase(gid_type const& gid)
  {
    m_data.erase(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erase an element from the container.
  /// @param gid The GID to be deleted.
  /// @return The number of elements erased
  //////////////////////////////////////////////////////////////////////
  size_t erase_sync(gid_type const& gid)
  {
    return m_data.erase(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a local reference to a specific index of the base
  /// container.
  /// @param gid The GID for which to create the reference
  /// @return A proxy of the value at @p gid
  ////////////////////////////////////////////////////////////////////////
  reference make_reference(gid_type const& gid)
  {
    return reference(accessor_t(this,m_data.find(gid)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a local iterator to a specific index of the base
  /// container.
  /// @param gid The GID for which to create the iterator
  /// @return An iterator of the value at @p gid
  ////////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return iterator(m_data.find(gid),
                    const_cast<unordered_multiset_base_container*>(this));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const local reference to a specific index of the base
  /// container.
  /// @param gid The GID for which to create the reference
  /// @return A proxy of the value at @p gid
  ////////////////////////////////////////////////////////////////////////
  const_reference make_const_reference(gid_type const& gid) const
  {
    return const_reference(const_accessor_t(this,m_data.find(gid)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const local iterator to a specific index of the base
  /// container.
  /// @param gid The GID for which to create the iterator
  /// @return An iterator of the value at @p gid
  ////////////////////////////////////////////////////////////////////////
  const_iterator make_const_iterator(gid_type const& gid) const
  {
    return const_iterator(m_data.find(gid),
                          const_cast<unordered_multiset_base_container*>(this));
  }

  iterator begin() const
  {
    return iterator(m_data.begin(),
                    const_cast<unordered_multiset_base_container*>(this));
  }

  iterator end() const
  {
    return iterator(m_data.end(),
                    const_cast<unordered_multiset_base_container*>(this));
  }

  const_iterator cbegin() const
  {
    return const_iterator(m_data.begin(),
                          const_cast<unordered_multiset_base_container*>(this));
  }

  const_iterator cend() const
  {
    return const_iterator(m_data.end(),
                          const_cast<unordered_multiset_base_container*>(this));
  }

  iterator find(gid_type const& gid) const
  {
    return iterator(m_data.find(gid),
                    const_cast<unordered_multiset_base_container*>(this));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of elements in this base container.
  /// @return The size of type size_t
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    return this->m_data.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return if the component is empty or not.
  /// @return a bool set to true if the component is empty, false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool empty() const
  {
    return this->m_data.empty();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clear the container by removing all the elements in it.
  //////////////////////////////////////////////////////////////////////
  void clear()
  {
    this->m_data.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return the domain of the base container
  /// @return The domain of the base container
  //////////////////////////////////////////////////////////////////////
  domain_type domain(void) const
  {
    return domain_type(this->m_data);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an instance of the container's hash function
  //////////////////////////////////////////////////////////////////////
  hasher hash_function() const
  {
    return this->m_data.hash_function();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an instance of the container's comparator function
  //////////////////////////////////////////////////////////////////////
  key_equal key_eq() const
  {
    return this->m_data.key_eq();
  }

  void define_type(typer& t)
  {
    t.base<bc_base>(*this);
    t.member(m_data);
    t.member(m_cid);
  }
}; // struct unordered_multiset_base_container

/////////////////////////////////////////////////////////////////////////
/// @brief Extracts the GID of the element pointed to by the iterator
/// @param it The target iterator
/// @return The value of the dereferenced iterator
/////////////////////////////////////////////////////////////////////////
template<typename Node>
typename boost::unordered::iterator_detail::c_iterator<Node>::value_type
gid_of(boost::unordered::iterator_detail::c_iterator<Node> it)
{
  return (*it);
}

} // namespace stapl

#endif
