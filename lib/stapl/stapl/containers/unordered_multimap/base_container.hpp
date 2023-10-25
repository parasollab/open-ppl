/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_MULTIMAP_BASE_CONTAINER_HPP
#define STAPL_CONTAINERS_UNORDERED_MULTIMAP_BASE_CONTAINER_HPP

#include <stapl/domains/iterator_domain.hpp>
#include <stapl/containers/base/bc_base.hpp>
#include <stapl/containers/iterators/local_iterator.hpp>
#include <stapl/containers/iterators/local_accessor.hpp>
#include <stapl/containers/map/functional.hpp>
#include <stapl/containers/type_traits/define_value_type.hpp>
#include <stapl/containers/unordered_multiset/multi_key.hpp>

namespace stapl {

template<typename Traits>
struct unordered_multimap_base_container;

template<typename Traits>
struct container_traits<unordered_multimap_base_container<Traits> >
  : public Traits
{
  typedef typename Traits::key_type                      key_type;
  typedef typename Traits::mapped_type                   mapped_type;
  typedef typename Traits::value_type                    value_type;
  typedef typename Traits::stored_type                   stored_type;
  typedef local_accessor<
            unordered_multimap_base_container<Traits> >  accessor_t;
  typedef proxy<value_type, accessor_t>                  reference;
};

//////////////////////////////////////////////////////////////////////
/// @brief The unordered multimap base container
/// @ingroup punorderedmultimapDist
/// @tparam Key Type of the key.
/// @tparam Mapped Type of the mapped value. Each element stores data of the
///   mapped type.
/// @tparam Traits Traits type. Specifies the internal configuration of the
///   container
/// @todo add move semantic constructor and assign operation.
//////////////////////////////////////////////////////////////////////
template<typename Traits>
struct unordered_multimap_base_container
  : public bc_base
{
private:
  typedef typename Traits::key_type     Key;
  typedef typename Traits::mapped_type  Mapped;
  typedef typename Traits::compare_type Compare;

public:
  typedef typename Traits::container_type                   container_type;
  typedef Key                                               key_type;
  typedef Mapped                                            mapped_type;
  typedef multi_key<key_type>                               gid_type;
  typedef std::pair<const gid_type, mapped_type>            value_type;
  typedef value_type                                        stored_type;
  typedef Compare                                           compare_type;

  typedef size_t                                            cid_type;
  typedef typename container_type::hasher                   hasher;
  typedef typename container_type::key_equal                key_equal;
  typedef typename Traits::domain_type                      domain_type;

  typedef local_accessor<
            unordered_multimap_base_container>              accessor_t;
  typedef proxy<value_type, accessor_t>                     reference;
  typedef local_iterator<unordered_multimap_base_container,
            accessor_t>                                     iterator;

  typedef const_local_accessor<
            unordered_multimap_base_container>              const_accessor_t;
  typedef proxy<value_type, const_accessor_t>               const_reference;
  typedef const_local_iterator<
            unordered_multimap_base_container,
            const_accessor_t>                               const_iterator;

private:
  typedef member_accessor<
    mapped_type,
    typename reference::template get_second<mapped_type>,
    accessor_t
  >                                                   second_accessor_type;

public:
  typedef proxy<mapped_type, second_accessor_type>    second_reference;

protected:
  /// @brief Storage for elements assigned to the base container.
  ///   The type of the storage is specified in the Traits parameter.
  container_type  m_data;
  size_t          m_cid;

public:
  ///////////////////////////////////////////////////////////////////
  /// @brief Create a base container with default ID and hash and
  ///   predicate functions
  /// @param hf Optional hash function provided by the user
  /// @param eq Optional predicate function provided by the user
  ///////////////////////////////////////////////////////////////////
  unordered_multimap_base_container(
    hasher const& hf    = hasher(),
    key_equal const& eq = key_equal())
    : m_data(boost::unordered::detail::default_bucket_count, hf, eq),
      m_cid(0)
  { }

  ///////////////////////////////////////////////////////////////////
  /// @brief Create a base container with a specific ID and default
  ///   hash and predicate functions
  /// @param hf Optional hash function provided by the user
  /// @param eq Optional predicate function provided by the user
  ///////////////////////////////////////////////////////////////////
  unordered_multimap_base_container(
    size_t cid,
    hasher const& hf = hasher(),
    key_equal const& eq = key_equal())
    : m_data(boost::unordered::detail::default_bucket_count, hf, eq),
      m_cid(cid)
  { }

  ~unordered_multimap_base_container(void)
  {
    bc_base_impl::cleanup_map_elements<mapped_type>::apply(m_data);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a reference to the data
  //////////////////////////////////////////////////////////////////////
  container_type& container(void)
  {
    return m_data;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const reference to the data
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
  /// @brief Get the mapped value of the element with GID @p gid.
  /// @param gid The GID of type gid_type for the element.
  /// @return a pair containing <key_type,mapped_type>
  //////////////////////////////////////////////////////////////////////
  value_type get_element(gid_type const& gid)
  {
    typename container_type::iterator it = m_data.find(gid);
    stapl_assert(it != m_data.end(),
      "Tried to get an element that is not present in the unordered_multimap");
    return value_type(gid.first, it->second);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to modify the value of an existing element. An element with
  ///   an equivalent GID should already be in the container.
  /// @param v A pair of <gid_type,mapped_type>. The first element of the
  ///   pair is the multi_key of the element to be modified and the second
  ///   value is the value that will replace the previous value associated
  ///   with the multi_key.
  //////////////////////////////////////////////////////////////////////
  void set_element(value_type const& v)
  {
    typename container_type::iterator it = m_data.find(v.first);
    stapl_assert(it != m_data.end(),
      "Tried to set an element that is not present in the unordered_multimap");
    it->second = v.second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to modify the value of an existing element. An element with
  ///   an equivalent GID should already be in the container.
  /// @param gid The GID of the element to be modified.
  /// @param val The value that will replace the previous value associated
  ///   with @p gid.
  //////////////////////////////////////////////////////////////////////
  void set_element(gid_type const& gid, mapped_type const& val)
  {
    set_element(value_type(gid, val));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Assigns a value of mapped_type to the element with the
  ///   specified GID.
  /// @param gid The specified GID.
  /// @param f The mapped_type to apply
  /// @todo Is this specialized function useful ?
  //////////////////////////////////////////////////////////////////////
  void apply_set(gid_type const& gid, detail::set_second<mapped_type> const& f)
  {
    typename container_type::iterator it = m_data.find(gid);
    stapl_assert(it != m_data.end(),
      "Tried to set an element that is not present in the unordered_multimap");
    f(*it);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke the functor provided on the element with the specified GID.
  /// @param gid The specified GID.
  /// @param f The functor to apply
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  void apply_set(gid_type const& gid, F const& f)
  {
    typename container_type::iterator it = m_data.find(gid);
    stapl_assert(it != m_data.end(),
      "Tried to set an element that is not present in the unordered_multimap");
    f(*it);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Apply a function object to an element at a specific GID and
  /// return the result.
  /// @param gid The GID associated with the element for which we want to apply
  /// the functor and read the result.
  /// @param f The functor to apply to @p gid
  /// @return The result of applying f to the value at @p gid
  /// @warning This assumes that the type Functor reflects a public type named
  /// result_type and that the invocation of its function operator returns a
  /// value that is convertible to result_type. In addition, Functor's
  /// function operator must be const.
  //////////////////////////////////////////////////////////////////////
  template <typename Functor>
  typename Functor::result_type
  apply_get(gid_type const& gid, Functor const& f)
  {
    typename container_type::iterator it = m_data.find(gid);
    stapl_assert(it != m_data.end(),
      "Tried to get an element that is not present in the unordered_multimap");
    return f(*it);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to insert a <Multi_key,Mapped> pair in the container.
  /// @param val The pair of <multi_key<key_type>,mapped_type> to be inserted.
  /// @return return true if it was inserted
  //////////////////////////////////////////////////////////////////////
  bool insert(value_type const& v)
  {
    m_data.insert(v);
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
  /// @param gid The GID of type multi_key to be deleted.
  /// @return The number of elements erased.
  //////////////////////////////////////////////////////////////////////
  size_t erase_sync(gid_type const& gid)
  {
    return m_data.erase(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::unordered_map::make_reference(gid_type const& gid)
  //////////////////////////////////////////////////////////////////////
  reference make_reference(gid_type const& gid)
  {
    return reference(accessor_t(this, m_data.find(gid)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::unordered_map::make_iterator(gid_type const& gid)
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return iterator(m_data.find(gid),
                    const_cast<unordered_multimap_base_container*>(this));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::unordered_map::make_const_reference(gid_type const& gid)
  //////////////////////////////////////////////////////////////////////
  const_reference make_const_reference(gid_type const& gid) const
  {
    return const_reference(const_accessor_t(this, m_data.find(gid)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::unordered_map::make_const_iterator(gid_type const& gid)
  //////////////////////////////////////////////////////////////////////
  const_iterator make_const_iterator(gid_type const& gid) const
  {
    return const_iterator(m_data.find(gid), this);
  }

  iterator begin(void)
  {
    return iterator(m_data.begin(), this);
  }

  iterator end(void)
  {
    return iterator(m_data.end(), this);
  }

  const_iterator cbegin(void) const
  {
    return const_iterator(m_data.begin(), this);
  }

  const_iterator cend(void) const
  {
    return const_iterator(m_data.end(), this);
  }

  iterator find(gid_type const& gid)
  {
    return iterator(m_data.find(gid), this);
  }

  const_iterator find(gid_type const& gid) const
  {
    return const_iterator(m_data.find(gid), this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of elements in this base container.
  /// @return The size of type size_t
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return m_data.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return if the component is empty or not.
  /// @return a bool set to true if it is empty, false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return m_data.size() == 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clear the container by removing all the elements in it.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    m_data.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return the domain of the base container
  /// @return The domain of the base container
  //////////////////////////////////////////////////////////////////////
  domain_type domain(void) const
  {
    return domain_type(m_data);
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

}; // struct unordered_multimap_base_container

/////////////////////////////////////////////////////////////////////////
/// @brief Extracts the GID of the element pointed to by the iterator
/// @param it The target iterator
/// @return The value of the dereferenced iterator
/////////////////////////////////////////////////////////////////////////
template<typename Iter>
typename boost::unordered::iterator_detail::iterator<
           Iter
         >::value_type::first_type
gid_of(boost::unordered::iterator_detail::iterator<Iter> it)
{
  return (*it).first;
}

} // namespace stapl

#endif
