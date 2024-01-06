/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_MAP_BASE_CONTAINER_HPP
#define STAPL_CONTAINERS_UNORDERED_MAP_BASE_CONTAINER_HPP

#include <stapl/containers/base/bc_base.hpp>
#include <stapl/domains/iterator_domain.hpp>
#include <stapl/containers/iterators/local_iterator.hpp>
#include <stapl/containers/iterators/const_local_iterator.hpp>
#include <stapl/containers/map/functional.hpp>

namespace stapl {

template<typename Traits>
struct unordered_map_base_container;

template<typename Traits>
struct container_traits<unordered_map_base_container<Traits> >
  : public Traits
{
  typedef typename Traits::key_type                      key_type;
  typedef typename Traits::mapped_type                   mapped_type;
  typedef typename Traits::value_type                    value_type;
  typedef typename Traits::stored_type                   stored_type;
  typedef typename Traits::domain_type                   domain_type;

  typedef local_accessor<
    unordered_map_base_container<Traits>
  >                                                      accessor_t;
  typedef proxy<value_type, accessor_t>                  reference;
  typedef const_local_accessor<
    unordered_map_base_container<Traits>
  >                                                      const_accessor_t;
  typedef proxy<value_type, const_accessor_t>            const_reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief The unordered map base container
/// @ingroup punorderedmapDist
/// @tparam Key Type of the key. Each element has a unique key value
/// @tparam Mapped Type of the mapped value. Each element stores data of the
///   mapped type.
/// @tparam Traits Traits type. Specifies the internal configuration of the
///   container
/// @todo add a typedef for const iterator
/// @todo add move semantic constructor and assign operation.
/// @bug function gid_of. local_iterator promotion does not currently work
//////////////////////////////////////////////////////////////////////
template<typename Traits>
struct unordered_map_base_container
  : public bc_base
{
private:
  typedef typename Traits::key_type     Key;
  typedef typename Traits::mapped_type  Mapped;
  typedef typename Traits::compare_type Compare;

public:
  typedef typename Traits::container_type              container_type;
  typedef Key                                          key_type;
  typedef Mapped                                       mapped_type;
  typedef std::pair<const key_type, mapped_type>       value_type;
  typedef value_type                                   stored_type;
  typedef Compare                                      compare_type;
  typedef typename container_type::hasher              hasher;
  typedef typename container_type::key_equal           key_equal;
  typedef key_type                                     gid_type;
  typedef size_t                                       cid_type;
  typedef typename Traits::domain_type                 domain_type;

  typedef local_accessor<unordered_map_base_container>       accessor_t;
  typedef proxy<value_type,accessor_t>                       reference;
  typedef local_iterator<unordered_map_base_container>       iterator;

  typedef const_local_accessor<unordered_map_base_container> const_accessor_t;
  typedef proxy<value_type, const_accessor_t>                const_reference;
  typedef const_local_iterator<unordered_map_base_container> const_iterator;

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
  unordered_map_base_container(
    hasher const& hf    = hasher(),
    key_equal const& eq = key_equal())
    : m_data(boost::unordered::detail::default_bucket_count, hf, eq),
      m_cid(0)
  { }

  unordered_map_base_container(
    size_t cid,
    hasher const& hf = hasher(),
    key_equal const& eq = key_equal())
    : m_data(boost::unordered::detail::default_bucket_count, hf, eq),
      m_cid(cid)
  { }

  ~unordered_map_base_container(void)
  {
    bc_base_impl::cleanup_map_elements<mapped_type>::apply(m_data);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the base container ID
  //////////////////////////////////////////////////////////////////////
  cid_type const& cid(void) const
  {
    return m_cid;
  }

  // //////////////////////////////////////////////////////////////////////
  // /// @brief Method used to promote a native iterator to a global iterator.
  // /// @param it The native iterator
  // /// @return the global iterator
  // /// @bug local_iterator promotion does not currently work
  // //////////////////////////////////////////////////////////////////////
  // gid_type gid_of(iterator& it) const
  // {
  //   return gid_type(it->first, 0); // Error
  // }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the mapped value of the element that has a specific key.
  /// @param key The Key of type key_type for which we need the value.
  /// @return a pair containing <key_type,mapped_type>
  //////////////////////////////////////////////////////////////////////
  value_type get_element(key_type const& key)
  {
    return value_type(key, m_data[key]);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to modify the value of an existing element. An element with
  ///   an equivalent key should already be in the container.
  /// @param val A pair of <key_type,mapped_type>. The first element of the
  ///   pair is the key of the element to be modified, the second value is the
  ///   value that will replace the previous value associated with the key.
  //////////////////////////////////////////////////////////////////////
  void set_element(value_type const& val)
  {
    stapl_assert(m_data.find(val.first) != m_data.end(),
      "Tried to set an element that is not present in the unordered_map");

    m_data[val.first] = val.second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::unordered_map::set_element(key_type const& key, mapped_type const& val)
  //////////////////////////////////////////////////////////////////////
  void set_element(key_type const& key, mapped_type const& val)
  {
    set_element(value_type(key, val));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Assigns a value of mapped_type to the element with the
  ///   specified key.
  /// @param key The specified key.
  /// @param f The mapped_type to apply
  /// @todo Is this specialized function useful ?
  //////////////////////////////////////////////////////////////////////
  void apply_set(key_type const& key, detail::set_second<mapped_type> const& f)
  {
    f(*(m_data.find(key)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke the functor provided on the element with the specified key.
  /// @param key The specified key.
  /// @param f The functor to apply
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  void apply_set(key_type const& key, F const& f)
  {
    f(*(m_data.find(key)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Apply a function object to an element at a specific GID and
  /// return the result.
  /// @param key The GID associated with the element for which we want to apply
  /// the functor and read the result.
  /// @param f The functor to apply to gid
  /// @return The result of applying f to the value at gid
  /// @warning This assumes that the type Functor reflects a public type named
  /// result_type and that the invocation of its function operator returns a
  /// value that is convertible to result_type. In addition, Functor's
  /// function operator must be const.
  //////////////////////////////////////////////////////////////////////
  template <typename Functor>
  typename boost::result_of<Functor(value_type&)>::type
  apply_get(key_type const& key, Functor const& f)
  {
    return f(*(m_data.find(key)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::unordered_map::operator[](key_type const& key)
  //////////////////////////////////////////////////////////////////////
  second_reference operator[](key_type const& key)
  {
    return this->make_reference(key).second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to insert a <Key,Mapped> pair in the container.
  /// @param val The pair of <key_type,mapped_type> to be inserted.
  //////////////////////////////////////////////////////////////////////
  void insert(value_type const& val)
  {
    this->m_data.insert(val);
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
  /// @brief Construct a <key_type, mapped_type> pair with @p key and a
  ///   default constructed mapped value if one does not exist.
  /// @return A bool indicating whether a new element was inserted.
  //////////////////////////////////////////////////////////////////////
  bool try_create(gid_type const& gid, composed_dist_spec* comp_spec = nullptr)
  {
    return m_data.insert(
      std::make_pair(gid,
        bc_base_impl::default_construct_element<mapped_type>::apply())
    ).second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erase a key from the container asynchronously.
  /// @param key The Key of type key_type to be deleted.
  //////////////////////////////////////////////////////////////////////
  void erase(key_type const& key)
  {
    m_data.erase(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erase a key from the container.
  /// @param key The Key of type key_type to be deleted.
  /// @return The number of elements erased.
  //////////////////////////////////////////////////////////////////////
  size_t erase_sync(key_type const& key)
  {
    return m_data.erase(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::unordered_map::make_reference(key_type const& key)
  //////////////////////////////////////////////////////////////////////
  reference make_reference(key_type const& key)
  {
    return reference(accessor_t(this, m_data.find(key)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::unordered_map::make_iterator(key_type const& key)
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(key_type const& key)
  {
    return iterator(m_data.find(key), this);
  }

  iterator begin(void)
  {
    return iterator(m_data.begin(), this);
  }

  iterator end(void)
  {
    return iterator(m_data.end(), this);
  }

  iterator find(key_type const& key)
  {
    return iterator(m_data.find(key), this);
  }

  const_iterator find(key_type const& key) const
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

  domain_type domain(void) const
  {
    return domain_type(m_data);
  }
}; // struct unordered_map_base_container

} // namespace stapl

#endif
