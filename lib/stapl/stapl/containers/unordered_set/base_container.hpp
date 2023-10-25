/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_SET_BASE_CONTAINER_HPP
#define STAPL_CONTAINERS_UNORDERED_SET_BASE_CONTAINER_HPP

#include <stapl/containers/base/bc_base.hpp>
#include <stapl/containers/iterators/const_local_iterator.hpp>
#include <stapl/containers/iterators/const_local_accessor.hpp>
#include <stapl/containers/map/functional.hpp>

namespace stapl {

template<typename Key, typename Traits>
struct unordered_set_base_container;

template<typename Key, typename Traits>
struct container_traits<unordered_set_base_container<Key, Traits> >
{
  typedef Key                                          key_type;
  typedef Key                                          mapped_type;
  typedef Key                                          gid_type;
  typedef typename Traits::container_type              container_type;
  typedef typename container_type::value_type          value_type;
  typedef typename Traits::domain_type                 domain_type;
  typedef local_accessor<
    unordered_set_base_container<Key, Traits>
  >                                                    accessor_t;
  typedef proxy<value_type, accessor_t>                reference;
  typedef const_local_accessor<
    unordered_set_base_container<Key, Traits>
  >                                                    const_accessor_t;
  typedef proxy<value_type, const_accessor_t>          const_reference;
};

//////////////////////////////////////////////////////////////////////
/// @brief The unordered set base container
/// @ingroup punorderedsetDist
/// @tparam Key Type of the key. Each element has a unique key value
/// @tparam Traits Traits type. Specifies the internal configuration of the
///   container.
/// @todo add move semantic constructor and assign operation.
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Traits>
struct unordered_set_base_container
  : public bc_base
{
  typedef typename Traits::container_type              container_type;
  typedef Key                                          key_type;
  typedef key_type                                     mapped_type;
  typedef typename container_type::value_type          value_type;
  typedef value_type                                   stored_type;
  typedef key_type                                     gid_type;
  typedef size_t                                       cid_type;
  typedef typename container_type::hasher              hasher;
  typedef typename container_type::key_equal           key_equal;
  typedef typename Traits::domain_type                 domain_type;

  typedef const_local_accessor<unordered_set_base_container> const_accessor_t;
  typedef proxy<value_type, const_accessor_t>                const_reference;
  typedef const_local_iterator<unordered_set_base_container> const_iterator;

  typedef local_accessor<unordered_set_base_container>       accessor_t;
  typedef proxy<value_type, accessor_t>                      reference;
  typedef local_iterator<unordered_set_base_container>       iterator;

protected:
  /// @brief Storage for elements assigned to the base container.
  /// The type of the storage is specified in the Traits parameter.
  container_type m_data;
  cid_type       m_cid;

public:
  unordered_set_base_container(
      hasher const& hf    = hasher(),
      key_equal const& eq = key_equal())
    : m_data(boost::unordered::detail::default_bucket_count,hf,eq),
      m_cid(0)
  { }

  unordered_set_base_container(
      cid_type cid,
      hasher const& hf    = hasher(),
      key_equal const& eq = key_equal())
    : m_data(boost::unordered::detail::default_bucket_count,hf,eq),
      m_cid(cid)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the base container ID
  //////////////////////////////////////////////////////////////////////
  cid_type const& cid(void) const
  {
    return m_cid;
  }

  value_type get_element(const key_type key)
  {
    typename container_type::iterator it = this->m_data.find(key);
    if (it!=this->m_data.end())
      return *it;
    stapl_assert(false, "key does not exist in set base container");
    return value_type();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to insert a key in the container.
  /// @param val The key to be inserted.
  /// @return return true if it was inserted
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
  /// @return The number of elements erased
  //////////////////////////////////////////////////////////////////////
  size_t erase_sync(key_type const& key)
  {
    return m_data.erase(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::unordered_set::make_reference(key_type const& key)
  //////////////////////////////////////////////////////////////////////
  reference make_reference(key_type const& k)
  {
    return reference(accessor_t(this,m_data.find(k)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an iterator that references the element whose GID is
  /// the specified @p key.
  /// @param key key value of the element to which an iterator will be
  /// created.
  /// @return iterator that can be used to access the element whose key
  /// value is @p key
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(key_type const& key)
  {
    return iterator(m_data.find(key),
                    const_cast<unordered_set_base_container*>(this));
  }

  iterator begin()
  {
    return iterator(m_data.begin(),
                    const_cast<unordered_set_base_container*>(this));
  }

  iterator end()
  {
    return iterator(m_data.end(),
                    const_cast<unordered_set_base_container*>(this));
  }

  iterator find(key_type const& key)
  {
    return iterator(m_data.find(key),
                    const_cast<unordered_set_base_container*>(this));
  }

  const_iterator find(key_type const& key) const
  {
    return const_iterator(m_data.find(key),
                          const_cast<unordered_set_base_container*>(this));
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
  size_t size() const
  {
    return this->m_data.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return if the component is empty or not.
  /// @return a bool set to true if it is empty, false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool empty() const
  {
    return (this->m_data.size()==0);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clear the container by removing all the elements in it.
  //////////////////////////////////////////////////////////////////////
  void clear()
  {
    this->m_data.clear();
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
  }

  domain_type domain(void) const
  {
    return domain_type(this->m_data);
  }

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element with specified GID.
  /// @param gid GID of the element on which to apply the functor
  /// @param f Functor to apply
  //////////////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void apply_set(gid_type const& gid, Functor const& f)
  {
    f(*(m_data.find(gid)));
  }

  template<typename Functor>
  typename Functor::result_type apply_get(gid_type const& gid, Functor const& f)
  {
    return f(*(m_data.find(gid)));
  }
}; // struct unordered_set_base_container

} // namespace stapl

#endif
