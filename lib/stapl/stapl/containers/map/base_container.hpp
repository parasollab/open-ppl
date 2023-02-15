/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MAP_BASE_CONTAINER_HPP
#define STAPL_CONTAINERS_MAP_BASE_CONTAINER_HPP

#include <stapl/containers/base/bc_base.hpp>
#include <stapl/domains/iterator_domain.hpp>
#include <stapl/containers/iterators/local_iterator.hpp>
#include <stapl/containers/iterators/const_local_iterator.hpp>
#include <stapl/containers/map/map_base_container_traits.hpp>
#include <stapl/containers/map/functional.hpp>

namespace stapl {

template<typename Traits>
struct map_base_container;


template<typename Traits>
struct container_traits<map_base_container<Traits>>
  : public Traits
{
private:
  using accessor_t       = local_accessor<map_base_container<Traits>>;
  using const_accessor_t = const_local_accessor<map_base_container<Traits>>;

public:
  using reference       = proxy<typename Traits::value_type, accessor_t>;
  using const_reference = reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief Base container for the @ref map container.
/// @tparam Key Type of the key. Each element has a unique key value
/// @tparam T Type of the mapped value. Each element stores data of the
///   mapped type.
/// @tparam Compare Comparator used for less_than equality of keys.
/// @tparam Traits Traits type, which specifies the internal configuration
///   of the base container.
/// @ingroup pmapDist
/// @todo Instance of the compare class should be passed to this class
/// @todo Add const reference and iterators, move constructor
//////////////////////////////////////////////////////////////////////
template<typename Traits>
struct map_base_container
  : public bc_base
{
private:
  typedef typename Traits::key_type     Key;
  typedef typename Traits::mapped_type  T;
  typedef typename Traits::compare_type Compare;

  typedef local_accessor<map_base_container>          accessor_t;
  typedef const_local_accessor<map_base_container>    const_accessor_t;
public:
  typedef typename define_value_type<T>::type         stored_type;
  typedef typename Traits::container_type             container_type;
  typedef Key                                         key_type;
  typedef T                                           mapped_type;
  typedef std::pair<const key_type, mapped_type>      value_type;
  typedef Compare                                     compare_type;
  typedef key_type                                    gid_type;
  typedef size_t                                      cid_type;
  typedef typename Traits::domain_type                domain_type;
  typedef proxy<value_type, accessor_t>               reference;
  typedef proxy<value_type, const_accessor_t>         const_reference;
  typedef local_iterator<map_base_container>          iterator;
  typedef const_local_iterator<map_base_container>    const_iterator;

private:
  typedef member_accessor<
    typename reference::template get_second<mapped_type>
>                                                   second_accessor_type;

public:
  typedef proxy<mapped_type, second_accessor_type>    second_reference;

protected:
  /// The underlying raw data
  container_type  m_data;

  /// This base container's id
  cid_type        m_cid;

public:
  map_base_container(cid_type const& cid)
    : m_cid(cid)
  { }

  ~map_base_container(void)
  {
    bc_base_impl::cleanup_map_elements<T>::apply(m_data);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the element corresponding to a specific GID. The element
  /// must be present in this base container.
  /// @param key The id associated with the element for which we want to read
  /// the value.
  /// @return A copy of the (key, value) pair.
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
  std::pair<const key_type, typename Traits::stored_type>
  get_stored_element(key_type const& key) const
  {
    typename container_type::const_iterator it = this->m_data.find(key);

    if (it!=this->m_data.end())
      return *it;

    stapl_assert(false, "key does not exist in map base container");

    return std::make_pair(key_type(), typename Traits::stored_type());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Update a (key, value) pair in this base container, or insert
  /// it if it does not exist.
  /// @param val A pair to update
  /// @todo Should this insert?
  //////////////////////////////////////////////////////////////////////
  void set_element(value_type const& val)
  {
    this->m_data[val.first] = val.second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Update a (key, value) pair in this base container, or insert
  /// it if it does not exist.
  /// @param key GID to update
  /// @param val Mapped value to update
  /// @todo Should this insert?
  //////////////////////////////////////////////////////////////////////
  void set_element(key_type const& key, mapped_type const& val)
  {
    set_element(value_type(key,val));
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

  template<typename F>
  typename boost::result_of<F(value_type&)>::type
  apply_get(key_type const& key, F const& f)
  {
    return f(*(m_data.find(key)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::map::operator[](key_type const& key)
  //////////////////////////////////////////////////////////////////////
  second_reference operator[](key_type const& key)
  {
    return this->make_reference(key).second;
  }

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
  /// @brief Construct an entry for @p gid if one does not exist
  ///  and return true.  If one does exist, do nothing and return false.
  /// @param key Key of the element that may be created
  /// @param comp_spec Optional parameter that can be provided when the
  /// element to be constructed is a container with a view-based distribution.
  //////////////////////////////////////////////////////////////////////
  bool try_create(key_type const& key, composed_dist_spec* comp_spec)
  {
    return m_data.insert(
      std::make_pair(key,
        comp_spec == nullptr ?
          bc_base_impl::default_construct_element<T>::apply() :
          bc_base_impl::construct_vbdist_element<T>::apply(key, comp_spec))
    ).second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::map_base_container::operator[](key_type const&)
  //////////////////////////////////////////////////////////////////////
  reference make_reference(key_type const& key)
  {
    return reference(accessor_t(this, m_data.find(key)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::map::make_iterator(key_type const&)
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(key_type const& key)
  {
    return iterator(m_data.find(key), this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a reference to the underlying storage
  //////////////////////////////////////////////////////////////////////
  container_type& container(void)
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
    return this->m_data.empty();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clear this base container by removing all of its elements.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->m_data.clear();
  }

  size_t erase(key_type const& key)
  {
    return m_data.erase(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Define serialization of the base container.
  //////////////////////////////////////////////////////////////////////
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
    return domain_type(m_data);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return this base container's ID.
  //////////////////////////////////////////////////////////////////////
  cid_type cid(void) const
  {
    return m_cid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Method to be removed. it is only for testing purposes
  //////////////////////////////////////////////////////////////////////
  void print(void)
  {
    typename container_type::const_iterator it = m_data.begin();

    for (;it!=m_data.end();++it)
    {
      std::cout << " (" << it->first <<"," << it->second <<")";
    }
    std::cout << std::endl;
  }
}; // class map_base_container

} // namespace stapl

#endif // STAPL_CONTAINERS_MAP_BASE_CONTAINER_HPP
