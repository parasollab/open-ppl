/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MULTI_ASSOCIATIVE_CONTAINER_DISTRIBUTION_HPP
#define STAPL_CONTAINERS_MULTI_ASSOCIATIVE_CONTAINER_DISTRIBUTION_HPP

#include "associative_distribution.hpp"

namespace stapl {

template<typename Container, typename DerivedDistribution>
class multi_associative_distribution;

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref distribution_traits for
/// @ref multi_associative_distribution.
//////////////////////////////////////////////////////////////////////
template <typename C, typename D>
struct distribution_traits<multi_associative_distribution<C,D> >
{
  typedef C                                              container_type;
  typedef typename container_traits<C>::directory_type   directory_type;
  typedef
    typename container_traits<C>::container_manager_type container_manager_type;
  typedef typename container_traits<C>::gid_type         gid_type;
  typedef gid_type                                       index_type;
  typedef typename container_traits<C>::value_type       value_type;
  typedef
    typename container_manager_type::base_container_type base_container_type;
  typedef container_accessor<C>                          accessor_type;
  typedef proxy<value_type, accessor_type>               reference;
};

//////////////////////////////////////////////////////////////////////
/// @brief Distribution for multi-associative containers, such as @ref
///   multimap and @ref unordered_multimap.
/// @tparam Container Type of the container that is managing
/// @tparam DerivedDistribution The custom distribution of the container
////////////////////////////////////////////////////////////////////////
template<typename Container, typename DerivedDistribution>
class multi_associative_distribution
  : public associative_distribution<Container, DerivedDistribution>
{
  typedef associative_distribution<Container, DerivedDistribution> base_type;

public:
  typedef Container                                      container_type;
  typedef typename base_type::directory_type             directory_type;
  typedef typename base_type::container_manager_type     container_manager_type;
  typedef typename base_type::base_container_type        base_container_type;
  typedef typename base_type::key_type                   key_type;
  typedef typename base_type::value_type                 value_type;
  typedef typename base_type::gid_type                   gid_type;
  typedef gid_type                                       index_type;
  typedef typename base_container_type::stored_type      stored_type;

  typedef typename base_type::accessor_type              accessor_type;
  typedef typename base_type::reference                  reference;
  typedef typename base_type::iterator                   iterator;

  typedef typename base_type::const_accessor_type        const_accessor_type;
  typedef typename base_type::const_reference            const_reference;
  typedef typename base_type::const_iterator             const_iterator;

  typedef distributed_domain<multi_associative_distribution> domain_type;

protected:
  domain_type m_domain;

public:
  multi_associative_distribution(void)
    : m_domain(this)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy construction of this distribution.
  /// @param other Another distribution to copy from
  //////////////////////////////////////////////////////////////////////
  multi_associative_distribution(multi_associative_distribution const& other)
    : base_type(other), m_domain(*this)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with a directory and container manager.
  /// @param directory Directory for this distribution
  /// @param bcmngr Container manager for this distribution
  //////////////////////////////////////////////////////////////////////
  multi_associative_distribution(directory_type const& directory,
                                 container_manager_type const& bcmngr)
    : base_type(directory, bcmngr), m_domain(*this)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with a directory, container manager,
  ///   and a flag to prevent keys from being registered at construction.
  /// @param directory Directory for this distribution
  /// @param bcmngr Container manager for this distribution
  /// @param reg Prevents keys from being registered at construction
  //////////////////////////////////////////////////////////////////////
  multi_associative_distribution(directory_type const& directory,
                                 container_manager_type const& bcmngr,
                                 boost::mpl::false_ const& reg)
    : base_type(directory, bcmngr, reg), m_domain(*this)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with a partition and a mapper.
  /// @param partition Partition used by the container
  /// @param mapper Mapper used by the container
  //////////////////////////////////////////////////////////////////////
  template <typename PS, typename Map>
  multi_associative_distribution(PS const& partition, Map const& mapper)
    : base_type(partition, mapper), m_domain(*this)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determines where a key should be inserted and invokes the
  ///   helper function on that location.
  /// @param v The pair of <key_type,mapped_type> to be inserted.
  /// @param is_pair Indicates whether a map or a set is invoking the function.
  //////////////////////////////////////////////////////////////////////
  void insert(value_type const& v, bool const& is_pair = false)
  {
    key_type const& key = static_cast<DerivedDistribution*>(this)->get_key(v);
    location_type home = this->directory().key_mapper()(gid_type(key)).first;

    if (home == get_location_id())
    {
      insert_impl(v, is_pair);
      return;
    }

    async_rmi(home, this->get_rmi_handle(),
      &multi_associative_distribution::insert_impl, v, is_pair);
  }

  //-------------------------------------------
  // Domain interface

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a const reference to the domain
  /// @todo Evaluate the performance of returning a copy rather than a
  ///   reference; we would like to avoid direct access to data members.
  //////////////////////////////////////////////////////////////////////
  domain_type const& domain(void) const
  {
    return m_domain;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invokes the count helper function where the key @p key is located.
  /// @param key The target key
  /// @return The number of elements counted
  /// @todo Propagate constness and remove const_cast.
  //////////////////////////////////////////////////////////////////////
  size_t count(key_type const& key) const
  {
    if (this->directory().is_registered(gid_type(key)))
    {
      typedef promise<size_t> promise_type;

      promise_type p;
      auto f = p.get_future();

      const_cast<multi_associative_distribution*>(this)->directory().
        invoke_where(
          std::bind(
            [](p_object& d, key_type const& k, promise_type& p)
            { down_cast<DerivedDistribution&>(d).count_impl(k, std::move(p)); },
            std::placeholders::_1, std::placeholders::_2, std::move(p)),
          key);

      return f.get();
    }

    // else
    return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the largest multiplicity of the key @p key.
  /// @param key The target key
  /// @return A GID with the highest multiplicity of @p key or an invalid
  ///   GID if the key does not exist in the container.
  //////////////////////////////////////////////////////////////////////
  gid_type equal_range(key_type const& k)
  {
    gid_type gid = gid_type(k);
    while (this->directory().is_registered(gid))
      gid = gid_type(gid.first, gid.second+1);

    if (gid == gid_type(k))
      return gid_type();

    return gid_type(gid.first, gid.second-1);
  }

  /////////////////////////////////////////////////////////////////////
  /// @brief Invokes the erase helper function where the GID @p
  ///   gid is located.
  /// @param gid The target GID
  /////////////////////////////////////////////////////////////////////
  void erase(gid_type const& gid)
  {
    if (this->directory().is_registered(gid))
    {

      location_type home = this->directory().key_mapper()(gid);

      if (home == this->get_location_id())
      {
        erase_impl(gid);
        return;
      }

      async_rmi(home, this->get_rmi_handle(), &base_type::erase_impl, gid);
    }
  }

  /////////////////////////////////////////////////////////////////////
  /// @brief Invokes the erase_sync helper function where the GID @p
  ///   gid is located.
  /// @param gid The target GID
  /// @return The number of elements removed
  /////////////////////////////////////////////////////////////////////
  size_t erase_sync(gid_type const& gid)
  {
    if (this->directory().is_registered(gid))
    {
      typedef promise<size_t> promise_type;

      promise_type p;
      auto f = p.get_future();

      this->directory().invoke_where(
        std::bind(
          [](p_object& d, key_type const& k, promise_type& p)
          {
            down_cast<DerivedDistribution&>(d).erase_sync_impl(k, std::move(p));
          },
          std::placeholders::_1, std::placeholders::_2, std::move(p)),
        gid);

      return f.get();
    }
    return 0;
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts an element into the container
  /// @param v The pair of <key_type,mapped_type> to be inserted.
  /// @param is_pair Indicates whether the container is a set or map; true
  ///   for map and false for set.
  //////////////////////////////////////////////////////////////////////
  void insert_impl(value_type const& v, bool const& is_pair)
  {
    gid_type gid = static_cast<DerivedDistribution*>(this)->get_key(v);
    while (this->m_container_manager.contains(gid))
      gid = gid_type(gid.first, gid.second+1);

    if (is_pair)
      this->m_container_manager.insert(stored_type(gid, v.second));
    else
      this->m_container_manager.insert(gid);

    this->directory().register_key(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes all elements with key equivalent to that of @p g
  ///   asynchronously.
  /// @param g The GID to be removed.
  //////////////////////////////////////////////////////////////////////
  void erase_impl(gid_type const& g)
  {
    gid_type gid = g;
    stapl_assert(this->container_manager().contains(gid),
                 "called on non-local gid");
    while (this->container_manager().contains(gid))
    {
      this->directory().unregister_key(gid);
      this->m_container_manager.erase(gid);
      gid = gid_type(gid.first, gid.second+1);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes all elements with key @p k from the container
  /// @param k The key to be removed.
  /// @param p The promise that stores the number of elements erased.
  //////////////////////////////////////////////////////////////////////
  void erase_sync_impl(key_type const& k, promise<size_t> p)
  {
    gid_type gid = gid_type(k);
    stapl_assert(this->container_manager().contains(gid),
                 "called on non-local gid");
    size_t count = 0;
    while (this->container_manager().contains(gid))
    {
      this->directory().unregister_key(gid);
      count += this->m_container_manager.erase_sync(gid);
      gid = gid_type(gid.first, gid.second+1);
    }
    p.set_value(count);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Counts all elements with key @p k in the container
  /// @param k The key to be counted.
  /// @param p The promise that stores the number of elements counted.
  //////////////////////////////////////////////////////////////////////
  void count_impl(key_type const& k, promise<size_t> p)
  {
    gid_type gid = gid_type(k);
    stapl_assert(this->container_manager().contains(gid),
                 "called on non-local gid");
    size_t count = 0;
    while (this->container_manager().contains(gid))
    {
      ++count;
      gid = gid_type(gid.first, gid.second+1);
    }
    p.set_value(count);
  }
}; // class multi_associative_distribution

} // namespace stapl

#endif // STAPL_CONTAINERS_MULTI_ASSOCIATIVE_CONTAINER_DISTRIBUTION_HPP
