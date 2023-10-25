/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_LIST_DISTRIBUTION_HPP
#define STAPL_CONTAINERS_LIST_DISTRIBUTION_HPP

#include <stapl/containers/iterators/container_accessor.hpp>
#include <stapl/containers/iterators/const_container_accessor.hpp>
#include <stapl/containers/iterators/container_iterator.hpp>
#include <stapl/containers/iterators/const_container_iterator.hpp>

#include <stapl/domains/list_distributed_domain.hpp>

#include <stapl/containers/distribution/list_metadata.hpp>
#include <stapl/containers/distribution/distribution.hpp>

#include <stapl/containers/distribution/operations/base.hpp>

#include <stapl/views/proxy.h>

namespace stapl {

template <typename Container>
class list_distribution;


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for @ref distribution_traits over
///        @ref list_distribution.
//////////////////////////////////////////////////////////////////////
template <typename Container>
struct distribution_traits<list_distribution<Container> >
{
private:
  typedef distribution<Container>                                base_type;

public:
  STAPL_IMPORT_TYPE(typename container_traits<Container>, directory_type)

  typedef typename base_type::container_manager_type     container_manager_type;
  typedef typename container_manager_type::gid_type      gid_type;
  typedef typename container_manager_type::value_type    value_type;
  typedef typename container_manager_type::
                        base_container_type              base_container_type;

  typedef Container                                              container_type;
  typedef list_distributed_domain<list_distribution<Container> > domain_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines the distribution used for the parallel list
///        container.
///
/// @tparam Container Type of the container that uses this distribution.
//////////////////////////////////////////////////////////////////////
template<typename Container>
class list_distribution
  : public distribution<Container>,
    public operations::base<list_distribution<Container>>
{
  typedef distribution<Container>                        base_type;

public:

  typedef std::bidirectional_iterator_tag                iterator_category;

  typedef Container                                      container_type;
  typedef typename base_type::directory_type             directory_type;
  typedef typename base_type::container_manager_type     container_manager_type;
  typedef typename container_manager_type::
                        base_container_type              base_container_type;

  typedef typename container_manager_type::gid_type      gid_type;

  typedef gid_type                                       index_type;

  typedef typename container_manager_type::value_type    value_type;

  /// Distribution metadata type used for coarsening
  typedef list_metadata<list_distribution>               loc_dist_metadata;
  typedef typename loc_dist_metadata::dom_info_type      dom_info_type;

  typedef list_distributed_domain<list_distribution>     domain_type;

  typedef container_accessor<list_distribution>          accessor_type;
  typedef proxy<value_type, accessor_type>               reference;

  typedef const_container_accessor<list_distribution>    const_accessor_type;
  typedef proxy<value_type, const_accessor_type>         const_reference;

  typedef container_iterator<
    list_distribution, accessor_type,
    iterator_category
  >                                                      iterator;
  typedef const_container_iterator<
    list_distribution, const_accessor_type,
    iterator_category
  >                                                      const_iterator;

protected:
  domain_type    m_domain;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Remove the element with the specified gid.
  ///
  /// Used by @p pop_back, @p pop_front, and @p erase.
  //////////////////////////////////////////////////////////////////////
  void remove(gid_type const& gid)
  {
    if (this->get_location_id() == gid.m_location)
    {
      remove_local(gid);
      return;
    }

    // else
    this->directory().invoke_where(
      std::bind(
        [](p_object& d, gid_type const& gid)
          { down_cast<list_distribution>(d).remove_local(gid); },
        std::placeholders::_1, std::placeholders::_2),
      gid);
  }

public:
  list_distribution(void)
    : m_domain(*this)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a distribution with with the given @p directory
  ///        and base container manager @p bcmangr.
  ///
  /// @param directory Directory used by the container.
  /// @param bcmangr Base container manager used by the container.
  //////////////////////////////////////////////////////////////////////
  list_distribution(directory_type const& directory,
                    container_manager_type const& bcmangr)
    : base_type(directory, bcmangr),
      m_domain(*this)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy construction of this distribution.
  ///
  /// @param other Another distribution to copy from.
  //////////////////////////////////////////////////////////////////////
  list_distribution(list_distribution const& other)
    : base_type(other),
      m_domain(*this)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a distribution with default constructed elements.
  ///
  /// @param partition Partition used by the container.
  /// @param mapper Mapper used by the container.
  //////////////////////////////////////////////////////////////////////
  list_distribution(typename directory_type::partition_type const& partition,
                    typename directory_type::mapper_type const& mapper)
    : base_type(partition, mapper),
      m_domain(*this)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a distribution with an initial value for elements.
  ///
  /// @param partition Partition used by the container.
  /// @param mapper Mapper used by the container.
  /// @param default_value The value that the elements in this
  ///                      distribution will be initialized with.
  //////////////////////////////////////////////////////////////////////
  list_distribution(typename directory_type::partition_type const& partition,
                    typename directory_type::mapper_type const& mapper,
                    value_type const& default_value)
    : base_type(partition, mapper, default_value),
      m_domain(*this)
  { }

  list_distribution& operator=(list_distribution const& other)
  {
    if (this != &other)
    {
      base_type::operator=(other);
      m_domain = domain_type(*this);
    }
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::list::insert(iterator const&, value_type const&)
  //////////////////////////////////////////////////////////////////////
  iterator insert(iterator const& it, value_type const& value)
  {
    gid_type gid = it.index();

    if (gid.m_base_container == nullptr)
    {
      if (this->domain().size() == 0)
        gid.m_location = 0;
      else
        gid.m_location = this->get_num_locations() - 1;
    }

    if (this->container_manager().contains(gid))
    {
      insert_local(gid, value);
    }
    else
    {
      this->directory().invoke_where(
        std::bind(
          [](p_object& d, gid_type const& gid, value_type const& value)
            { down_cast<list_distribution>(d).insert_local(gid, value); },
          std::placeholders::_1, std::placeholders::_2, value),
        gid);
    }

    if (this->domain().size() == 1)
      return make_iterator(this->domain().first());
    else
      return --make_iterator(gid);
 }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::list::push_front(value_type const&)
  //////////////////////////////////////////////////////////////////////
  void push_front(value_type const& value)
  {
    gid_type gid;
    gid.m_location = 0;

    if (this->get_location_id() == gid.m_location)
      push_front_local(value);
    else {
      this->directory().invoke_where(
        std::bind(
          [](p_object& d, value_type const& value)
            { down_cast<list_distribution>(d).push_front_local(value); },
          std::placeholders::_1, value),
        gid);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::list::pop_front
  //////////////////////////////////////////////////////////////////////
  void pop_front(void)
  { this->remove(m_domain.first()); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::list::push_back(value_type const&)
  //////////////////////////////////////////////////////////////////////
  void push_back(value_type const& value)
  {
    gid_type gid;
    gid.m_location = this->get_num_locations() - 1;

    if (this->get_location_id() == gid.m_location) {
      push_back_local(value);
      return;
    }

    // else
    this->directory().invoke_where(
      std::bind(
        [](p_object& d, value_type const& value)
          { down_cast<list_distribution>(d).push_back_local(value); },
        std::placeholders::_1, value),
      gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::list::pop_back
  //////////////////////////////////////////////////////////////////////
  void pop_back(void)
  { this->remove(m_domain.last()); }


  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::list::erase(iterator const&)
  //////////////////////////////////////////////////////////////////////
  iterator erase(iterator pos)
  {
    this->remove(gid_of(pos++));
    return pos;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::list::clear(void)
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->m_container_manager.clear();
    this->directory().reset();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper method to insert the given @p value locally
  //////////////////////////////////////////////////////////////////////
  void insert_local(gid_type const& gid, value_type const& value)
  {
    this->m_container_manager.insert(gid, value);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper method to insert at front the given @p value locally
  //////////////////////////////////////////////////////////////////////
  void push_front_local(value_type const& value)
  {
    this->m_container_manager.add(value, true);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper method to insert at the end the given @p value locally
  //////////////////////////////////////////////////////////////////////
  void push_back_local(value_type const& value)
  {
    this->m_container_manager.add(value);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper method to remove the specified element.
  //////////////////////////////////////////////////////////////////////
  void remove_local(gid_type const& gid)
  {
    this->m_container_manager.remove(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a reference to the specified element in the list.
  /// @param gid GID of the element.
  ////////////////////////////////////////////////////////////////////////
  reference make_reference(gid_type const& gid)
  {
    return reference(accessor_type(this, gid));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an iterator pointing to the specified element
  ///        in the list.
  /// @param gid GID of the element.
  ////////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return iterator(this,gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a const_iterator pointing to the specified element
  ///        in the list.
  /// @param gid GID of the element.
  ////////////////////////////////////////////////////////////////////////
  const_iterator make_const_iterator(gid_type const& gid) const
  {
    return const_iterator(this,gid);
  }

  domain_type& domain(void)
  {
    return m_domain;
  }

  domain_type const& domain(void) const
  {
    return m_domain;
  }

  size_t size(void) const
  {
    return m_domain.size();
  }

  iterator begin(void)
  {
    return make_iterator(this->domain().first());
  }

  const_iterator begin(void) const
  {
    return make_const_iterator(this->domain().first());
  }

  const_iterator cbegin(void) const
  {
    return make_const_iterator(this->domain().first());
  }

  iterator end(void)
  {
    return ++(make_iterator(this->domain().last()));
  }

  const_iterator end(void) const
  {
    return ++(make_const_iterator(this->domain().last()));
  }

  const_iterator cend(void) const
  {
    return ++(make_const_iterator(this->domain().last()));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an iterator pointing to the specified element
  ///        in the list.
  /// @param gid GID of the element.
  /// @todo Replace the invocations of this method with make_iterator.
  ////////////////////////////////////////////////////////////////////////
  iterator find(gid_type const& gid)
  {
    return make_iterator(gid);
  }

  const_iterator find(gid_type const& gid) const
  {
    return make_const_iterator(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper method to set the provided @p promise with the
  ///        result of advancing @p n positions the given @p gid.
  //////////////////////////////////////////////////////////////////////
  void defer_advance(gid_type const& gid, long long n,
                     bool globally, promise<gid_type> p)
  {
    m_domain.defer_advance(gid, n, globally, std::move(p));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper method to set the provided @p promise with the
  ///        result of verifying the given @p gid is in this location.
  //////////////////////////////////////////////////////////////////////
  void defer_contains(gid_type const& gid, promise<bool> p)
  {
    p.set_value(this->container_manager().contains(gid));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper method to set the provided @p promise with the
  ///        result of computing the distance between the two given
  ///        gids (@p gida, @p gidb).
  //////////////////////////////////////////////////////////////////////
  void defer_distance(gid_type const& gida, gid_type const& gidb,
                      promise<size_t> p)
  {
    this->container_manager().defer_distance(gida, gidb, std::move(p));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper method to set the provided @p promise with the
  ///        result of verifying the given @p gid is in this location.
  /// @todo Verify if this method can be merged with defer_contains.
  //////////////////////////////////////////////////////////////////////
  void contains_helper(domain_type const& dom, gid_type const& gid,
                       promise<bool> p) const
  {
    dom.defer_contains(gid, std::move(p));
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the metadata associated with the given @p gid by
  ///        setting the value of the promise.
  ///
  /// @param gid Id of the element of interest
  /// @param p Promise that will return the locality information to the
  ///          location that invoked the method.
  //////////////////////////////////////////////////////////////////////
  void defer_metadata_at(gid_type const& gid, promise<dom_info_type> p)
  {
    typedef typename container_manager_type::const_iterator c_iter_t;

    c_iter_t cit     = this->container_manager().begin();
    c_iter_t cit_end = this->container_manager().end();

    for (; cit != cit_end; ++cit)
    {
      base_container_type* bc = *cit;
      if ((!bc->domain().empty()) && (bc->domain().contains(gid)))
      {
        typename dom_info_type::domain_type ndom(
          bc->domain().first(), bc->domain().last(), *this
        );
        p.set_value(dom_info_type(
          typename dom_info_type::cid_type(), // bc.cid()
          ndom, const_cast<base_container_type*>(bc),
          LQ_CERTAIN, get_affinity(),
          this->get_rmi_handle(), this->get_location_id()
        ));

        return;
      }
    }

    // The gid was not found.  Abort the execution.
    std::fprintf(stderr,
      "list_distribution::defer_metadata_at: GID not on location"
      "specified by directory.\n");
    std::exit(1);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the metadata associated with the given @p index.
  ///
  /// @todo Add const qualifier when constness has been properly propagated.
  //////////////////////////////////////////////////////////////////////
  future<dom_info_type>
  metadata_at(gid_type const& gid)
  {
    typedef typename container_manager_type::const_iterator c_iter_t;

    c_iter_t cit     = this->container_manager().begin();
    c_iter_t cit_end = this->container_manager().end();

    for (; cit != cit_end; ++cit)
    {
      base_container_type* bc = *cit;
      if ((!bc->domain().empty()) && (bc->domain().contains(gid)))
      {
        typename dom_info_type::domain_type ndom(
          bc->domain().first(), bc->domain().last(), *this);

        return make_ready_future(dom_info_type(
                 typename dom_info_type::cid_type(), // bc.cid()
                 ndom, const_cast<base_container_type*>(bc),
                 LQ_CERTAIN, get_affinity(),
                 this->get_rmi_handle(), this->get_location_id()));
      }
    }

    // Element was not found locally.  Retrieve the metadata from the location
    // at which the element is stored.
    typedef promise<dom_info_type> promise_type;

    promise_type p;
    auto f = p.get_future();

    this->directory().invoke_where(
      std::bind(
        [](p_object& d, gid_type const& gid, promise_type& p)
        {
          down_cast<list_distribution>(d).defer_metadata_at(gid, std::move(p));
        },
        std::placeholders::_1, std::placeholders::_2, std::move(p)),
      gid);

    return f;
  }

  size_t local_size(void) const
  {
    return domain().local_size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Method used only for debugging and testing purposes.
  //////////////////////////////////////////////////////////////////////
  void print(void)
  {
    this->m_container_manager.print();
  }
}; // class list_distribution

} // namespace stapl

#endif // STAPL_CONTAINERS_LIST_DISTRIBUTION_HPP
