/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SINGLE_CONTAINER_REGISTRY_HPP
#define STAPL_CONTAINERS_SINGLE_CONTAINER_REGISTRY_HPP

#include <boost/optional/optional.hpp>
#include <boost/functional/factory.hpp>

#include "../ordering/base_container_ordering.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Base container storage for pContainers that are guaranteed to
///        have only one base container per location.
///
/// @tparam Container Type of the base container.
/// @todo Replace the base container pointer with a boost::optional to
///       have the container in the stack frame instead of heap allocated.
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct single_container_registry
{
public:
  STAPL_IMPORT_TYPE(typename Container, gid_type)
  typedef Container                     base_container_type;
  typedef Container*                    iterator;
  typedef Container const*              const_iterator;

protected:
  typedef boost::factory<Container*>    factory_type;

private:
   iterator m_base_container;

public:
  single_container_registry(void)
    : m_base_container(nullptr)
  { }

  ~single_container_registry(void)
  {
    delete m_base_container;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::begin
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return m_base_container;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::end
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return m_base_container ? m_base_container + 1 : m_base_container;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::begin
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return m_base_container;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::end
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return m_base_container ? m_base_container + 1 : m_base_container;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the base container that contains the
  ///        given GID. If not found, it returns an end iterator.
  ///
  /// @param gid GID for the element in question.
  //////////////////////////////////////////////////////////////////////
  iterator find(gid_type const& gid)
  {
    if (m_base_container && m_base_container->domain().contains(gid))
      return this->begin();
    else
      return this->end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the base container that contains the
  ///        given GID. If not found, it returns an end iterator.
  ///
  /// @param gid GID for the element in question.
  //////////////////////////////////////////////////////////////////////
  const_iterator find(gid_type const& gid) const
  {
    if (m_base_container && m_base_container->domain().contains(gid))
      return this->begin();
    else
      return this->end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the base container that contains the
  ///        given GID. If not found, it aborts in debug mode and is undefined
  ///        behavior in non-debug mode.
  ///
  /// @param gid GID for the element in question.
  //////////////////////////////////////////////////////////////////////
  iterator find_expect(gid_type const& gid)
  {
    stapl_assert(m_base_container && m_base_container->domain().contains(gid),
      "GID expected in this container manager, but is not found.");
    
    return this->begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the base container that contains the
  ///        given GID. If not found, it aborts in debug mode and is undefined
  ///        behavior in non-debug mode.
  ///
  /// @param gid GID for the element in question.
  //////////////////////////////////////////////////////////////////////
  const_iterator find_expect(gid_type const& gid) const
  {
    stapl_assert(m_base_container && m_base_container->domain().contains(gid),
      "GID expected in this container manager, but is not found.");
    
    return this->begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns number of the local base containers in this base
  ///        container manager storage.
  //////////////////////////////////////////////////////////////////////
  size_t size(void)
  {
    return m_base_container == nullptr ? 0 : 1;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::clear
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    if (m_base_container)
      m_base_container->clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a base container that is responsible for a given
  ///        domain through the use of a factory and inserts it into the
  ///        registry.
  ///
  /// @param dom Domain of the base container. Currently unused.
  /// @param f Factory that when invoked, returns a pointer to a newly
  ///          created base container. The registry assumes ownership of
  ///          the pointer.
  /// @return Pointer to the base container that was added
  //////////////////////////////////////////////////////////////////////
  template<typename Domain, typename Factory>
  base_container_type* insert_range(Domain const& dom, Factory const& f)
  {
    m_base_container = f();
    return this->begin();
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Clone the base container and apply a function that takes the
  ///        old base container and the newly created one.
  /// @param other The other instance of the registry to clone.
  /// @param f Function that is applied after cloning that accepts the old
  ///          base container and then the new one.
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  void clone_apply(single_container_registry const& other, F const& f)
  {
    if (m_base_container != nullptr)
      delete m_base_container;

    m_base_container = new base_container_type(*other.m_base_container);

    f(const_cast<iterator>(other.begin()), this->begin());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Find the domain and component id of the base container that
  ///        will be created for the pContainer on this location.
  ///
  /// @param partition instance of @ref balanced_partition that partitions
  ///        GIDs into the domains of the base containers.
  /// @param mapper Maps partitions to locations.
  /// @return a pair of the cid and domain of the base container.
  ///
  /// This specialization for @ref balanced_partition is able to construct
  /// the base containers without communication, and as a result is faster
  /// than the general algorithm implemented in the non-specialized bc_domain
  /// method.
  //////////////////////////////////////////////////////////////////////
  template <typename Partition, typename Mapper, typename BC_Factory>
  void init(Partition const& partition, Mapper const& mapper,
            BC_Factory const& bc_factory, base_container_ordering& ordering,
            typename std::enable_if<
              std::is_same<
                Partition, balanced_partition<typename Partition::value_type>
              >::value>::type* = 0)
  {
    typedef typename Partition::value_type::index_type index_type;

    typename Mapper::cid_type cid         = get_location_id();
    typename Partition::value_type domain = partition[cid];

    base_container_type* const bc =
      insert_range(domain, [&]() { return bc_factory(domain, cid); });

    ordering.insert_after(0, bc, cid);
    ordering.m_is_ordered   = true;
    ordering.m_total_num_bc =
      domain.empty() ? get_num_locations() : partition.size();
  }
}; // struct single_container_registry

} // namespace stapl

#endif // STAPL_CONTAINERS_SINGLE_CONTAINER_REGISTRY_HPP
