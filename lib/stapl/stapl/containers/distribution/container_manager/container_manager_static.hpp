/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MANAGER_STATIC_HPP
#define STAPL_CONTAINERS_MANAGER_STATIC_HPP

#include <boost/utility/typed_in_place_factory.hpp>

#include <stapl/domains/partitioned_domain.hpp>
#include <stapl/utility/invoke_arg.hpp>
#include "local_partition_info_fwd.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Container manager for static containers that disallow migration
/// and allows at most one base container per location.
///
/// @tparam BaseContainer The base container class.
///
/// @todo m_base_container is mutable right now because of incorrect
///   definition of const_iterator.
//////////////////////////////////////////////////////////////////////
template<typename BaseContainer>
class container_manager_static
{
  //////////////////////////////////////////////////////////////////////
  /// @brief Return the domain and component id of the base container
  ///        in the case where the partition is balanced.
  ///
  /// @param partition Partitions the elements of the container into partitions
  ///        that are stored in base containers.
  /// @param mapper Maps partitions to locations.
  /// @return a pair of the cid and domain of the base container.
  ///
  /// This specialization is needed to prevent the static_array used to store
  /// partial domain information in @ref get_local_partition_info from needing
  /// to invoke the algorithm itself.
  //////////////////////////////////////////////////////////////////////
  template <typename Partition, typename Mapper>
  std::pair<typename Mapper::cid_type, typename Partition::value_type>
  bc_domain(Partition const& partition, Mapper const& mapper,
            typename std::enable_if<std::is_same<
                Partition, balanced_partition<typename Partition::value_type>
             >::value>::type* = 0)
  {
    typedef typename Partition::value_type domain_type;
    typedef partitioned_domain<domain_type> part_dom_type;
    part_dom_type part_dom(partition.global_domain());

    typename part_dom_type::subdomains_view_type subdomains =
      part_dom.local_subdomains();
    stapl_assert(subdomains.size() == 1,
      "container_manager_static::init requires balanced partition.");
    return
      std::make_pair(typename Mapper::cid_type(get_location_id()),
                     subdomains[0]);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return the domain and component id of the base container
  ///        in instances where the partition is not balanced.
  ///
  /// @param partition Partitions the elements of the container into partitions
  ///        that are stored in base containers.
  /// @param mapper Maps partitions to locations.
  /// @return a pair of the cid and domain of the base container.
  ///
  /// This function is called when the pContainer partition isn't balanced,
  /// and communication is required to get each location the domain of the
  /// base container it must create.
  //////////////////////////////////////////////////////////////////////
  template <typename Partition, typename Mapper>
  std::pair<typename Mapper::cid_type, typename Partition::value_type>
  bc_domain(Partition const& partition, Mapper const& mapper,
            typename std::enable_if<!std::is_same<
              Partition, balanced_partition<typename Partition::value_type>
            >::value>::type* = 0)
  {
    // get local base container ids
    typedef typename Partition::value_type                  domain_type;
    typedef typename domain_type::index_type                index_type;
    typedef typename boost::icl::interval<index_type>::type interval_type;

    typedef std::tuple<
      boost::icl::interval_set<index_type>, typename Mapper::cid_type,
      location_type
    >  bc_info_type;

    std::vector<bc_info_type> bc_info =
      cm_impl::get_local_partition_info(
        cm_impl::get_partial_partition_info(partition, mapper));

    if (!bc_info.empty())
    {
      stapl_assert(bc_info.size() == 1,
        "container_manager_static requires one base container per location.");
      stapl_assert(get<0>(bc_info[0]).iterative_size() == 1,
        "base containers cannot be created for disjoint domains");
      interval_type interval(*get<0>(bc_info[0]).begin());
      return
        std::make_pair(get<1>(bc_info[0]),
          domain_type(boost::icl::first(interval), boost::icl::last(interval)));
    }
    else
    {
      index_type invalid = index_bounds<index_type>::invalid();
      return
        std::make_pair(typename Mapper::cid_type(get_location_id()),
                       typename Partition::value_type(invalid, invalid));
    }

  }

  typedef BaseContainer                component_type;

public:
  STAPL_IMPORT_TYPE(typename BaseContainer, cid_type)
  STAPL_IMPORT_TYPE(typename BaseContainer, gid_type)
  STAPL_IMPORT_TYPE(typename BaseContainer, value_type)

  typedef BaseContainer                base_container_type;

  typedef BaseContainer*               iterator;
  typedef BaseContainer const*         const_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Instantiate the base container that should be on this
  /// location based on partition and mapping information.
  ///
  /// @param partition The container's partition object.
  /// @param mapper The container's mapper object.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper>
  container_manager_static(Partition const& partition, Mapper const& mapper)
  {
    std::pair<typename Mapper::cid_type, typename Partition::value_type> bc_info
      = bc_domain(partition, mapper);

    m_base_container =
      boost::in_place<BaseContainer>(bc_info.second, bc_info.first);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Instantiate the base container that should be on this
  /// location based on partition and mapping information and populate the
  /// container with a default value.
  ///
  /// @param partition The container's partition object
  /// @param mapper The container's mapper object.
  /// @param default_value The value that should initialize the base containers.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper>
  container_manager_static(Partition const& partition,
                           Mapper const& mapper,
                           value_type const& default_value)
  {
    std::pair<typename Mapper::cid_type, typename Partition::value_type> bc_info
      = bc_domain(partition, mapper);

    m_base_container =
      boost::in_place<BaseContainer>(bc_info.second, bc_info.first,
                                     default_value);
  }

protected:
  /// @brief The single base container on this location.
  /// Use optional<> for corner case when n < p.
  boost::optional<BaseContainer> m_base_container;


  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::clear
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    m_base_container->clear();
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::contains
  //////////////////////////////////////////////////////////////////////
  bool contains(gid_type const& gid) const
  {
    return m_base_container->domain().contains(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copybrief container_manager::add_element
  /// @warning It is not valid to add elements to a static base container
  //////////////////////////////////////////////////////////////////////
  void add_element(gid_type const&, value_type const&)
  {
    stapl_assert(false,
      "trying to add an element to a static base container manager");
  }

  //////////////////////////////////////////////////////////////////////
  /// @copybrief container_manager::remove_element
  /// @warning It is not valid to remove elements to a static base container
  //////////////////////////////////////////////////////////////////////
  void remove_element(gid_type const&)
  {
    stapl_assert(false,
      "trying to remove an element from a static base container manager");
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::within
  //////////////////////////////////////////////////////////////////////
  cid_type within(gid_type const&) const
  {
    return m_base_container->cid();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of base containers on the current location.
  ///
  /// Returns 1, as container_manager_static always manages one base
  /// container per location.
  //////////////////////////////////////////////////////////////////////
  size_t constexpr size(void) const
  {
    return 1;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::begin
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return m_base_container.get_ptr();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::end
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return m_base_container.get_ptr() + 1;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::begin
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return m_base_container.get_ptr();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::end
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return m_base_container.get_ptr() + 1;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::apply_set
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void apply_set(gid_type const& gid, Functor const& f)
  {
    stapl_assert(contains(gid),
      "invoking a function on an unknown base container");

    m_base_container->apply_set(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::contains_apply_set
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  bool contains_apply_set(gid_type const& gid, Functor const& f)
  {
    if (!contains(gid))
     return false;

    m_base_container->apply_set(gid, f);

    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::apply_get
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type
  apply_get(gid_type const& gid, Functor const& f)
  {
    stapl_assert(contains(gid),
      "invoking a function on an unknown base container");

    return m_base_container->apply_get(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::apply_get
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type
  apply_get(gid_type const& gid, Functor const& f) const
  {
    stapl_assert(contains(gid),
      "invoking a function on an unknown base container");

    return m_base_container->apply_get(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::contains_apply_get
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  boost::optional<typename Functor::result_type>
  contains_apply_get(gid_type const& gid, Functor const& f)
  {
    typedef boost::optional<typename Functor::result_type> return_t;

    if (!contains(gid))
      return return_t();

    // else
    return return_t(m_base_container->apply_get(gid, f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc container_manager::contains_apply_get
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  boost::optional<typename Functor::result_type>
  contains_apply_get(gid_type const& gid, Functor const& f) const
  {
    typedef boost::optional<typename Functor::result_type> return_t;

    if (!contains(gid))
      return return_t();

    // else
    return return_t(m_base_container->apply_get(gid, f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a base container method on a certain GID. The
  ///   element must exist in the current base container manager.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  //////////////////////////////////////////////////////////////////////
  template<typename C, typename... T, typename... U>
  void invoke(gid_type const& gid, void (C::* const& pmf)(T...), U&&... u)
  {
    stapl_assert(contains(gid), "failed to find gid in container manager");

    (m_base_container.get_ptr()->*pmf)(std::forward<U>(u)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invokes a base container method on the given @p gid if it is
  ///        present on this location.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param u   Arguments to pass to the member function.
  /// @return True if gid was found and functor applied, otherwise false.
  //////////////////////////////////////////////////////////////////////
  template<typename C, typename... T, typename... U>
  bool
  contains_invoke(gid_type const& gid, void (C::* const& pmf)(T...), U&&... u)
  {
    if (!contains(gid))
      return false;

    (m_base_container.get_ptr()->*pmf)(std::forward<U>(u)...);

    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a base container method on a certain GID and return
  /// the result. The element must exist in the current base container manager.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param u   Arguments to pass to the member function.
  /// @return    The result of invoking the function pointer.
  //////////////////////////////////////////////////////////////////////
  template<typename C, typename Rtn, typename... T, typename... U>
  Rtn invoke(gid_type const& gid, Rtn (C::* const& pmf)(T...), U&&... u)
  {
    stapl_assert(contains(gid), "failed to find gid in container manager");

    return (m_base_container.get_ptr()->*pmf)(std::forward<U>(u)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invokes a base container method on the given @p gid if it is
  ///        present on this location.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param u   Arguments to pass to the member function.
  /// @return boost::optional with result of invocation if element was found.
  //////////////////////////////////////////////////////////////////////
  template<typename C, typename Rtn, typename... T, typename... U>
  boost::optional<Rtn>
  contains_invoke(gid_type const& gid, Rtn (C::* const& pmf)(T...), U&&... u)
  {
    if (!contains(gid))
      return boost::optional<Rtn>();

    return boost::optional<Rtn>(
      (m_base_container.get_ptr()->*pmf)(std::forward<U>(u)...));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a const base container method on a certain GID and return
  /// the result. The element must exist in the current base container manager.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param u   Arguments to pass to the member function.
  /// @return    The result of invoking the function pointer.
  //////////////////////////////////////////////////////////////////////
  template<typename C, typename Rtn, typename... T, typename... U>
  Rtn
  const_invoke(gid_type const& gid, Rtn (C::* const& pmf)(T...), U&&... u) const
  {
    stapl_assert(contains(gid), "failed to find gid in container manager");

    return (m_base_container.get_ptr()->*pmf)(std::forward<U>(u)...);
  }
}; // class container_manager_static

} // namespace stapl

#endif // STAPL_CONTAINERS_MANAGER_STATIC_HPP
