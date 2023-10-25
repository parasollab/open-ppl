/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_SYSTEM_VIEW_HPP
#define STAPL_VIEWS_SYSTEM_VIEW_HPP

#include <stapl/domains/interval.hpp>
#include <stapl/views/distribution_spec_view.hpp>
#include <stapl/containers/distribution/specification_functors.hpp>
#include <stapl/views/proxy/trivial_accessor.hpp>
#include <stapl/views/metadata/extraction/generator.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility/typed_in_place_factory.hpp>
#include <vector>

namespace stapl {

namespace dist_view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Mapping from location id in the creator gang to a location id
/// in the created gang based on an explicit set of locations that will
/// be part of the created gang.
///
/// @tparam LocDom Type of the domain used for the explicit location id set.
//////////////////////////////////////////////////////////////////////
template <typename LocDom>
struct explicit_loc_mapping_function
{
private:
  LocDom m_locs;

public:
  explicit_loc_mapping_function(LocDom const& locs)
    : m_locs(locs)
  { }

  stapl::location_type operator()(stapl::location_type id) const
  { return m_locs.distance(m_locs.first(), id); }

  bool operator==(explicit_loc_mapping_function const& other) const
  { return m_locs == other.m_locs; }

  void define_type(typer& t)
  { t.member(m_locs); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Mapping from location id in the created gang to a location id
/// in the creator gang based on an explicit set of locations that will
/// be part of the created gang.
///
/// This mapping is the inverse of the mapping provided by
/// @ref explicit_loc_mapping_function.
///
/// @tparam LocDom Type of the domain used for the explicit location id set.
//////////////////////////////////////////////////////////////////////
template <typename LocDom>
struct explicit_loc_resolution_function
{
private:
  LocDom m_locs;

public:
  explicit_loc_resolution_function(LocDom const& locs)
    : m_locs(locs)
  { }

  stapl::location_type operator()(stapl::location_type id) const
  { return m_locs.advance(m_locs.first(), id); }

  bool operator==(explicit_loc_resolution_function const& other) const
  { return m_locs == other.m_locs; }

  void define_type(typer& t)
  { t.member(m_locs); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Basic @ref p_object used by @ref system_container to initialize
///  a gang over its set of locations.
//////////////////////////////////////////////////////////////////////
struct trivial_p_object
  : public p_object
{ };


struct system_container;


//////////////////////////////////////////////////////////////////////
/// @todo either remove metadata from system view (does it make sense?)
///  or properly compute affinity information.
//////////////////////////////////////////////////////////////////////
struct system_container_distribution
  : public p_object
{
  typedef indexed_domain<location_type>            domain_type;
  typedef system_container*                        component_type;
  typedef metadata_entry<
    domain_type, component_type, unsigned int
  >                                                dom_info_type;

  future<dom_info_type> metadata_at(size_t gid)
  {
    const unsigned int id = 0;

    return make_ready_future(dom_info_type(
             id, domain_type(location_type(gid), location_type(gid)), 0,
             LQ_CERTAIN, affinity_tag(), this->get_rmi_handle(), gid));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Container that represents the set of locations in the system.
///
/// The @ref identity_container was considered as an alternative to this
/// implementation, but lacked the finite size required to represent the
/// finite set of locations being used to execute an application.
/// @ingroup system_view
/// @internal
//////////////////////////////////////////////////////////////////////
struct system_container
{
  typedef location_type                                     value_type;
  typedef proxy<location_type,
            trivial_accessor<location_type> >               reference;
  // trivial_accessor provides read only.
  // reference and const_reference are the same type.
  typedef reference                                         const_reference;
  typedef domset1D<location_type>                           domain_type;
  typedef location_type                                     gid_type;
  typedef unsigned int                                      cid_type;
  typedef system_container_distribution                     distribution_type;
  typedef metadata::generator_extractor<system_container>   loc_dist_metadata;

private:
  location_type                                  m_size;
  distribution_type                              m_dist;
  level                                          m_level;
  boost::shared_ptr<std::vector<location_type>>  m_locs;
  boost::optional<trivial_p_object>              m_gang;

public:
  system_container(location_type nlocs)
    : m_size(nlocs), m_level(level::invalid), m_locs()
  { }

  system_container(level const& lvl)
    : m_size(0), m_level(lvl), m_locs()
  {
    stapl_assert(lvl == current_level || lvl == lowest_level,
     "unsupported level specifier in system container");
  }

  system_container(std::vector<location_type> const& locs)
    : m_size(locs.size()), m_level(level::invalid),
      m_locs(new std::vector<location_type>(locs))
  { }

  distribution_type& distribution(void)
  { return m_dist; }

  value_type get_element(gid_type const& index)
  {
    if (!m_size)
      m_size = stapl::get_num_locations();

    stapl_assert(index < m_size,
      "system_container::get_element received out of range index");
    if (!m_locs)
      return index;
    else
      return (*m_locs)[index];
  }

  const_reference operator[](gid_type const& index)
  {
    if (!m_size)
      m_size = stapl::get_num_locations();

    stapl_assert(index < m_size,
      "system_container::operator[] received out of range index");
    if (!m_locs)
      return reference(trivial_accessor<value_type>(index));
    else
      return reference(trivial_accessor<value_type>((*m_locs)[index]));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the element indexed.
  ///
  /// @param index of element to return
  /// @return element reference
  //////////////////////////////////////////////////////////////////////
  const_reference make_reference(gid_type const& index)
  {
    return this->operator[](index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  size_t version(void) const
  {
    return 0;
  }

  size_t size(void) const
  {
    stapl_assert(m_size, "system_container uninitialized when size called");
    return m_size;
  }

  bool explicit_locs(void) const
  { return m_locs != nullptr; }

  std::vector<location_type> const& get_locs(void) const
  {
    if (!this->explicit_locs())
      stapl::abort("set of locations in system view is not set");

    return *m_locs;
  }

  bool level_specified(void) const
  { return m_level != level::invalid; }

  level level_spec(void) const
  { return m_level; }

  domain_type domain(void) const
  {
    if (!m_locs)
      return domain_type(0, m_size-1);
    else
    {
      if (m_locs->back() - m_locs->front() + 1 == m_size)
        return domain_type(m_locs->front(), m_locs->back());
      else
      {
        domain_type d;
        for (auto loc : *m_locs)
          d += loc;
        return d;
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a @ref gang representing the locations this system
  ///   container instance refers to, on demand and return a reference
  ///   to an @ref p_object in that communication group, so that other
  ///   p_objects can be initialized within it by utilizing gang switching.
  //////////////////////////////////////////////////////////////////////
  p_object const& group(void)
  {
    if (level_specified())
    {
       if (m_level == current_level)
       {
         if (!m_gang)
           m_gang = boost::in_place<trivial_p_object>();

         return *m_gang;
       }

       // else
       if (m_level == lowest_level)
       {
         if (!m_gang)
         {
           gang g;
           m_gang = boost::in_place<trivial_p_object>();
         }

        return *m_gang;
       }
    }

    stapl_assert(this->domain().contains(get_location_id()),
      "location shouldn't be participating in sub gang creation");

    if (!m_gang)
    {
      domain_type loc_domain = this->domain();

      gang g(loc_domain.size(),
             explicit_loc_mapping_function<domain_type>(loc_domain),
             explicit_loc_resolution_function<domain_type>(loc_domain));

      m_gang = boost::in_place<trivial_p_object>();
    }

    return *m_gang;
  }

  template <typename F>
  typename F::result_type apply_get(gid_type const& gid, F const& f) const
  {
    if (!m_locs)
      return f(gid);
    else
      return f((*m_locs)[gid]);
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.member(m_size);
    t.member(m_locs);
  }
};

} // namespace dist_view_impl


//////////////////////////////////////////////////////////////////////
/// @brief Creates a read-only view representing the set of locations
/// in the system.
///
/// @return A system_view that represents the locations of the system
///         used in the current computation.
/// @ingroup system_view
//////////////////////////////////////////////////////////////////////
distribution_spec_view<dist_view_impl::system_container,
  indexed_domain<location_type>, location_type, location_type>*
system_view(location_type nlocs = get_num_locations());


//////////////////////////////////////////////////////////////////////
/// @brief Creates a read-only view representing a set of locations
/// in the system.
///
/// @return A system_view that represents the subset of the system whose
///         location ids are specified in the vector provided.
/// @ingroup system_view
//////////////////////////////////////////////////////////////////////
distribution_spec_view<dist_view_impl::system_container,
  indexed_domain<location_type>, location_type, location_type>*
system_view(std::vector<location_type> const& locs);


//////////////////////////////////////////////////////////////////////
/// @brief Creates a read-only view representing a set of locations
/// in the system.
///
/// @return A system_view that represents the subset of the system whose
///         locations are contained in the next lower level of the
///         system hierarchy.
/// @ingroup system_view
//////////////////////////////////////////////////////////////////////
distribution_spec_view<dist_view_impl::system_container,
  indexed_domain<location_type>, location_type, location_type>*
system_view(level const lvl);

//////////////////////////////////////////////////////////////////////
/// @brief Creates a read-only view representing a set of locations
/// in the system.
///
/// @tparam ContainerGID Type of GID in the container to be constructed
///         using a view-based specification of which the system_view
///         being constructed is the base.
/// @tparam ContainerCID Type of the partition id in the container to be
///         constructed using a view-based specification of which the
///         system_view being constructed is the base.
///
/// @return A system_view that represents the subset of the system whose
///         locations are contained in the next lower level of the
///         system hierarchy.
/// @ingroup system_view
//////////////////////////////////////////////////////////////////////
template <typename ContainerGID, typename ContainerCID>
distribution_spec_view<dist_view_impl::system_container,
  indexed_domain<location_type>, ContainerGID, ContainerCID>*
system_view(level const lvl)
{
  typedef distribution_spec_view<dist_view_impl::system_container,
            indexed_domain<location_type>, ContainerGID, ContainerCID> type;
  return new type(new dist_view_impl::system_container(lvl),
    indexed_domain<location_type>(0),
    identity_map<location_type, location_type, ContainerGID, ContainerCID>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates a read-only view representing a set of locations
/// in the system using an external @ref system_container.
///
/// @tparam ContainerGID Type of GID in the container to be constructed
///         using a view-based specification of which the system_view
///         being constructed is the base.
/// @tparam ContainerCID Type of the partition id in the container to be
///         constructed using a view-based specification of which the
///         system_view being constructed is the base.
/// @param  sys_ct System container initialized and managed externally.
///
/// @return A system_view that represents the subset of the system whose
///         locations are contained in the next lower level of the
///         system hierarchy.
/// @ingroup system_view
//////////////////////////////////////////////////////////////////////
template <typename ContainerGID, typename ContainerCID>
distribution_spec_view<dist_view_impl::system_container,
  indexed_domain<location_type>, ContainerGID, ContainerCID>*
system_view(dist_view_impl::system_container& sys_ct)
{
  typedef distribution_spec_view<dist_view_impl::system_container,
            indexed_domain<location_type>, ContainerGID, ContainerCID> type;
  return new type(
    sys_ct,
    indexed_domain<location_type>(0),
    identity_map<location_type, location_type, ContainerGID, ContainerCID>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates a read-only view representing a set of @c locs
/// in the system using an external @ref system_container.
///
/// @tparam ContainerGID Type of GID in the container to be constructed
///         using a view-based specification of which the system_view
///         being constructed is the base.
/// @tparam ContainerCID Type of the partition id in the container to be
///         constructed using a view-based specification of which the
///         system_view being constructed is the base.
/// @param  locs Explicit set of locations in the system view.
///
/// @return A system_view that represents the subset of the system whose
///         locations are contained in the next lower level of the
///         system hierarchy.
/// @ingroup system_view
//////////////////////////////////////////////////////////////////////
template <typename ContainerGID, typename ContainerCID>
distribution_spec_view<dist_view_impl::system_container,
  indexed_domain<location_type>, ContainerGID, ContainerCID>*
system_view(std::vector<location_type> const& locs)
{
  typedef distribution_spec_view<dist_view_impl::system_container,
            indexed_domain<location_type>, ContainerGID, ContainerCID> type;
  return new type(
    new dist_view_impl::system_container(locs),
    indexed_domain<location_type>(0),
    identity_map<location_type, location_type, ContainerGID, ContainerCID>());
}



//////////////////////////////////////////////////////////////////////
/// @brief Creates a read-only view whose domain is the identifiers of
/// the partitions of an element partitioning and whose mapping function
/// maps from partition id to location id.
///
/// The view is defined over a view that represents the set of
/// locations on to which this view will map partitions.  The set of
/// operations on the view is currently empty and will be expanded as useful
/// operations are identified.
///
/// @note The system view must have a lifetime that exceeds the mapping_view.
///
/// @param system view over the system on to which parition ids will be mapped
/// @param domain set of partition ids to be mapped to locations.
/// @param mapping_func Unary functor that receives a partition id
/// and returns a location id.
/// @return A view capable of mapping partition ids to locations.
/// @ingroup mapping_view
//////////////////////////////////////////////////////////////////////
template <typename SysView, typename PartitionIds, typename MappingFunction,
          typename GID = typename PartitionIds::index_type>
distribution_spec_view<SysView, PartitionIds, GID,
  typename PartitionIds::index_type>
mapping_view(SysView& system, PartitionIds const& domain,
             MappingFunction const& mapping_func)
{
  return
    distribution_spec_view<SysView, PartitionIds, GID,
      typename PartitionIds::index_type>(&system, domain, mapping_func);
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates a read-only view whose domain is the identifiers of
/// a container's elements and whose mapping function maps from element
/// GID to partition id.
///
/// When the partitioning_view is defined over a mapping_view the result
/// is a view which defines the distribution of container elements across
/// the locations of the system.
///
/// @param mapping_view view that will map the partition ids to locations
/// @param domain Set of element GIDs to be partitioned.
/// @param partitioning_func Unary functor that receives a GID and returns
/// a partition id.
/// @return A view capable of partitioning GIDs of container elements.
/// @ingroup partitioning_view
//////////////////////////////////////////////////////////////////////
template <typename MappingView, typename GIDDomain,
          typename PartitioningFunction>
distribution_spec_view<MappingView, GIDDomain, typename GIDDomain::index_type,
  typename MappingView::index_type>
partitioning_view(MappingView& mapping_view, GIDDomain const& domain,
                  PartitioningFunction const& partitioning_func)
{
  return distribution_spec_view<
    MappingView, GIDDomain,
    typename GIDDomain::index_type, typename MappingView::index_type
   >(&mapping_view, domain, partitioning_func);
}

} // namespace stapl

#endif // STAPL_VIEWS_SYSTEM_VIEW_HPP
