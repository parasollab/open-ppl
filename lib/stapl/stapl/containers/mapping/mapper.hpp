/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MAPPER_HPP
#define STAPL_CONTAINERS_MAPPER_HPP

#include <stapl/views/mapping_functions/mapping_functions.hpp>
#include <stapl/views/array_ro_view.hpp>

#include <stapl/domains/indexed.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/generators/identity.hpp>

#include <stapl/utility/use_default.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief A container's mapper is responsible for mapping base container IDs
/// (essentially subdomains) to locations.
///
/// This particular mapper maps subdomains to locations in a blocked and
/// balanced manner. That is, given the subdomains (p0, p1, ..., pk) and l
/// locations, each location will be mapped contiguous blocks of k/l subdomains.
///
/// @tparam CID The base container ID type
///
/// @todo The Dom template parameter is never used in the codebase and adds
/// needless and measurable template overhead, given how much the mapper type
/// is used in the container framework.  Still might be useful, so we should
/// create a basic_mapper class with Dom (and CID for that matter, right now
/// it's always size_t), fixed.  And leave mapper more general for those who
/// want to specialize behavior in the future.  See similar idea in
/// @ref directory_impl::manager and @ref basic_manager.
///
/// @todo The code using m_locations has been commented due to a O(p) lookup
/// the current implementation uses in components()/location_position(). The
/// goal of the variable is to allow mapping to a sparse (i.e., not renamed)
/// group of locations.  The feature is not used in the source right now, and
/// the current code will cause unacceptable overhead for the current, general,
/// not sparse case. (a) A better implementation needs to be found and (b) more
/// typing / templating needs to be done to avoid any overhead when not in use.
//////////////////////////////////////////////////////////////////////
template<typename CID> //, typename Dom = use_default>
struct mapper
{
  typedef CID                                              cid_type;

  //  typedef typename select_parameter<
  //    Dom, indexed_domain<size_t>
  //  >::type                                                  domain_type;
  typedef indexed_domain<size_t>                            domain_type;


  /// @brief A view used to return the CIDs a location is responsible for.
  typedef array_ro_view<
    identity_container<cid_type>, domain_type
  >                                                        cid_view_type;


  typedef location_type                                    value_type;

private:
  /// @brief The partitioner that is used to do the mapping of CIDs.
  typedef balanced_partition<domain_type>                  partition_cids_type;

  partition_cids_type m_partition_cids;

#if 0
  // The domain of location
  domain_type              m_locations;

  ///////////////////////////////////////////////////////////////////
  /// @brief Find the position of a certain location within the set of
  ///   total locations that this mapper is using as its codomain.
  /// @param loc The location to look up
  /// @return How many locations precede this location
  ///////////////////////////////////////////////////////////////////
  size_t location_position(location_type loc) const
  {
    size_t pos            = 0;
    location_type tmp_loc = m_locations.first();

    while (tmp_loc != loc && pos < m_locations.size())
    {
      ++pos;
      tmp_loc = m_locations.advance(tmp_loc, 1);
    }

    return pos;
  }

  FROM CTOR
   ,domain_type const& locations = domain_type(
     get_num_locations(static_cast<size_t>())))
  m_partition_cids(dom, locations.size())
  , m_locations(locations)
#endif

public:
  mapper() = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Instantiate the mapper with the domain of base container IDs
  /// and optionally, the set of locations that we are mapping to.
  /// @param dom Domain of base container IDs (CIDs)
  ///
  /// Explicitly calling freestanding get_num_locations(), as member
  /// is not initialized yet in this constructor.
  //////////////////////////////////////////////////////////////////////
  mapper(domain_type const& dom)
    : m_partition_cids(dom, stapl::get_num_locations())
  { }

#if 0
  //////////////////////////////////////////////////////////////////////
  /// @brief Instantiate the mapper with the domain of base container IDs
  /// and optionally, the set of locations that we are mapping to.
  /// @param dom Domain of base container IDs (CIDs)
  /// @param locations Domain of locations. By default, this is the domain
  /// [0, 1, ..., p) (i.e., the set of all locations)
  //////////////////////////////////////////////////////////////////////
  mapper(domain_type const& dom,
         domain_type const& locations = domain_type(
           get_num_locations(static_cast<size_t>(is_nested()))))
    : m_partition_cids(dom, locations.size()), m_locations(locations)
  { }
#endif

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a view that contains all of the base container IDs
  /// that a particular location is responsible for. This is the inverse
  /// of @ref mapper::map.
  /// @param loc The location to query
  /// @return A view of base container IDs
  ///
  /// @todo The heap allocation here is needless overhead.  The only caller
  /// of this code is the base_container_manager ctor, which consumes the
  /// result and then discards it on scope exit.  Should use a reference to
  /// the partition_cids domain element or something already allocated and/or
  /// on the stack.
  //////////////////////////////////////////////////////////////////////
  cid_view_type components(size_t loc) const
  {
#if 0
    if (m_locations.contains(loc))
    {
      const size_t pos = location_position(loc);

      return cid_view_type(
        new identity_container<cid_type>(), m_partition_cids[pos]
      );
    }

    // else
    return cid_view_type(new identity_container<cid_type>(), domain_type());
#endif

    return cid_view_type(
      new identity_container<cid_type>(), m_partition_cids[loc]
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Map a base container ID to a location
  /// @param cid The base container ID
  /// @return The location that the base container ID is mapped to
  //////////////////////////////////////////////////////////////////////
  location_type map(cid_type const& cid) const
  {
    stapl_assert(
      m_partition_cids.global_domain().contains(cid),
      "mapping invalid cid to a location"
    );

    return m_partition_cids.find(cid);
  }


  size_t get_num_locations(void) const
  {
    return m_partition_cids.size();
  }


  cid_type next(cid_type const& cid) const
  {
    return m_partition_cids.global_domain().advance(cid,1);
  }

  bool valid(cid_type const& cid) const
  {
    return m_partition_cids.global_domain().contains(cid);
  }

  void define_type(typer& t)
  {
    t.member(m_partition_cids);
    // t.member(m_locations);
  }
}; // class mapper

} // namespace stapl

#endif // STAPL_CONTAINERS_MAPPER_HPP

