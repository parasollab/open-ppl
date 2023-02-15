/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MAP_METADATA_HPP
#define STAPL_CONTAINERS_MAP_METADATA_HPP

#include <stapl/domains/iterator_domain.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/views/metadata/container_fwd.hpp>
#include <stapl/containers/iterators/gid_of_fwd.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Class for computing the metadata of @ref map_distribution,
/// and all other associative distributions.
/// @tparam Distribution Type of the Distribution.
//////////////////////////////////////////////////////////////////////
template<typename Distribution>
class map_metadata
{
  STAPL_IMPORT_TYPE(typename Distribution, index_type)
  STAPL_IMPORT_TYPE(typename Distribution, base_container_type)
  STAPL_IMPORT_TYPE(typename Distribution, container_manager_type)

  typedef typename base_container_type::domain_type      bc_domain_type;

  typedef iterator_domain<
    Distribution, typename bc_domain_type::field_selector
  >                                                        domain_type;

public:
  // public access required to allow the container's distribution class access.
  typedef metadata_entry<domain_type, base_container_type*> dom_info_type;

private:
  typedef metadata::growable_container<dom_info_type>      metadata_type;

public:
  typedef metadata_type                                    metadata_cont_type;
  typedef std::pair<bool, metadata_cont_type*>             return_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the metadata of the specified container.
  /// @see metadata_entry.
  /// @param cont A pointer to the container.
  ///
  /// Calls the operator on the distribution of the provided container.
  ///
  /// @return a pair indicating if the metadata container is static and
  ///   balance distributed and a pointer to the metadata container.
  //////////////////////////////////////////////////////////////////////
  template <typename Container>
  return_type operator()(Container* cont)
  {
    return operator()(&(cont->distribution()));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the metadata of the specified distribution.
  /// @see metadata_entry.
  /// @param dist A pointer to the distribution.
  ///
  /// @return a pair indicating if the metadata container is static and
  ///   balance distributed and a pointer to the metadata container.
  //////////////////////////////////////////////////////////////////////
  return_type operator()(Distribution* dist)
  {
    metadata_cont_type* out_part = new metadata_type;

    typedef typename container_manager_type::const_iterator citer_t;

    citer_t cit     = dist->container_manager().begin();
    citer_t cit_end = dist->container_manager().end();

    for ( ; cit != cit_end; ++cit)
    {
      base_container_type& bc = *cit;

      if (!bc.domain().empty())
      {
        domain_type ndom(
          bc.domain().first(), bc.domain().last(), *dist, bc.domain().size()
        );

        out_part->push_back_here(dom_info_type(
           typename dom_info_type::cid_type(), ndom, &bc,
           LQ_CERTAIN, get_affinity(),
           dist->get_rmi_handle(), dist->get_location_id()
        ));
      }
    }

    out_part->update();

    return std::make_pair(false, out_part);
  }
}; // class map_metadata

} // namespace stapl

#endif // STAPL_CONTAINERS_MAP_METADATA_HPP
