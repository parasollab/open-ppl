/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_LIST_METADATA_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_LIST_METADATA_HPP

#include <stapl/containers/list/list_gid.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/views/metadata/container_fwd.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/distribution/container_manager/ordering/base_container_ranking.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Class for computing the metadata of @ref list_distribution and
/// for all the other containers that export metadata as domains of
/// iterators (assoc).
/// @tparam Distribution Type of the Distribution.
/// @todo Replace the use of this class with static_metdata.  Issues are
/// the type returned by iteration over container manager and a difference
/// in partition domain type and base container domain type.
//////////////////////////////////////////////////////////////////////
template<typename Distribution>
class list_metadata
{
public:
  typedef typename Distribution::index_type              index_type;

private:
  typedef typename Distribution::gid_type                gid_type;
  typedef typename Distribution::base_container_type     base_container_type;
  typedef typename Distribution::container_manager_type  container_manager_type;

public:
  typedef typename distribution_traits<Distribution>::domain_type   domain_type;
  // public access required to allow the container's distribution class access.
  typedef metadata_entry<domain_type, base_container_type*>
    dom_info_type;

private:
  typedef metadata::growable_container<dom_info_type>    metadata_type;

public:
  typedef std::pair<bool, metadata_type*>                return_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the metadata of specified container.
  /// @see metadata_entry.
  /// @param c A pointer to the container that uses this metadata.
  ///
  /// Calls the function operator on the distribution of the provided container.
  ///
  /// @return a pair indicating if the metadata container is static and
  ///   balance distributed and a pointer to the metadata container.
  //////////////////////////////////////////////////////////////////////
  template <typename Container>
  return_type operator()(Container* c)
  {
    return operator()(&(c->distribution()));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the metadata of specified distribution.
  /// @see metadata_entry.
  /// @param dist A pointer to the distribution that uses this metadata.
  ///
  /// @return a pair indicating if the metadata container is static and
  ///   balance distributed and a pointer to the metadata container.
  /// @todo Add support for cross-gang metadata extraction.
  //////////////////////////////////////////////////////////////////////
  return_type operator()(Distribution* dist)
  {
    const size_t num_comp =
      base_container_ranking(dist->container_manager().m_ordering);

    metadata_type* out_part = new metadata_type(num_comp);

    // same groups
    if (compare_gangs(out_part->get_rmi_handle(), dist->get_rmi_handle()) == 0)
    {
      typename container_manager_type::const_iterator cit
        = dist->container_manager().begin();
      typename container_manager_type::const_iterator cit_end
        = dist->container_manager().end();
      for (;cit != cit_end; ++cit) {
        base_container_type* bc = *cit;

        domain_type ndom(bc->domain().first(), bc->domain().last(), *dist);

        size_t rank = dist->container_manager().rank(bc);

        (*out_part)[rank]=(dom_info_type(
           rank, ndom, bc,
           LQ_CERTAIN, get_affinity(),
           dist->get_rmi_handle(), dist->get_location_id()
        ));
      }
    }
    else
    {
      abort("coarsening views cross-gang is not yet supported");
#if 0
      // This code will be updated and used as more complex nesting of
      // containers is supported.
      //
      // This code was modeled after the array_metadata.  The issue is that
      // Distribution::local_return_type uses domains from the partition, which
      // are not the same type as the domains from the base containers of the
      // list.


      // execution in one location, data in different(s) location(s)
      if (out_part->get_num_locations()==1)
      {
        typedef typename Distribution::local_return_type data_type;
        futures<data_type> h1 = opaque_rmi(
          all_locations, dist->get_rmi_handle(), &Distribution::metadata
        );

        for (size_t i=0;i<h1.size();++i)
        {
          data_type md = h1.get(i);

          for (size_t j=0;j<md.size();++j)
          {
            dom_info_type x = md[j];
            (*out_part)[x.id()] = x;
          }
        }
      }
      else
      {
        // Multiple locations trying to access data from one location
        size_t tid       = out_part->get_location_id();
        typename container_manager_type::const_iterator cit
          = dist->container_manager().begin();
        typename container_manager_type::const_iterator cit_end
          = dist->container_manager().end();
        for (;cit != cit_end; ++cit) {
          base_container_type* bc = *cit;
          balanced_partition<domain_type> pdom(bc.domain(),
                                               out_part->get_num_locations());
          (*out_part)[tid] = dom_info_type(pdom[tid], &bc, tid,
                                           get_affinity(), tid);
        }
      }
#endif
    }

    out_part->update();
    return std::make_pair(false, out_part);
  }
};

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_LIST_METADATA_HPP
