/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_GRAPH_METADATA_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_GRAPH_METADATA_HPP

#include <stapl/views/metadata/container_fwd.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/skeletons/utility/tags.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Class for computing the metadata of @ref graph_distribution .
/// @tparam Distribution Type of the Distribution.
/// @ingroup pgraphDist
//////////////////////////////////////////////////////////////////////
template<typename Distribution>
class graph_metadata
{
public:
  typedef typename Distribution::index_type              index_type;
  typedef typename Distribution::location_type           location_type;
  typedef typename Distribution::container_type          container_type;
  typedef typename Distribution::base_container_type     base_container_type;
  typedef typename Distribution::container_manager_type  container_manager_type;
  typedef typename base_container_type::domain_type      bc_domain_type;

  typedef typename graph_distribution_traits<
                     Distribution,
                     container_type>::
                   metadata_domain_type                  domain_type;

  typedef metadata_entry<
    domain_type, base_container_type*
  >                                                      dom_info_type;
  typedef metadata::growable_container<dom_info_type>    metadata_type;

  typedef std::pair<bool, metadata_type*>                return_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the metadata of specified container.
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
  /// @brief Return the metadata of specified distribution.
  /// @see metadata_entry.
  /// @param dist A pointer to the distribution.
  /// @return a pair indicating if the metadata container is static and
  ///   balance distributed and a pointer to the metadata container.
  /// @todo Add support for cross-gang metadata extraction.
  /// @todo Eliminate default construction of out_part and use another
  /// p_object to determine how to extract metadata.
  //////////////////////////////////////////////////////////////////////
  return_type operator()(Distribution* dist)
  {
    const size_t num_comp       = dist->num_base_containers();

    metadata_type* out_part = new metadata_type(num_comp);

    typedef typename container_manager_type::iterator citer_t;


    // same groups
    if (compare_gangs(out_part->get_rmi_handle(), dist->get_rmi_handle()) == 0)
    {
      static_array<size_t>              num_empty_bcontainers(num_comp, 0);
      array_view<static_array<size_t> > num_empty_view(num_empty_bcontainers);

      citer_t cit     = dist->container_manager().begin();
      citer_t cit_end = dist->container_manager().end();

      for (;cit != cit_end; ++cit)
      {
        base_container_type& bc = *cit;
        size_t rank = dist->container_manager().rank(&bc);
        if (bc.domain().empty())
          num_empty_bcontainers[rank] = 1;
      }
      typedef metadata::detail::plus<size_t,
                typename dom_info_type::delay_type> wf_t;

      using namespace skeletons;

      size_t num_empty =
        stapl::reduce<tags::no_coarsening>(num_empty_view, wf_t());

      stapl::scan<tags::no_coarsening>(
        num_empty_view, num_empty_view, wf_t(), true);

      if (num_empty != 0)
      {
        delete out_part;
        out_part = new metadata_type(num_comp-num_empty);
      }

      cit = dist->container_manager().begin();
      for (;cit != cit_end; ++cit)
      {
        base_container_type& bc = *cit;
        domain_type ndom(bc.domain(),dist);
        size_t rank = dist->container_manager().rank(&bc);
        size_t prior_empty_bconts = num_empty == 0 ? 0 : num_empty_view[rank];
        if (!bc.domain().empty())
        {
          (*out_part)[rank-prior_empty_bconts] =
            dom_info_type(
              rank, ndom, &bc,
              LQ_CERTAIN, get_affinity(),
              dist->get_rmi_handle(), dist->get_location_id()
            );
        }
      }
      if (num_empty != 0)
        stapl::rmi_fence();
    }
    else
    {
      // execution in one location, data in different(s) location(s)
      if (out_part->get_num_locations() == 1)
      {
        typedef typename Distribution::local_return_type data_type;
        typedef futures<data_type>                       futures_t;

        futures_t h1 = opaque_rmi(all_locations,
                                  dist->get_rmi_handle(),
                                  &Distribution::metadata);

        typename futures_t::size_type              handle_size = h1.size();
        typename futures_t::aggregate_result_type  md_entries = h1.get();

        size_t num_empty = 0;
        // map is used to avoid O(num bcontainer) storage.

        std::map<size_t, size_t> prior_empty_bcontainers;

        for (size_t i=0; i != handle_size; ++i)
        {
          data_type& md = md_entries[i];

          for (size_t j=0;j<md.size();++j)
          {
            if (md[j].domain().empty())
            {
              ++num_empty;
              prior_empty_bcontainers[md[j].id()] = 1;
            }
          }
        }

        if (num_empty != 0)
        {
          delete out_part;

          typedef std::map<size_t, size_t>::iterator map_iter_t;

          out_part = new metadata_type(num_comp-num_empty);

          map_iter_t curr = prior_empty_bcontainers.begin();
          map_iter_t end  = prior_empty_bcontainers.end();
          map_iter_t prev = curr;

          ++curr;

          for (; curr != end; ++curr, ++prev)
          {
            curr->second += prev->second;
          }

          curr         = prior_empty_bcontainers.begin();
          curr->second = 0;
        }

        for (size_t i=0; i != handle_size; ++i)
        {
          data_type md = md_entries[i];

          for (size_t j=0; j != md.size(); ++j)
          {
            dom_info_type x = md[j];
            if (!x.domain().empty())
            {
              // upper bound is used because we're guaranteed key is greater
              // than the id.
              std::map<size_t, size_t>::iterator prev_empties =
                prior_empty_bcontainers.upper_bound(x.id());
              // adjust iterator to value lower than id.
              if (num_empty != 0)
              {
                --prev_empties;
                (*out_part)[x.id()-prev_empties->second] = x;
              } else {
                (*out_part)[x.id()] = x;
              }
            }
          }
        }
      }

      else
      {
        // Multiple locations trying to access data from one location
        abort("coarsening views cross-gang is not yet supported");
#if 0
        size_t tid       = out_part->get_location_id();
        citer_t cit     = dist->container_manager().begin();
        citer_t cit_end = dist->container_manager().end();

        for ( ; cit != cit_end; ++cit) {
          base_container_type& bc = *cit;
          balanced_partition<bc_domain_type> pdom(bc.domain(),
                                               out_part->get_num_locations());
          // How do I construct an interator_domain from this information?
          bc_domain_type bdom = pdom[tid];
          (*out_part)[tid] =
            dom_info_type(domain_type(bdom.first(), bdom.last()), &bc, tid,
                          get_affinity(), tid);
        }
#endif
      }
    }

    out_part->update();

    return std::make_pair(false, out_part);
  }
}; // class graph_metadata

} // namespace stapl

#endif
