/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_DISTRIBUTION_TRAITS_HPP
#define STAPL_CONTAINERS_GRAPH_DISTRIBUTION_TRAITS_HPP

#include <stapl/domains/distributed_domain.hpp>
#include <stapl/domains/iterator_domain.hpp>
#include <stapl/domains/domain_interval.hpp>

namespace stapl {

//Forward declaration of hierarchical graph traits
template<graph_attributes D, graph_attributes M, typename VertexPx,
         typename EdgePx, typename PSx, typename Mapx>
struct hgraph_traits;


//////////////////////////////////////////////////////////////////////
/// @brief Default domains for the @ref graph_distribution.
/// Specifies domain and metadata domain types.
/// @ingroup pgraphTraits
/// @tparam Distribution The type of the distribution.
/// @tparam Traits The traits of the container.
///
/// An @ref iterator_domain is used as the domain in this specialization.
////////////////////////////////////////////////////////////////////////
template<typename Distribution, typename Traits>
struct domain_selector
{
  typedef domainset1D<Distribution>       domain_type;

  typedef domain_type                     metadata_domain_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Default domains for the @ref graph_distribution,
/// specialized for the @ref hierarchical_graph.
/// Specifies domain and metadata domain types.
/// @ingroup pgraphTraits
/// @tparam Distribution The type of the distribution.
/// @tparam Traits The traits of the container.
///
/// A @ref domainset1D is used as the domain in this specialization.
////////////////////////////////////////////////////////////////////////
template<typename Distribution,
         graph_attributes D, graph_attributes M, typename VertexPx,
         typename EdgePx, typename PSx, typename Mapx>
struct domain_selector<Distribution,
                       hgraph_traits<D, M, VertexPx, EdgePx, PSx, Mapx> >
{
  typedef domainset1D<Distribution>               domain_type;
  typedef domain_type                             metadata_domain_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the @ref graph_distribution.
/// Specifies distribution, domain and metadata types.
/// @ingroup pgraphTraits
/// @tparam Distribution The type of the distribution.
/// @tparam C The type of the container.
////////////////////////////////////////////////////////////////////////
template<typename Distribution,
         typename C>
struct graph_distribution_traits
{
  typedef Distribution                                 distribution_type;

  typedef domain_selector<distribution_type,
                          typename container_traits<C>::traits_t> dom_s_t;
  typedef typename dom_s_t::domain_type               domain_type;
  typedef typename dom_s_t::metadata_domain_type      metadata_domain_type;
};

}

#endif /*STAPL_CONTAINERS_GRAPH_DISTRIBUTION_TRAITS_HPP*/
