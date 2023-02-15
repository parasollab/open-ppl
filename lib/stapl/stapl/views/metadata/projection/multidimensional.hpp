/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_PROJECTION_MULTIDIMENSIONAL_HPP
#define STAPL_VIEWS_METADATA_PROJECTION_MULTIDIMENSIONAL_HPP

#include <stapl/runtime.hpp>
#include <stapl/views/metadata/container_fwd.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/domains/partitioned_domain.hpp>

#include <stapl/views/metadata/locality_dist_metadata.hpp>
#include <stapl/views/metadata/projection/construct_domain.hpp>

#include <stapl/containers/sequential/graph/graph.h>
#include <stapl/utility/domain_ops.hpp>

#include <stapl/containers/partitions/block_partition.hpp>
#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>

#include <stapl/utility/do_once.hpp>
#include <stapl/utility/pack_ops.hpp>
#include <stapl/utility/integer_sequence.hpp>

#include <stapl/views/metadata/projection/geometry.hpp>


namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Graph-based metadata projection algorithm to project
///        a view's information
///
/// @todo operator should return a shared_ptr.
/// @todo The sequential graph type should be a map, as we are deleting
///       random vertices in the algorithm and we would have better
///       asymptotic complexity.
//////////////////////////////////////////////////////////////////////
template <typename View, typename P>
class multidimensional_metadata_projection
{
  typedef typename View::domain_type                   domain_type;
  typedef typename View::map_func_type                 map_func_type;
  typedef typename P::value_type::component_type       component_type;
  typedef typename domain_type::gid_type               gid_type;

  typedef metadata_entry<domain_type, component_type,
            typename P::value_type::cid_type>          dom_info_type;

  typedef typename View::view_container_type           container_type;
  typedef typename container_type::distribution_type   distribution_type;
  typedef typename distribution_type::dom_info_type    loc_info;
  typedef typename dimension_traits<View>::type        dimension_type;

  typedef typename default_traversal<
    dimension_type::value>::type                        traversal_t;
  typedef multiarray_impl::block_partition<traversal_t> decomposed_partition_t;
  typedef typename decomposed_partition_t::value_type   decomposed_domain_t;

  typedef sequential::graph<
            UNDIRECTED, NONMULTIEDGES, loc_info, properties::no_property
          > graph_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return whether a domain occurs before another domain in
  ///        the k'th dimension.
  //////////////////////////////////////////////////////////////////////
  template<int k, typename D>
  bool domain_greater(D const& a, D const& b) const
  {
    return get<k>(a.first()) > get<k>(b.last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive two vertex descriptors for hyperrectangles in the graph
  ///        and order them such that the first parameter is "above"
  ///        the second parameter.
  //////////////////////////////////////////////////////////////////////
  template<typename G, std::size_t... Dims>
  void swap_hyperrectangle_if_greater(G& g, std::size_t& a, std::size_t& b,
                                      index_sequence<Dims...>) const
  {
    auto const& a_dom = g.find_vertex(a)->property().domain();
    auto const& b_dom = g.find_vertex(b)->property().domain();

    // if b is above a
    if (pack_ops::functional::or_(domain_greater<Dims>(a_dom, b_dom)...))
      std::swap(a, b);

  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Merge the two hyperrectangles given by the vertex descriptors
  ///        a and b by combining their volume and replacing the edges
  ///        of one into the other and deleting the other.
  //////////////////////////////////////////////////////////////////////
  template<typename G>
  void merge_hyperrectangle(G& g, std::size_t a, std::size_t b) const
  {
    // sort the hyperrectangles s.t. a is "above" b
    swap_hyperrectangle_if_greater(g, a, b,
      make_index_sequence<dimension_type::value>()
    );

    auto const& a_dom = g.find_vertex(a)->property().domain();
    auto const& b_dom = g.find_vertex(b)->property().domain();

    // create a new hyper-rectangle that covers both domains
    domain_type new_dom(a_dom.first(), b_dom.last());

    // update md entry
    g.find_vertex(a)->property().set_domain(new_dom);

    // add b's edges to a
    for (auto&& e : *g.find_vertex(b))
      g.add_edge(a, e.target());

    // remove b
    g.delete_vertex(b);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Perform the graph hyperrectangle merging algorithm.
  ///
  ///        Iteratively attempt to merge neighboring hyperrectangles
  ///        until no more adjacent hyperrectangles can be merged.
  ///
  /// @param g A graph whose properties are hyperrectangles
  //////////////////////////////////////////////////////////////////////
  template<typename G>
  void coarsen_grid(G& g) const
  {
    // remove all edges that are between two chunks with
    // different metadata information
    std::vector<typename G::edge_descriptor> to_delete;

    for (auto&& v : g)
    {
      for (auto&& e : v)
      {
        auto const& s_md = v.property();
        auto const& t_md = g.find_vertex(e.target())->property();

        const bool different_md =
          (s_md.location() != t_md.location()) ||
          (s_md.affinity() != t_md.affinity()) ||
          (s_md.id() != t_md.id());

        if (different_md)
          to_delete.push_back(e.descriptor());
      }
    }

   for (auto&& e : to_delete)
     g.delete_edge(e);

   bool done = false;

   // iterate until all hyper-rectangles have been merged
   while (!done)
   {
     done = true;

     std::unordered_set<std::size_t> seen;
     std::vector<std::pair<std::size_t, std::size_t> > ms;

     for (auto&& v : g)
     {
       for (auto&& e : v)
       {
         bool have_seen = seen.count(e.source()) || seen.count(e.target());
         bool is_mergeable = geometry_impl::is_mergeable<domain_type>::apply(
           g.find_vertex(e.source())->property().domain(),
           g.find_vertex(e.target())->property().domain()
         );

         // mark this edge to be merged if we haven't seen
         // it before and it is indeed mergeable
         if (is_mergeable && !have_seen)
         {
           seen.insert(e.source());
           seen.insert(e.target());

           ms.push_back(std::make_pair(e.source(), e.target()));

           done = false;
         }
       }
     }

     // merge hyper-rectangles that we found in this pass
     for (auto&& e : ms)
       merge_hyperrectangle(g, e.first, e.second);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Perform a spatial decomposition of the domain of the
  ///        views that represents a partitioning of work for projection.
  ///
  /// @param view The view that we are projecting
  //////////////////////////////////////////////////////////////////////
  static decomposed_partition_t
  decomposed_partition(View* vw)
  {
    return decomposed_partition_t(
      vw->domain(),
      multiarray_impl::make_multiarray_size<dimension_type::value>()(
        get_num_locations()
      )
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Domain of the original space that represents the part of
  ///        the space that the location @p rank is responsible for
  ///        projecting the view in.
  ///
  /// @param part Spatial decomposition of original space
  /// @param rank The location
  //////////////////////////////////////////////////////////////////////
  static decomposed_domain_t
  decomposed_domain(decomposed_partition_t const& part, std::size_t const rank)
  {
    typedef nd_reverse_linearize<gid_type, traversal_t> project_func_t;

    auto multidimensional_rank = project_func_t(part.dimensions())(rank);

    return part[multidimensional_rank];
  }

public:
  typedef metadata::growable_container<dom_info_type> md_cont_type;
  typedef std::pair<bool, md_cont_type*>              return_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Project the metadata information from the view's container
  ///        into the view's space and store it in part.
  //////////////////////////////////////////////////////////////////////
  return_type operator()(View* vw, P* part,
                         bool part_is_fixed = true) const
  {

    // the resulting md information
    md_cont_type* out_part = new md_cont_type();

    auto mf(vw->mapfunc());

    distribution_type& distribution = vw->container().distribution();

    // create a multidimensional partition to balance the work of projection
    auto decomposed = decomposed_partition(vw);

    // figure out which portion of the global space this location is
    // responsible for projecting
    auto subdomain = decomposed_domain(decomposed, get_location_id());
    nd_linearize<gid_type, traversal_t> linearizer(subdomain.dimensions(),
                                                   subdomain.first());

    // create a graph representing the subdomain that this location
    // is projecting locality for
    typedef geometry_impl::grid_generator<dimension_type::value> generator;
    generator gen(subdomain.dimensions());
    auto grid = gen.template build<graph_type>();

    std::size_t num_futures_recvd = 0;

    // fill the graph with metadata information
    domain_map(subdomain, [&](gid_type const& e) {
      auto gid = mf(e);

      // compute the vertex id in the grid for this gid
      const std::size_t vd = linearizer(e);

      // query the distribution for locality information
      distribution.metadata_at(gid).async_then(
        [&num_futures_recvd, &grid, e, vd](future<loc_info> f) {

        auto md = f.get();

        grid.find_vertex(vd)->property() = loc_info(
          md.id(), domain_type(e, e), md.component(), md.location_qualifier(),
          md.affinity(), md.handle(), md.location()
        );

        num_futures_recvd++;
      });
    });

    // wait until we've populated the graph with the right
    // metadata information
    block_until([&]() {
      return num_futures_recvd == grid.get_num_vertices();
    });

    // merge hyperrectangles in the graph
    coarsen_grid(grid);

    // add all of the metadata from the graph to the
    // output coarsened container
    for (auto&& v : grid)
    {
      auto const& md = v.property();

      out_part->push_back_here(dom_info_type(
        typename dom_info_type::cid_type(),
        md.domain(),
        static_cast<component_type>(md.component()),
        md.location_qualifier(), md.affinity(),
        md.handle(), md.location()
      ));
    }

    out_part->update();

    delete part;

    return std::make_pair(false, out_part);
  }
};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_PROJECTION_MULTIDIMENSIONAL_HPP
