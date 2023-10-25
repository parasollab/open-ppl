/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_PARTITIONERS_PARTITIONER_UTILS_HPP
#define STAPL_CONTAINERS_GRAPH_PARTITIONERS_PARTITIONER_UTILS_HPP

#include <boost/mpl/has_xxx.hpp>
#include <stapl/domains/interval.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/algorithms/hierarchical_view.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/native_view.hpp>

namespace stapl {


//Select closed domain type for levels of Hierarchical view if possible
BOOST_MPL_HAS_XXX_TRAIT_DEF(closed_domain_type)


//////////////////////////////////////////////////////////////////////
/// @brief struct extracting the closed-form domain type of a graph view
///        if any, otherwise domset1D type is used.
/// @ingroup pgraphPartitioner
/// @tparam bool true if there is a domain type.
/// @tparam GraphVw graph view type.
///
/// General case: there is a closed-form domain type
//////////////////////////////////////////////////////////////////////
template<bool, typename GraphVw>
struct extracted_closed_domain_type
{
  typedef typename GraphVw::view_container_type::closed_domain_type type;
};


//////////////////////////////////////////////////////////////////////
/// @brief struct extracting the closed-form domain type of a graph view
///        if any, otherwise domset1D type is picked.
/// @ingroup pgraphPartitioner
/// @tparam bool true if there is a domain type.
/// @tparam GraphVw graph view type.
///
/// 'False' case: there is no closed-form domain type and domset1D is used.
//////////////////////////////////////////////////////////////////////
template<typename GraphVw>
struct extracted_closed_domain_type<false, GraphVw>
{
  typedef domset1D<typename GraphVw::vertex_descriptor> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief struct extracting the closed-form domain type of a graph view
///        if any, otherwise domset1D type is picked.
/// @ingroup pgraphPartitioner
/// @tparam bool true if there is a domain type.
/// @tparam GraphVw graph view type.
///
/// 'False' case: there is no closed-form domain type and domset1D is used.
//////////////////////////////////////////////////////////////////////
template<typename GraphVw>
struct extract_closed_domain_type
{
  typedef typename extracted_closed_domain_type<
                     has_closed_domain_type<
                       typename GraphVw::view_container_type>::value,
                       GraphVw>::type type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper to have the partition hierarchical view type.
/// @ingroup pgraphPartitioner
/// @tparam GraphVw graph view type of the base graph.
/// @tparam EdgeProperty edge property type of the hierarchical view.
//////////////////////////////////////////////////////////////////////
template<typename GraphVw, typename EdgeProperty=properties::no_property>
struct partition_view_type
{
  typedef hierarchical_graph_view<dynamic_graph<DIRECTED, MULTIEDGES,
                                  hierarchical_graph_view<
                                    typename GraphVw::view_container_type,
                                    typename extract_closed_domain_type<
                                      GraphVw>::type >, EdgeProperty> > type;
};


namespace detail {

template<typename Hview, typename Gview, typename VP>
struct add_vertex_wf
{
  Gview const*  m_gvw0;
  Hview*        m_hvw0;
  size_t        m_level;

  typedef void result_type;

  add_vertex_wf(Gview const& gvw0, Hview& hvw0, size_t level)
    : m_gvw0(&gvw0), m_hvw0(&hvw0), m_level(level)
  { }

  template<typename Descriptor>
  void operator()(Descriptor vd) const
  {
    typedef typename VP::domain_type hview_domain_type;

    m_hvw0->add_vertex(
      vd, VP(0, m_gvw0->container(), hview_domain_type(), m_gvw0->mapfunc())
    );
  }

  void define_type(typer& t)
  {
    abort("add_vertex_wf isn't expected to be serialized");

    t.member(m_gvw0);
    t.member(m_hvw0);
    t.member(m_level);
  }
}; // struct add_vertex_wf

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Create a partition view level over a graph view with
///        a specific number of partitions.
/// @ingroup pgraphPartitioner
/// @param gview base graph view.
/// @param num_partitions number of partitions.
/// @return partition view of size num_partitions.
//////////////////////////////////////////////////////////////////////
template <typename GView>
typename partition_view_type<GView>::type
create_partition_view(GView const& gview, size_t const& num_partitions)
{
  typedef typename partition_view_type<GView>::type::
                                           vertex_property hview_property_type;
  typedef typename partition_view_type<GView>::type::
                                      view_container_type hview_container_type;

  typedef hierarchical_graph_view<hview_container_type> hview_type;

  hview_container_type* ct_ptr = new hview_container_type();

  typedef detail::add_vertex_wf<
    hview_container_type, GView, hview_property_type
  > wf_t;

  map_func(wf_t(gview, *ct_ptr, 0), counting_view<size_t>(num_partitions));

  return hview_type(1, ct_ptr);
}


//////////////////////////////////////////////////////////////////////
/// @brief Create a partition view level over another partition view with
///        a specific number of partitions.
/// @ingroup pgraphPartitioner
/// @param hview partition view.
/// @param num_partitions number of partitions.
/// @return partition view of size num_partitions.
//////////////////////////////////////////////////////////////////////
template <typename PG, typename Dom, typename MapFunc, typename Derived>
typename partition_view_type<hierarchical_graph_view<PG, Dom, MapFunc, Derived>
                                                                        >::type
create_partition_view(hierarchical_graph_view<PG, Dom, MapFunc, Derived> const&
                                           hview, size_t const& num_partitions)
{
  typedef hierarchical_graph_view<PG, Dom, MapFunc, Derived> Hview;

  typedef typename partition_view_type<Hview>::type::vertex_property
                                                           hview_property_type;
  typedef typename partition_view_type<Hview>::type::view_container_type
                                                          hview_container_type;
  typedef hierarchical_graph_view<hview_container_type> hview_type;

  hview_container_type* ct_ptr = new hview_container_type();

  typedef detail::add_vertex_wf<
    hview_container_type, Hview, hview_property_type
  > wf_t;

  map_func(wf_t(hview, *ct_ptr, hview.level()),
           counting_view<size_t>(num_partitions));

  return hview_type(hview.level() + 1, ct_ptr);
}


//////////////////////////////////////////////////////////////////////
/// @brief Functor adding a vertex to a super-vertex.
/// @ingroup pgraphPartitioner
/// @param vid vertex descriptor to be added.
/// @param sv super-vertex reference.
//////////////////////////////////////////////////////////////////////
struct add_to_supervertex
{
  typedef void result_type;

private:
  size_t vertex_descriptor;

public:
  add_to_supervertex(size_t const& vid)
    : vertex_descriptor(vid)
  { }

  template<typename SuperVertex>
  void operator()(SuperVertex& sv) const
  {
    sv.domain()+=vertex_descriptor;
  }

  void define_type(typer& t)
  {
    t.member(vertex_descriptor);
  }

};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to move a vertex id from a partition to a
///        target partition.
/// @ingroup pgraphPartitioner
/// @tparam Hview partition view type.
/// @param vertex_descriptor vertex descriptor to be moved.
/// @param part_view partition view.
/// @param part_target_id target partition id.
/// @param sv source partition.
//////////////////////////////////////////////////////////////////////
template <typename Hview>
struct move_vertex_wf
{
  typedef void result_type;

private:
  size_t vertex_descriptor;
  mutable Hview partition_view;
  size_t partition_target_id;

public:
  move_vertex_wf(size_t const& vid, Hview& part_view,
                                                  size_t const& part_target_id)
   : vertex_descriptor(vid), partition_view(part_view), partition_target_id
                                                               (part_target_id)
  { }

  template<typename SuperVertex>
  void operator()(SuperVertex& sv) const
  {
    sv.domain()-=vertex_descriptor;
    //add vertex id to p2 partition
    partition_view.vp_apply_async(partition_target_id, add_to_supervertex
                                                          (vertex_descriptor));
  }

  void define_type(typer& t)
  {
    t.member(vertex_descriptor);
    t.member(partition_view);
    t.member(partition_target_id);
  }

};


//////////////////////////////////////////////////////////////////////
/// @brief Move a vertex id from a partition to a target partition.
/// @ingroup pgraphPartitioner
/// @tparam Hview partition view type.
/// @param partition_view partition view.
/// @param partition_source_id source partition.
/// @param partition_target_id target partition id.
/// @param vertex_id vertex descriptor to be moved.
//////////////////////////////////////////////////////////////////////
template <typename Hview>
void move_vertex(Hview& partition_view, size_t const& partition_source_id,
                 size_t const& partition_target_id, size_t const& vertex_id)
{
  //move from source partition to target partition
  partition_view.vp_apply_async(partition_source_id, move_vertex_wf<Hview>
                             (vertex_id, partition_view, partition_target_id));
}


//////////////////////////////////////////////////////////////////////
/// @brief Functor to specify the creation of super-vertices representing
///        the actual distribution of a new level of hierarchy
/// @ingroup pgraphPartitioner
/// @tparam GV base graph view type.
//////////////////////////////////////////////////////////////////////
template<class GV>
struct native_partitioner
{
  typedef domset1D<size_t>                        setdom_type;
  typedef array<setdom_type>               set_container_type;
  typedef array_view<set_container_type>          view_type;
  typedef typename result_of::counting_view<size_t>::type descriptor_view_type;

  struct copy_domain
  {
    typedef void result_type;

    template<typename Elt1, typename Elt2>
    void operator()(Elt1 elt1, Elt2 elt2)
    {
      typename Elt1::vertex_iterator it;
      setdom_type dom;
      for (it=elt1.begin(); it!=elt1.end(); ++it)
      {
        dom+=(*it).descriptor();
      }
      elt2=dom;
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a pair of a list of domains containing
  ///        super-vertices' children and a list of super-vertices' descriptors
  ///        for the creation of a new level of hierarchy representing
  ///        the current distribution
  /// @param g graph view
  /// @param lvl level of hierarchy
  /// @return pair of vertex domains and super-vertices' descriptors
  //////////////////////////////////////////////////////////////////////
  std::pair<view_type, descriptor_view_type> operator() (GV g, size_t lvl) const
  {

    typename stapl::result_of::native_view<GV>::type native_vw =
      stapl::native_view(g);
    size_t num_partitions=native_vw.size();

    view_type array_vw(new set_container_type(num_partitions));

    map_func(copy_domain(),native_vw,array_vw);

    return std::make_pair(array_vw, counting_view<size_t>(num_partitions));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to specify the super-edges for the hierarchy level
///        representing the current distribution
/// @ingroup pgraphPartitioner
//////////////////////////////////////////////////////////////////////
struct native_ef
{
  typedef properties::no_property value_type;
  template<class Graph>
  void operator()(Graph* g, size_t lvl) const
  { }

};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to compute the centroid of a cell.
/// @todo remove the copy of the edge list when the vertex proxy is
/// working correctly.
//////////////////////////////////////////////////////////////////////
struct build_centroids_wf
{
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param cell mesh cell.
  //////////////////////////////////////////////////////////////////////
  template <typename Cell>
  void operator()(Cell cell) const
  {
    typedef typename Cell::property_type::geom_vector_type geom_vector_type;
    typedef typename Cell::adj_edges_type::edge_type::
                               property_type::array_view_type  vertex_array;
    typedef typename vertex_array::domain_type                  domain_type;
    typedef typename Cell::adj_edges_type::edge_type::
                        property_type::vid_sequence_type  vid_sequence_type;
    typename Cell::adj_edge_iterator edge_it = cell.begin(),
                                     edge_it_end = cell.end();

    vid_sequence_type list_vids;
    geom_vector_type result;
    //iterate over the faces
    for (; edge_it != edge_it_end; ++edge_it)
    {
      vid_sequence_type vids = (*edge_it).property().get_vertex_ids();

      std::copy(vids.begin(), vids.end(), std::back_inserter(list_vids));

    }

    std::sort(list_vids.begin(), list_vids.end());
    typename vid_sequence_type::iterator v_end =
             std::unique(list_vids.begin(), list_vids.end());
    size_t num_vids = v_end - list_vids.begin();
    list_vids.resize(num_vids);

    vertex_array v_array = (*cell.begin()).property().get_vertices();
    //Calculate centroid
    vertex_array vert_array = vertex_array(v_array.container(),
                                             domain_type(list_vids));
    typename vertex_array::iterator it = vert_array.begin(),
                                end_it = vert_array.end();

    //FIXME: change with stapl::accumulate when view coarsening fixed
    for (; it != end_it; ++it)
      result += *it;

    result /= num_vids;
    cell.property().set_centroid(result);
  }
};



//////////////////////////////////////////////////////////////////////
/// @brief Compute the cell centroids of a mesh.
/// @param mesh mesh data structure.
//////////////////////////////////////////////////////////////////////
template <typename MeshType>
void build_centroids(MeshType & mesh)
{
  typedef graph_view<MeshType> view_type;
  view_type mesh_vw(mesh);
  map_func(build_centroids_wf(), mesh_vw);
}

} //namespace stapl

#endif //STAPL_CONTAINERS_GRAPH_PARTITIONERS_PARTITIONER_UTILS_HPP
