/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/array/static_array.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/unordered_map/unordered_map.hpp>
#include <stapl/views/map_view.hpp>
#include <stapl/views/repeated_view.hpp>

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_GRAPH_METRICS_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_GRAPH_METRICS_HPP

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Stores the data for various local metrics for each partition.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
struct partition_metric_data
{
  typedef std::map<size_t, size_t> unique_nbr_map;

  /// The number of boundary edges (edges that cross this partition).
  size_t m_num_boundary_edges;
  /// The number of internal edges.
  size_t m_num_internal_edges;
  /// The number of unique external neighbors.
  size_t m_num_unique_ext_nbr;
  /// The number of connected components.
  size_t m_num_cc;
  /// The number of vertices in this partition.
  size_t m_num_vertices;
  /// @todo This seems to be unused. Remove this.
  size_t m_nbr;
  /// The unique neighbors.
  unique_nbr_map m_unm;

  partition_metric_data()
    : m_num_boundary_edges(0), m_num_internal_edges(0), m_num_unique_ext_nbr(0),
      m_num_cc(0), m_num_vertices(0), m_nbr(), m_unm()
  { }

  size_t num_boundary_edges() const
  { return m_num_boundary_edges; }

  size_t num_internal_edges() const
  { return m_num_internal_edges; }

  size_t num_unique_ext_nbr() const
  { return m_num_unique_ext_nbr; }

  size_t num_cc() const
  { return m_num_cc; }

  size_t num_vertices() const
  { return m_num_vertices; }

  size_t unm_size() const
  { return m_unm.size(); }

  void set_num_boundary_edges(size_t be)
  { m_num_boundary_edges = be; }

  void set_num_internal_edges(size_t ie)
  { m_num_internal_edges = ie; }

  void set_num_unique_ext_nbr(size_t nuen)
  { m_num_unique_ext_nbr = nuen; }

  void set_num_cc(size_t ncc)
  { m_num_cc = ncc; }

  void set_num_vertices(size_t nv)
  { m_num_vertices = nv; }

  void unm_insert(unique_nbr_map::value_type t)
  { m_unm.insert(t); }

  void incr_num_boundary_edges()
  { ++m_num_boundary_edges; }

  void incr_num_internal_edges()
  { ++m_num_internal_edges; }

  void incr_num_unique_ext_nbr()
  { ++m_num_unique_ext_nbr; }

  void incr_num_cc()
  { ++m_num_cc; }

  void incr_num_vertices()
  { ++m_num_vertices; }

  void define_type(typer& t)
  {
    t.member(m_num_boundary_edges);
    t.member(m_num_unique_ext_nbr);
    t.member(m_num_internal_edges);
    t.member(m_num_cc);
    t.member(m_num_vertices);
    t.member(m_unm);
  }
};



//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref partition_metric_data.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Accessor>
class proxy<partition_metric_data, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef partition_metric_data target_t;
  typedef target_t::unique_nbr_map unique_nbr_map;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  proxy const& operator=(target_t const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  size_t num_boundary_edges() const
  { return Accessor::const_invoke(&target_t::num_boundary_edges); }

  size_t num_internal_edges() const
  { return Accessor::const_invoke(&target_t::num_internal_edges); }

  size_t num_unique_ext_nbr() const
  { return Accessor::const_invoke(&target_t::num_unique_ext_nbr); }

  size_t num_cc() const
  { return Accessor::const_invoke(&target_t::num_cc); }

  size_t num_vertices() const
  { return Accessor::const_invoke(&target_t::num_vertices); }

  size_t unm_size() const
  { return Accessor::const_invoke(&target_t::unm_size); }

  void set_num_boundary_edges(size_t be)
  { Accessor::invoke(&target_t::num_boundary_edges, be); }

  void set_num_internal_edges(size_t ie)
  { Accessor::invoke(&target_t::num_internal_edges, ie); }

  void set_num_unique_ext_nbr(size_t nuen)
  { Accessor::invoke(&target_t::num_unique_ext_nbr, nuen); }

  void set_num_cc(size_t ncc)
  { Accessor::invoke(&target_t::num_cc, ncc); }

  void set_num_vertices(size_t nv)
  { Accessor::invoke(&target_t::num_vertices, nv); }

  void unm_insert(unique_nbr_map::value_type t)
  { Accessor::invoke(&target_t::unm_insert, t); }

  void incr_num_boundary_edges()
  { Accessor::invoke(&target_t::incr_num_boundary_edges); }

  void incr_num_internal_edges()
  { Accessor::invoke(&target_t::incr_num_internal_edges); }

  void incr_num_unique_ext_nbr()
  { Accessor::invoke(&target_t::incr_num_unique_ext_nbr); }

  void incr_num_cc()
  { Accessor::invoke(&target_t::incr_num_cc); }

  void incr_num_vertices()
  { Accessor::invoke(&target_t::incr_num_vertices); }
}; // struct proxy


namespace graph_metrics_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to update the metrics for each partition based on
/// new incoming data and the update flag.
/// @ingroup pgraphAlgoDetails
///
/// Update flag indicates which metrics to update, while the existing
/// metric for the partition is updated based on the incoming metric
/// from neighboring partitions.
/// @todo m_new_pair is probably useless. With the new semantics borrowed from
///   from the map container, the operator() now has access to the new_pair.
//////////////////////////////////////////////////////////////////////
struct update
{
  typedef void result_type;

  enum
  { UPDATE_NBE, UPDATE_NIE, UPDATE_NUEN, UPDATE_NUEN_NBE, UPDATE_NCC,
    UPDATE_NV, UPDATE_ALL };

  size_t m_update_flag;
  std::pair<size_t, partition_metric_data> m_new_pair;

  update()
    : m_update_flag(UPDATE_ALL)
  { }

  update(size_t update_flag,
         std::pair<size_t, partition_metric_data> const& part_data)
    : m_update_flag(update_flag), m_new_pair(part_data)
  { }

  template<class T1, class T2>
  void operator()(T1& old_pair,T2& new_pair) const
  {
    switch (m_update_flag)
    {
    case UPDATE_NBE:
      old_pair.second.incr_num_boundary_edges();
      break;
    case UPDATE_NIE:
      old_pair.second.incr_num_internal_edges();
      break;
    case UPDATE_NUEN:
      old_pair.second.unm_insert(*(m_new_pair.second.m_unm.begin()));
      old_pair.second.set_num_unique_ext_nbr(old_pair.second.unm_size());
      break;
    case UPDATE_NUEN_NBE:
      old_pair.second.unm_insert(*(m_new_pair.second.m_unm.begin()));
      old_pair.second.set_num_unique_ext_nbr(old_pair.second.unm_size());
      old_pair.second.incr_num_boundary_edges();
      break;
    case UPDATE_NCC:
      old_pair.second.incr_num_cc();
      break;
    case UPDATE_NV:
      old_pair.second.incr_num_vertices();
      break;
    default:
      break;
    }
  }

  void define_type(typer& t)
  {
    t.member(m_update_flag);
    t.member(m_new_pair);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to calculate boundary and internal edge data
/// for each partition.
/// @ingroup pgraphAlgoDetails
///
/// Used by @ref vertex_task.
//////////////////////////////////////////////////////////////////////
template<typename MapView>
struct target_wf
{
  typedef void result_type;
  typedef std::pair<const size_t, partition_metric_data> hash_map_pair;

  size_t m_src;
  size_t m_src_part;
  MapView* m_mapv;

  target_wf(size_t src, size_t src_part, MapView* mapv)
    : m_src(src), m_src_part(src_part), m_mapv(mapv)
  { }

  template<typename T>
  void operator()(T const& v) const
  {
    size_t curr_part = v;
    hash_map_pair temp_pair1(m_src_part, partition_metric_data());
    if (m_src_part != curr_part) {
      temp_pair1.second.unm_insert(std::pair<size_t, size_t>(curr_part, 0));
      temp_pair1.second.set_num_boundary_edges(1);
      m_mapv->insert(temp_pair1, update(update::UPDATE_NUEN_NBE,
                                              temp_pair1));
    } else {
      temp_pair1.second.set_num_internal_edges(1);
      m_mapv->insert(temp_pair1, update(update::UPDATE_NIE, temp_pair1));
    }
  }

  void define_type(typer& t)
  {
    t.member(m_src);
    t.member(m_src_part);
    t.member(m_mapv);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to calculate boundary and internal edge data
/// for each partition.
/// @ingroup pgraphAlgoDetails
///
/// Uses @ref target_wf to communicate with neighbors.
//////////////////////////////////////////////////////////////////////
template<typename GraphView, typename MapView>
struct vertex_task
{
  GraphView* m_gview;
  MapView*   m_mview;

  typedef void result_type;

  vertex_task(GraphView& gview, MapView& mview)
    : m_gview(&gview), m_mview(&mview)
  { }

  template<typename Vertex, typename VertexPartitionMap>
  void operator()(Vertex v, VertexPartitionMap& part_map) const
  {
    typename Vertex::adj_edge_iterator ei = v.begin();
    typename Vertex::adj_edge_iterator ei_end = v.end();
    for (; ei != ei_end; ++ei)
      m_gview->container().vp_apply_async((*ei).target(),
        target_wf<MapView>(v.descriptor(), part_map.get(v), m_mview));
  }

  void define_type(typer& t)
  {
    t.member(m_gview);
    t.member(m_mview);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to initialize the map holding the partition
/// metric data for each partition.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct init_num_vertices_task
{
  typedef void result_type;

  template<typename Vertex, typename MapView, typename VertexPartitionMap>
  void operator()(Vertex v, MapView& view,
                  VertexPartitionMap& part_map) const
  {
    std::pair<const size_t, partition_metric_data> temp_pair1(
      part_map.get(v), partition_metric_data());
    temp_pair1.second.set_num_vertices(1);
    view.insert(temp_pair1, update(update::UPDATE_NV, temp_pair1));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to compute the SV-ratio and smoothness metric
/// for each partition.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct compute_wf
{
  template<typename Ref1, typename Ref2, typename Ref3>
  void operator()(Ref1 p, Ref2 elem1, Ref3 elem2) const
  {
    elem1
      = static_cast<float>(static_cast<float>(p.second.num_boundary_edges()) /
                           static_cast<float>(p.second.num_internal_edges() +
                                              p.second.num_boundary_edges()));
    elem2
      = static_cast<double>(static_cast<double>(p.second.num_unique_ext_nbr()) /
                            static_cast<double>(p.second.num_internal_edges()));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to return the number of vertices for each
/// partition.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct get_num_vertices_map
{
  template<typename Ref>
  size_t operator()(Ref elem) const
  {
    return elem.second.num_vertices();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to return the number of vertices plus number
/// of ghost vertices for each partition.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct get_num_ghost_vertices_map
{
  template<typename Ref>
  size_t operator()(Ref elem) const
  {
    return elem.second.num_vertices() + elem.second.num_boundary_edges();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to return the number of internal edges for each
/// partition.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct get_num_internal_edges_map
{
  template<typename Ref>
  size_t operator()(Ref elem) const
  {
    return elem.second.num_internal_edges();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to return the number of cross edges for each
/// partition.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct get_num_cross_edges_map
{
  template<typename Ref>
  size_t operator()(Ref elem) const
  {
    return elem.second.num_boundary_edges();
  }
};

} // namespace graph_metrics_detail


//////////////////////////////////////////////////////////////////////
/// @brief Stores data about the quality of the partition of the graph.
/// @ingroup pgraphAlgo
///
/// This is the global metric for all graphs.
/// These metrics are useful for scientific-computing codes that work
/// on partitioned meshes. The metrics produce information about the
/// quality of the partition. These metrics are based on those requested
/// by a National Lab.
//////////////////////////////////////////////////////////////////////
struct metrics_info
{
  typedef static_array<float> array_t;
  typedef unordered_map<size_t, partition_metric_data> phm_t;
  typedef array_view<array_t> array_view_t;
  typedef map_view<phm_t> map_view_t;

  /// Array containing surface to volume ratios for all partitions.
  array_view_t sv_ratio;
  /// Array containing smoothness ratios for all partitions.
  array_view_t smoothness;
  /// Sum of internal edges.
  size_t sum_internal_edges;
  /// Sum of cross edges.
  size_t sum_cross_edges;
  /// Max num of vertices in any partition.
  size_t max_part_vertices;
  /// Load imbalance among partitions.
  float load_imbalance;
  /// Load imbalance (including ghost vertices) among partitions.
  float load_imbalance_ghosting;
  /// Amount of local communication per vertex.
  float comm_vs_local;

  /// @brief Map containing further details of partitions.
  /// Data type is @ref partition_info.
  /// Key type indicates the partition ID.
  map_view_t part_details_map;

  metrics_info(array_view_t& sv_ratio_, array_view_t& smoothness_,
               size_t sum_internal_edges_, size_t sum_cross_edges_,
               float load_imbalance_, float load_imbalance_ghosting_,
               float comm_vs_local_,
               size_t max_part_vertices_,
               map_view_t& part_details_map_)
  : sv_ratio(sv_ratio_), smoothness(smoothness_),
    sum_internal_edges(sum_internal_edges_), sum_cross_edges(sum_cross_edges_),
    max_part_vertices(max_part_vertices_),
    load_imbalance(load_imbalance_),
    load_imbalance_ghosting(load_imbalance_ghosting_),
    comm_vs_local(comm_vs_local_),
    part_details_map(part_details_map_)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to compute various metrics for the provided input
/// graph, based on the graph-partition provided.
/// @ingroup pgraphAlgo
///
/// These metrics are useful for scientific-computing codes that work
/// on partitioned meshes. The metrics produce information about the
/// quality of the partition. These metrics are based on those requested
/// by a National Lab.
///
/// @param g The @ref graph_view over the input graph.
/// @param vertex_partition_map The input vertex property map specifying
/// the partition that each vertex belongs to.
/// @return A @ref metrics_info object containing information about the
/// quality of the specified partition for the input graph.
//////////////////////////////////////////////////////////////////////
template<typename Graph, typename VertexPartitionMap>
metrics_info graph_metrics(Graph& g, VertexPartitionMap& vertex_partition_map)
{
  using namespace graph_metrics_detail;

  typedef graph_view<Graph> graph_view_t;
  typedef unordered_map<size_t, partition_metric_data> phm_t;
  typedef map_view<phm_t> phmv_t;
  typedef static_array<float> array_t;
  typedef array_view<array_t> array_view_t;

  graph_view_t graph_vw(g);
  phm_t* xmap = new phm_t();
  phmv_t builder_map_vw(*xmap);

  map_func(init_num_vertices_task(), graph_vw, make_repeat_view(builder_map_vw),
           make_repeat_view(vertex_partition_map));

  map_func(vertex_task<graph_view_t, phmv_t>(graph_vw, builder_map_vw),graph_vw,
           make_repeat_view(vertex_partition_map));

  size_t sz = xmap->size();

  array_t* sv_ratio = new array_t(sz);
  array_t* smoothness = new array_t(sz);
  array_view_t sv_ratio_view(sv_ratio);
  array_view_t smoothness_view(smoothness);

  // This view takes ownership of the container. It will be later returned to
  // the user as part of the result.
  phmv_t map_vw(xmap);

  map_func(compute_wf(), map_vw, sv_ratio_view, smoothness_view);

  size_t max_part_vertices  = map_reduce(get_num_vertices_map(),
                                         stapl::max<size_t>(), map_vw);
  size_t sum_internal_edges = map_reduce(get_num_internal_edges_map(),
                                         stapl::plus<size_t>(), map_vw);
  size_t sum_cross_edges    = map_reduce(get_num_cross_edges_map(),
                                         stapl::plus<size_t>(), map_vw);

  size_t max_part_ghost_vertices = map_reduce(get_num_ghost_vertices_map(),
                                              stapl::max<size_t>(), map_vw);

  float avg_vertices   = static_cast<float>(g.num_vertices())
                       / static_cast<float>(sz);
  float avg_int_edges  = static_cast<float>(sum_internal_edges)
                       / static_cast<float>(sz);
  float load_imbalance = static_cast<float>(max_part_vertices)
                       / static_cast<float>(avg_vertices);

  float avg_ghost_vertices = static_cast<float>(g.num_vertices()+
                                                sum_cross_edges)
                           / static_cast<float>(sz);

  float load_imbalance_ghosting = static_cast<float>(max_part_ghost_vertices)
                                / static_cast<float>(avg_ghost_vertices);

  float comm_vs_local  = static_cast<float>(avg_int_edges)
                       / static_cast<float>(avg_vertices);

  // Result copies the views and takes ownership of any containers inside them,
  // where needed.
  metrics_info result(sv_ratio_view, smoothness_view, sum_internal_edges,
                      sum_cross_edges, max_part_vertices, load_imbalance,
                      load_imbalance_ghosting, comm_vs_local, map_vw);

  return result;
}

}
#endif
