/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_CONNECTED_COMPONENTS_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_CONNECTED_COMPONENTS_HPP

#include <stapl/containers/graph/algorithms/execute.hpp>
#include <stapl/algorithms/algorithm.hpp>

namespace stapl {

namespace cc_algo_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Work-function to initialize each vertex's CCID to its own descriptor.
///
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct init_cc_wf
{
  typedef void result_type;

  template<class Vertex>
  void operator()(Vertex v) const
  {
    v.property().cc(v.descriptor());
    v.property().active(true);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Reducer functor for @ref connected_components().
///
/// Reduces two CC properties to update the first one.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct vp_reducer
{
  template<typename VP1, typename VP2>
  void operator()(VP1& p1, VP2& p2) const
  {
    if (p1.cc() > p2.cc()) {
      p1.cc(p2.cc());
      p1.active(true);
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to extract the different CCs and count the vertices
/// belonging to them.
///
/// @tparam UMap The type of the map used for storing the output: [CCID->count].
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename UMap>
class extract_graph_cc_stats_wf
{
public:
  typedef UMap result_type;

  template<typename Graph>
  result_type operator()(Graph g) const
  {
    typedef typename Graph::vertex_iterator VI;
    UMap count_map;
    VI vi = g.begin();
    VI vi_e = g.end();
    for (; vi != vi_e; ++vi)
      count_map[(*vi).property().cc()]++;
    return count_map;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to combine two input maps by adding their
/// valid elements.
/// @tparam UMap The type of the map.
/// @todo Eliminate the copy of m2p once the proxy supports it, as the
/// data is local.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename UMap>
class reduce_cc_stats_wf
{
public:
  typedef UMap result_type;

  template<typename Map>
  result_type operator()(Map m1p, Map m2p) const
  {
    typedef typename UMap::iterator IT;
    UMap count_map(m1p);
    UMap m2(m2p);

    std::pair<IT, bool> done;
    for (IT it = m2.begin(); it != m2.end(); ++it) {
      if (count_map.insert(*it).second == false)
        count_map[(*it).first] += (*it).second;
    }
    return count_map;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to extract the vertices belonging to a given CCID.
///
/// Stores the vertices satisfying the condition in the destination view.
/// @tparam VD The type of the vertex descriptor
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VD>
class extract_cc_stats_wf
{
  VD  m_cc_id;

public:
  typedef void result_type;

  extract_cc_stats_wf(VD const& ccid)
    : m_cc_id(ccid)
  { }

  template<typename Graph, typename Dest>
  void operator()(Graph g, Dest dest)
  {
    typedef typename Graph::vertex_iterator    VI;
    VI vi = g.begin();
    VI vi_e = g.end();
    for (; vi != vi_e; ++vi) {
      if ((*vi).property().cc() == m_cc_id)
        dest.push_back((*vi).descriptor());
    }
  }

  void define_type(typer& t)
  { t.member(m_cc_id); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to update neighbor vertices with CCID information.
///
/// Marks neighboring vertices with the sender's CCID if the target vertex's
/// CCID is larger than that of the sender.
/// @tparam VD The type of the vertex descriptor
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VD>
class update_func
{
public:
  typedef VD                        cc_id_type;
  typedef bool                      result_type;

  cc_id_type        m_cc_id;

  update_func(cc_id_type const& cc_id = std::numeric_limits<cc_id_type>::max())
    : m_cc_id(cc_id)
  { }

  template <class Vertex>
  bool operator()(Vertex&& target) const
  {
    if (target.property().cc() > m_cc_id ) {
      target.property().cc(m_cc_id);
      target.property().active(true);
      return true;
    }
    //else ignore.
    return false;
  }

  void define_type(typer& t)
  { t.member(m_cc_id); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function for computing connected-component IDs for each vertex.
///
/// Marks all vertices belonging to the same CC with the same
/// connected-component ID.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class cc_map_wf
{
public:
  using concurrency_model = sgl::weak_concurrency;

  template<class Vertex, class GraphVisitor>
  bool operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    using descriptor_type =
      typename std::decay<Vertex>::type::vertex_descriptor;

    if (v.property().active()) {
      v.property().active(false);
      graph_visitor.visit_all_edges(std::forward<Vertex>(v),
        update_func<descriptor_type>(v.property().cc()));
      return true;
    }
    return false;
  }
};
} // namespace cc_algo_detail


//////////////////////////////////////////////////////////////////////
/// @brief A Parallel Connected Components (CC) Algorithm.
///
/// Marks all vertices belonging to the same CC with the same
/// connected-component ID.
/// @param policy A policy for execution.
/// @param g The @ref graph_view over the input graph.
/// property_type on the vertex needs to have methods:
/// vertex_descriptor cc(void) -- for getting the connected-component id.
/// cc(vertex_descriptor) -- for setting the connected-component id.
/// bool active(void) -- for checking if vertices are active.
/// void active(bool) -- for marking vertices as active.
/// @return The number of iterations performed by the algorithm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Policy, typename GView>
size_t connected_components(Policy&& policy, GView& g)
{
  using namespace cc_algo_detail;
  // set everyone as the source
  map_func(init_cc_wf(), g);
  typedef update_func<typename GView::vertex_descriptor> update_func_t;
  return sgl::execute(
    std::forward<Policy>(policy), g, cc_map_wf{}, update_func_t{}, vp_reducer{}
  );
}

//////////////////////////////////////////////////////////////////////
/// @brief A Parallel Connected Components (CC) Algorithm.
///
/// Marks all vertices belonging to the same CC with the same
/// connected-component ID.
/// @param g The @ref graph_view over the input graph.
/// property_type on the vertex needs to have methods:
/// vertex_descriptor cc(void) -- for getting the connected-component id.
/// cc(vertex_descriptor) -- for setting the connected-component id.
/// bool active(void) -- for checking if vertices are active.
/// void active(bool) -- for marking vertices as active.
/// @return The number of iterations performed by the algorithm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GView>
size_t connected_components(GView& g)
{
  auto exec_policy = sgl::execution_policy<GView>{sgl::level_sync_policy{}};
  return connected_components(exec_policy, g);
}

//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to compute the number of vertices belonging to each
/// connected-component in the graph, given the connected-component of
/// each vertices.
/// @param g The @ref graph_view over the input graph.
/// property_type on the vertex needs to have methods:
/// vertex_descriptor cc(void) -- for getting the connected-component id.
/// cc(vertex_descriptor) -- for setting the connected-component id.
/// The connected-component for each vertex must be available, or be filled-in
/// by calling connected_components(g) algorithm before calling cc_stats.
/// @return A vector of pairs containing the CCID and the number of vertices
/// in that connected-component, for each CC. [Size: O(num CC)]
/// @todo The std::vector being returned here should be a pContainer
/// populated using the butterfly skeleton.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GView>
std::vector<std::pair<typename GView::vertex_descriptor, size_t> >
cc_stats(GView& g)
{
  using namespace cc_algo_detail;
  typedef std::map<typename GView::vertex_descriptor, size_t> umap_t;
  umap_t umap = map_reduce(extract_graph_cc_stats_wf<umap_t>(),
                           reduce_cc_stats_wf<umap_t>(), native_view(g));

  typedef std::vector<std::pair<typename GView::vertex_descriptor,
                                size_t> >  array_t;
  array_t cc_array(umap.begin(), umap.end());

  return cc_array;
}


//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to compute all vertices belonging to the specified
/// connected-component.
///
/// Given a Graph, and Connected-Component IDs for each vertex, this returns
/// all vertices belonging to the connected-component specified by CCid.
/// @param g The @ref graph_view over the input graph.
/// property_type on the vertex needs to have methods:
/// vertex_descriptor cc(void) -- for getting the connected-component id.
/// cc(vertex_descriptor) -- for setting the connected-component id.
/// The connected-component for each vertex must be available, or be filled-in
/// by calling connected_components(g) algorithm before calling cc_stats.
/// @param CCid The connected-component ID of the desired connected-component.
/// @param output The output view used to store and return vertices belonging
/// to the specified connected-component. Can be a view over
/// stapl::array<std::vector<VD> > or any 1-d view of std::vectors<VD>.
/// Each location stores the local vertices belonging to the specified CC.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GView, typename VertexDescriptor, typename ResView>
void cc_stats(GView& g, VertexDescriptor CCid, ResView& output)
{
  using namespace cc_algo_detail;
  map_func(extract_cc_stats_wf<VertexDescriptor>(CCid), native_view(g), output);
}


//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to compute if two specified vertices belong to the
/// same connected component (i.e., are connected through a path), based on
/// the given input graph and the Connected-Component IDs of vertices.
/// @param g The @ref graph_view over the input graph.
/// property_type on the vertex needs to have methods:
/// vertex_descriptor cc(void) -- for getting the connected-component id.
/// cc(vertex_descriptor) -- for setting the connected-component id.
/// The connected-component for each vertex must be available, or be filled-in
/// by calling connected_components(g) algorithm before calling is_same_cc.
/// @param v1 Vertex descriptor of the first vertex.
/// @param v2 Vertex descriptor of the second vertex.
/// @return True if both vertices are in the same Connected-Component, or
/// false otherwise.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GView>
bool is_same_cc(GView& g,
                typename GView::vertex_descriptor v1,
                typename GView::vertex_descriptor v2)
{
  return (*g.find_vertex(v1)).property().cc()
    == (*g.find_vertex(v2)).property().cc();
}

} // end namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_ALGORITHMS_CONNECTED_COMPONENTS_HPP
