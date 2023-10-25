/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_RANDOM_DAG_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_RANDOM_DAG_HPP

#include <stapl/containers/graph/generators/generator.hpp>
#include <stapl/unordered_map.hpp>
#include <vector>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor which adds edges from an unordered_map adjacency list.
//////////////////////////////////////////////////////////////////////
template <typename UMapTargets>
struct dag_neighbors
{
  UMapTargets* m_umap_targets;
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param umap_targets A stapl::unordered map where each key is a vertex id
  ///   and each value is a list of its outgoing neighbors
  //////////////////////////////////////////////////////////////////////
  dag_neighbors(UMapTargets& umap_targets)
    : m_umap_targets(&umap_targets)
  { }

  template<typename Vertex, typename Graph>
  void operator()(Vertex v, Graph& view)
  {
    std::vector<std::size_t> targets = (*m_umap_targets)[v.descriptor()];

    // Remove duplicates
    std::sort(targets.begin(), targets.end());
    targets.erase(std::unique(targets.begin(), targets.end()), targets.end());

    for (auto e: targets)
    {
      view.add_edge_async(v.descriptor(), e);
    }
  }

  void define_type(typer& t)
  {
    t.member(m_umap_targets);
  }
};



//////////////////////////////////////////////////////////////////////
/// @brief Functor which generates the unordered_map adjacency list.
//////////////////////////////////////////////////////////////////////
template <typename UMapTargets>
struct wf_generate_dag_neighbors
  : protected rand_gen
{
  size_t m_num_vertices;
  size_t m_max_degree;

  typedef void result_type;

  wf_generate_dag_neighbors(size_t num_vertices, size_t max_degree)
    : m_num_vertices(num_vertices), m_max_degree(max_degree)
  { }


  template<typename UMapView>
  void operator()(size_t v_id, UMapView& view)
  {
    size_t num_neighbors = this->rand(1, m_max_degree + 1);

    for (size_t i = 0; i < num_neighbors; ++i)
    {
      size_t neighbor = v_id;
      while (neighbor == v_id)
        neighbor = this->rand(m_num_vertices);

      auto mm = std::minmax(v_id, neighbor);
      view[mm.first].push_back(mm.second);
    }
  }

  void define_type(typer& t)
  {
    t.member(m_num_vertices);
    t.member(m_max_degree);
  }
};


template <typename UMapView>
void generate_acyclic_edges_list(UMapView& targets, size_t n,
                                 size_t num_neighbors)
{
  stapl::generators::detail::wf_generate_dag_neighbors<UMapView>
    wf_generate(n,num_neighbors);

  stapl::map_func(wf_generate,
                  stapl::counting_view<size_t>(n),
                  stapl::make_repeat_view(targets));
}
} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Generates a random directed acyclic graph (DAG).
///
/// The generated random directed acyclic graph will contain n vertices
/// and potentially multiple components. Each vertex will attempt to
/// generate between 1 and `max_num_neighbors` neighbors and the edges
/// are added such that for every edge (u,v), u < v.
///
/// This function mutates the input graph.
/// @param g A view over the graph to generate.
/// @param n Number of vertices in the generated graph.
/// @return The original view, now containing the generated graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_random_dag(GraphView& g, size_t n, size_t max_num_neighbors=6)
{
  stapl::unordered_map<size_t,std::vector<size_t> > umap_targets;
  typedef stapl::map_view<stapl::unordered_map<size_t, std::vector<size_t> > >
    UMapView;

  UMapView umap_targets_vw(umap_targets);
  stapl::generators::detail::generate_acyclic_edges_list(
    umap_targets_vw, n, max_num_neighbors
  );

  typedef stapl::generators::detail::dag_neighbors<UMapView> ef_t;
  ef_t dag_gen(umap_targets_vw);
  return stapl::generators::make_generator<GraphView, ef_t>(g, n, dag_gen)();
}


//////////////////////////////////////////////////////////////////////
/// @brief Generates a random directed acyclic graph (DAG).
///
/// The generated random directed acyclic graph will contain n vertices
/// and potentially multiple components. Each vertex will attempt to
/// generate between 1 and `max_num_neighbors` neighbors and the edges
/// are added such that for every edge (u,v), u < v.
///
/// The returned view owns its underlying container.
///
/// @param n Number of vertices in the generated graph.
/// @param max_num_neighbors 
/// @return A view over the generated graph.
///
/// @b Example
/// @snippet random_dag.cc Example
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_random_dag(size_t n, size_t max_num_neighbors=6)
{
  stapl::unordered_map<size_t,std::vector<size_t> > umap_targets;
  typedef stapl::map_view<stapl::unordered_map<size_t,std::vector<size_t> > >
    UMapView;

  UMapView umap_targets_vw(umap_targets);
  stapl::generators::detail::generate_acyclic_edges_list(
    umap_targets_vw, n, max_num_neighbors
  );

  typedef stapl::generators::detail::dag_neighbors<UMapView> ef_t;
  ef_t dag_gen(umap_targets_vw);
  return stapl::generators::make_generator<GraphView, ef_t>(n, dag_gen)();
}

} // namespace generators

} // namespace stapl
#endif
