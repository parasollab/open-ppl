/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_RANDOM_WALK_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_RANDOM_WALK_HPP

#include <stapl/containers/graph/algorithms/paradigms/graph_paradigm.hpp>
#include <stapl/containers/graph/generators/generator_base.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>

namespace stapl {

namespace random_walk_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Vertex-initializer functor for @ref random_walk().
///
/// Initializes vertices' traversal-level.
/// If the vertex is the source, the level is one (active).
/// All other vertices' levels are set to zero (inactive).
/// @tparam VD Type of the vertex-descriptor.
/// @tparam LevelType Type of the vertex-level.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <class VD, class LevelType>
class rw_init_wf
{
  typedef VD vd_type;
  vd_type    m_source;

public:
  typedef void result_type;

  rw_init_wf(vd_type source)
    : m_source(source)
  { }

  template <class Vertex>
  void operator()(Vertex v)
  {
    if (v.descriptor() == m_source) {
      v.property().level(LevelType(1));
    } else {
      v.property().level(LevelType(0));
    }
  }

  void define_type(typer& t)
  { t.member(m_source); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Visitor functor for @ref random_walk().
///
/// Updates the target vertex with random-walk-level information.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class update_func
{
public:
  size_t            m_level;
  size_t            m_walk_length;
  typedef bool result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param level The traversal-level of the target vertex.
  //////////////////////////////////////////////////////////////////////
  update_func(size_t level = 0, size_t walk_length = 0)
    : m_level(level), m_walk_length(walk_length)
  { }

  template <class Vertex>
  result_type operator()(Vertex&& target) const
  {
    // update target regardless of previous visits,
    // due to stateless Markov Chain.
    if (m_level <= m_walk_length)
    {
      target.property().level(m_level);
      return true;
    }
    else
      return false;
  }

  void define_type(typer& t)
  { t.member(m_level); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to continue the random-walk traversal from
/// the given vertex to one of its neighbors.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename C>
struct random_walk_wf
  : generators::rand_gen
{
  typedef generators::rand_gen base_type;
  C* m_res;
  size_t m_max_walk_length;

  random_walk_wf(C* c, size_t max_walk_length, size_t seed)
    :  base_type(seed),
       m_res(c),
       m_max_walk_length(max_walk_length)
  { }

  typedef bool result_type;

  template<typename Vertex, typename GraphVisitor>
  bool operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    if (v.property().level() == graph_visitor.level()
        && graph_visitor.level() <= m_max_walk_length) {
      // output the vertex in the walk.
      m_res->set_element(graph_visitor.level()-1, v.descriptor());
      // increment the visit counter on this vertex.
      v.property().visits(v.property().visits()+1);
      // visit a new neighbor, if one exists (vertex is not a sink).
      if (v.size() > 0) {
        const auto random_neighbor_idx
          = const_cast<random_walk_wf*>(this)->rand(v.size());
        auto vit = v.begin();
        for (size_t i=0; i<random_neighbor_idx; ++i)
          ++vit;
        // use the update-function to carry on traversal.
        graph_visitor.visit(
          (*vit).target(),
          update_func(v.property().level() + 1, m_max_walk_length));
      }
      return true;
    }
    return false;
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_res);
    t.member(m_max_walk_length);
  }
};

}; // namespace random_walk_impl;


//////////////////////////////////////////////////////////////////////
/// @brief Parallel Random-Walk Algorithm.
///
/// The algorithm starts at the given vertex and randomly chooses one of its
/// out edges as the next edge in the walk.  The algorithm moves to the chosen
/// edge's target vertex and repeats the process until the walk has reached the
/// desired length.  If a sink is reached before the desired length is
/// reached, the algorithm returns the path found up to that point.
/// @param graph The @ref graph_view over the input graph.
/// @param path_length The desired path length.
/// @param k The maximum amount of asynchrony allowed in each phase.
/// 0 <= k <= inf.
/// k == 0 implies level-synchronous random walk.
/// k >= D implies fully asynchronous random walk (D is diameter of graph).
/// @return The vertices in the resulting path. A value of infinity denotes
/// an invalid value, implying the path was shortened to that point.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GView>
array_view<stapl::array<typename GView::vertex_descriptor> >
random_walk(GView& graph,
            typename GView::vertex_descriptor const& source,
            size_t const& path_length, size_t const& k = 0)
{
  using namespace random_walk_impl;

  typedef typename GView::vertex_descriptor vd_type;
  typedef stapl::array<vd_type> array_t;

  array_t* res_vec = new array_t(path_length,
                                 std::numeric_limits<vd_type>::max());

  // Initialize the vertices.
  map_func(rw_init_wf<vd_type, size_t>(source), graph);

  auto rand_seed = graph.get_location_id() + time(NULL);
  size_t walk_length
    = graph_paradigm(random_walk_wf<array_t>(res_vec, path_length, rand_seed),
                     update_func(), graph, k);

  walk_length *= (k+1);
  if (walk_length < path_length)
    res_vec->resize(walk_length);
  return array_view<array_t>(res_vec);
}

} // namespace stapl
#endif
