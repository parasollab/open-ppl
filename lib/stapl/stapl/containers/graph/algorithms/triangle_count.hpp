/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_TRIANGLE_COUNT_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_TRIANGLE_COUNT_HPP

#include <stapl/containers/graph/algorithms/execute.hpp>

namespace stapl {

namespace triangle_count_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to add elements of a pair for triangle counting.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct tc_add_pair
{
  typedef std::pair<size_t, size_t> result_type;

  template <typename T>
  result_type operator()(T t1, T t2) const
  { return std::make_pair(t1.first+t2.first, t1.second+t2.second); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to initialize vertices for triangle counting.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct triangle_count_init_wf
{
  typedef void result_type;

  template <typename T>
  void operator()(T v) const
  {
    v.property().num_triangles(0);
    v.property().num_connected_triplets(0);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to return the number of triangles each vertex
/// is a part of, along with the number of connected-tiplets.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct get_count_and_triplets
{
  typedef std::pair<size_t, size_t> result_type;

  template <typename T>
  result_type operator()(T v) const
  {
    return std::make_pair(v.property().num_triangles(),
                          v.property().num_connected_triplets());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to compute triangles and connected-triplets and
/// store the count on the target vertex.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct update_func
{
  typedef bool result_type;
  typedef size_t vd_type;

  vd_type m_second;
  std::vector<vd_type> m_thirds;

  update_func(vd_type const& second = std::numeric_limits<vd_type>::max())
    : m_second(second), m_thirds()
  { }

  update_func(vd_type const& second, vd_type const& third)
    : m_second(second), m_thirds(1, third)
  { }

  update_func(vd_type const& second,
              std::vector<vd_type> const& thirds)
    : m_second(second), m_thirds(thirds)
  { }

  template <class Vertex>
  bool operator()(Vertex&& target) const
  {
    if (target.descriptor() <= m_second)
      return false;
    if (m_thirds.empty())  //second visit.
      target.property().waiting_queue_add(m_second);
    else {  // third visit, search for closing edge.
      size_t num_triangles = target.property().num_triangles();
      size_t num_connected_triplets
        = target.property().num_connected_triplets();
      for (auto const& third : m_thirds) {
        for (auto const& e : target) {
          ++num_connected_triplets;
          if (e.target() == third) {
            ++num_triangles;
            //            break;
          }
        }
      }
      target.property().num_triangles(num_triangles);
      target.property().num_connected_triplets(num_connected_triplets);
      return false;
    }
    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_second);
    t.member(m_thirds);
  }
};

std::ostream& operator<<(std::ostream& os, update_func const& uf)
{
  const uint32_t phase = uf.m_thirds.empty() ? 2 : 3;
  os << "phase " << phase << ": " << uf.m_second << " {";

  for (auto third : uf.m_thirds)
    os << third << " ";

  os << "}";

  return os;
}


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to compute the triangulation of each vertex
/// and push it to the vertex's neighbors.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct triangle_count_wf
{
  typedef bool result_type;

  template<typename Vertex, typename GraphVisitor>
  bool operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    if (graph_visitor.level() == 1) {  // first visit.
      graph_visitor.visit_all_edges(v, update_func(v.descriptor()));
      return true;
    } else if (!v.property().waiting_queue_empty()) {  // second visit.
      graph_visitor.visit_all_edges(v,
                                    update_func(v.descriptor(),
                                                v.property().waiting_queue()));
      v.property().waiting_queue_clear();
      return true;
    }
    return false;
  }
};

}; // namespace triangle_count_impl;


//////////////////////////////////////////////////////////////////////
/// @brief Parallel Triangle Counting and Clustering Coefficient Algorithm.
///
/// Counts the number of triangles in the input @ref graph_view,
/// storing partial counts on vertices that are part of triangles.
/// Also computes the clustering-coefficient of the input @ref graph_view,
/// storing counts of connected triplets on vertices.
/// Formula:
/// C(G) = 3x#triangles / #connected-triplets-of-vertices.
///      = #closed-triplets / #connected-triplets-of-vertices.
///
/// @param graph The @ref graph_view over the input graph.
/// @param k The maximum amount of asynchrony allowed in each phase.
/// 0 <= k <= inf.
/// k == 0 implies level-synchronous paradigm.
/// k >= D implies fully asynchronous paradigm (D is diameter of graph).
/// @return pair of the number of triangles detected and the
/// clustering-coefficient of the graph.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GView>
std::pair<size_t, double> triangle_count(GView& graph, size_t k=0)
{
  using namespace triangle_count_impl;
  map_func(triangle_count_init_wf(), graph);
  kla_paradigm(triangle_count_wf{}, update_func{}, graph, k);
  auto x = map_reduce(get_count_and_triplets(), tc_add_pair(), graph);
  return std::make_pair(x.first, double(3.0*x.first) / double(x.second));
}

} // namespace stapl
#endif
