/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_UNDIRECTED_TRIANGLE_COUNT_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_UNDIRECTED_TRIANGLE_COUNT_HPP

#include <stapl/containers/graph/algorithms/paradigms/graph_paradigm.hpp>
#include <stapl/containers/graph/functional.hpp>

namespace stapl {

namespace triangle_count_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Work-function to initialize vertices for triangle counting.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct undirected_triangle_count_init
{
  typedef void result_type;

  template <typename T>
  void operator()(T v) const
  {
    v.property() = 0;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to compute triangles and connected-triplets and
/// store the count on the target vertex.
///
/// @tparam Range The type of the edge-list from the neighbor vertex
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Range = std::vector<std::size_t>>
struct tc_neighbor_op
{
  using result_type = bool;

  std::size_t m_first;
  Range m_thirds;

  tc_neighbor_op() = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief first The vertex that this visit is originiating from
  /// @brief thirds Incident edges of the originating vertex
  //////////////////////////////////////////////////////////////////////
  tc_neighbor_op(std::size_t first, Range thirds)
    : m_first(first), m_thirds(std::move(thirds))
  { }

  template<typename Vertex>
  result_type operator()(Vertex&& second) const
  {
    using edge_type =
      typename std::decay<Vertex>::type::adj_edges_type::value_type;

    size_t num_triangles = second.property();

    auto edge_it = second.begin();

    auto desc = second.descriptor();

    // Perform set intersection between `second`'s edges and `m_thirds`,
    // where all of the edges are to vertices that are smaller than
    // `second`'s descriptor and first, because we're only counting
    // a triangle if its descriptors are in descending order:
    //     first > second > third.
    // We are assuming that both `m_thirds` and `second.edges()` are
    // sorted by target.
    for (auto const& third : m_thirds) {
      if (third.target() >= desc || third.target() >= m_first)
        break;

      while ((*edge_it).target() < third.target())
        edge_it++;

      // We are assuming that there are no multiedges, so count only
      // consider the first match.
      if ((*edge_it).target() == third.target())
        num_triangles += 1;
    }

    second.property() = num_triangles;
    return false;
  }

  void define_type(typer& t)
  {
    t.member(m_thirds);
  }
};

template<typename Range>
std::ostream& operator<<(std::ostream& os, tc_neighbor_op<Range> const& nop)
{
  os << nop.m_first << " {";

  for (auto third : nop.m_thirds)
    os << third.target() << " ";

  os << "}";

  return os;
}


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to compute the triangulation of each vertex
/// and push it to the vertex's neighbors.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct tc_vertex_op
{
  using result_type = bool;
  using concurrency_model = sgl::weak_concurrency;

  template<typename Vertex, typename GraphVisitor>
  bool operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    using edge_iterator = typename std::decay<Vertex>::type::adj_edge_iterator;

    constexpr bool is_contiguous =
      runtime::is_contiguous_iterator<edge_iterator>::value;

    return op_impl(
      std::forward<Vertex>(v),
      std::forward<GraphVisitor>(graph_visitor),
      std::integral_constant<bool, is_contiguous>{}
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Overload for edge lists that are not contiguous. Not
  ///        currently supported.
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex, typename GraphVisitor>
  bool op_impl(Vertex&& v, GraphVisitor&& graph_visitor, std::false_type) const
  {
    stapl::abort("This iterator is not contiguous");
    return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Overload for edge lists that are contiguous. Can use range
  ///        aggregation.
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex, typename GraphVisitor>
  bool op_impl(Vertex&& v, GraphVisitor&& graph_visitor, std::true_type) const
  {
    using edge_type =
      typename std::decay<Vertex>::type::adj_edges_type::value_type;
    using edge_iterator = typename std::decay<Vertex>::type::adj_edge_iterator;
    using nop_type = tc_neighbor_op<immutable_range_wrapper<edge_iterator>>;

    const std::size_t desc = v.descriptor();

    // Send edge list to all my neighbors that have a lower descriptor
    nop_type nop{desc, make_immutable_range(v.begin(), v.end())};

    graph_visitor.visit_all_edges_if(v, std::move(nop),
      [desc](edge_type const& e) {
        return e.target() < desc;
    });

    return false;
  }
};

}; // namespace triangle_count_impl;



//////////////////////////////////////////////////////////////////////
/// @brief Counts the number of triangles in the input @ref graph_view,
/// storing partial counts on vertices that are part of triangles.
///
/// This algorithm assumes that the input graph is undirected. Multiple
/// triangles between the same set of three vertices are counted exactly
/// once, even if there are multiple edges between targets.
///
/// The property to use for this algorithm is any integral type.
///
/// @param graph The @ref graph_view over the input graph.
/// @return The number of triangles detected
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GView>
std::size_t undirected_triangle_count(GView& graph)
{
  using namespace triangle_count_impl;

  // This algorithm relies on the property that edge lists are sorted by
  // target. That way, set intersection can be used to compare adjacency lists.
  graph.sort_edges(detail::edge_target_comp{});
  kla_params<GView> params;
  params.sort_edges = false;

  map_func(undirected_triangle_count_init(), graph);

  kla_paradigm(tc_vertex_op{}, tc_neighbor_op<>{}, graph, 0, params);

  return map_reduce(
    detail::extract_property<std::size_t>{},
    stapl::plus<std::size_t>{}, graph
  );
}

} // namespace stapl

#endif
