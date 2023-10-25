/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_EDGE_LIST_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_EDGE_LIST_HPP

#include <stapl/algorithms/sorting.hpp>
#include <stapl/skeletons/functional/butterfly.hpp>
#include <stapl/skeletons/executors/execute.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Work function to convert a sequence of edges to a local
///        adjacency list representation.
//////////////////////////////////////////////////////////////////////
struct edges_to_adjacency_list
{
  using result_type = std::map<std::size_t, std::vector<std::size_t>>;

  bool m_reverse_edges;

  edges_to_adjacency_list(bool reverse_edges)
    : m_reverse_edges(reverse_edges) {}

  template<typename E>
  result_type operator()(E&& edges) const
  {
    result_type adj;

    for (auto&& e : edges)
    {
      const std::size_t source = e.first;
      const std::size_t target = e.second;

      adj[source].push_back(target);

      if (m_reverse_edges)
        adj[target].push_back(source);
    }

    return adj;
  }

  void define_type(typer& t)
  {
    t.member(m_reverse_edges);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Butterfly work function to merge adjacency lists together
///        and filter out only the subset of vertices that is pertinent
///        for this level of the butterfly.
//////////////////////////////////////////////////////////////////////
struct merge_adjacencies
{
  using result_type = std::map<std::size_t, std::vector<std::size_t>>;

private:
  using partition_type = balanced_partition<indexed_domain<std::size_t>>;
  using domain_type = partition_type::value_type;

  /// Total size of the graph (n)
  std::size_t m_total_size;
  std::size_t m_butterfly_size;
  std::size_t m_butterfly_level;
  std::size_t m_butterfly_index;

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the range of vertex descriptors that should be returned
  ///   from this work function
  //////////////////////////////////////////////////////////////////////
  domain_type restricted_domain() const
  {
     const std::size_t parts = std::pow(2, m_butterfly_level+1);
     const std::size_t index_divisor = m_butterfly_size / 2;
     const std::size_t index = m_butterfly_index / index_divisor;

     partition_type part(domain_type(0, m_total_size-1), parts);

     return part[index];
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute a range of iterators from the map of vertices that
  ///  should be emitted.
  //////////////////////////////////////////////////////////////////////
  std::pair<result_type::const_iterator, result_type::const_iterator>
  restricted_range(result_type const& x) const
  {
    auto dom = restricted_domain();

    auto lower = x.lower_bound(dom.first());
    auto upper = x.upper_bound(dom.last());

    return std::make_pair(lower, upper);
  }

public:
  merge_adjacencies(std::size_t total_size) : m_total_size(total_size) { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the positions of this work function in the butterfly
  //////////////////////////////////////////////////////////////////////
  void set_position(std::size_t butterfly_size, std::size_t index1,
                    std::size_t index2, std::size_t level)
  {
    m_butterfly_size = 2*butterfly_size;
    m_butterfly_level = level;
    m_butterfly_index = index1;
  }

  template<typename E>
  result_type operator()(E&& lhs, E&& rhs) const
  {
    result_type merged = lhs;

    // TODO: this copy should not be necessary
    result_type rhs_copy = rhs;

    for (auto&& v : rhs_copy)
      for (auto&& e : v.second)
        merged[v.first].push_back(e);

    auto filtered_range = restricted_range(merged);

    result_type filtered(filtered_range.first, filtered_range.second);

    return filtered;
  }

  void define_type(typer& t)
  {
    t.member(m_total_size);
    t.member(m_butterfly_size);
    t.member(m_butterfly_level);
    t.member(m_butterfly_index);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function to take the output of the butterfly (which
///        should be the adjacency list for only the vertices on this
///        location) and write the edges back to the graph.
///
/// @tparam View The graph view to populate
//////////////////////////////////////////////////////////////////////
template<typename View>
struct write_adjacencies_to_graph
{
  using result_type = void;

  View* m_view;

  write_adjacencies_to_graph(View* view) : m_view(view) { }

  template<typename E>
  void operator()(E&& edges)
  {
    // TODO: this copy should not be necessary
    std::map<std::size_t, std::vector<std::size_t>> copy = edges;

    for (auto&& v : copy)
      for (auto&& e : v.second)
        m_view->add_edge_async(v.first, e);
  }

  void define_type(typer& t)
  {
    t.member(m_view);
  }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Transform a graph represented by a flat edge list into a
///        @c stapl::graph.
///
/// @warning This method can only be called when the number of processors is a
///    power of 2, as the butterfly requires that p is a power of 2. This can be
///    modified by performing a reduction to power of 2 before the butterfly and
///    a split after the butterfly.
///
/// @param edge_list View over an array of pairs where the first element is
///   the edge source and the second is the target.
/// @param g The graph to populate. This graph must be prepopulated with
///   vertices.
/// @param reverse_edges If true, every edge's reverse will also be
///   added to the graph
//////////////////////////////////////////////////////////////////////
template<typename EdgeListView, typename GraphView>
void transform_edge_list(EdgeListView edge_list, GraphView g,
                         bool reverse_edges = false)
{
  auto skeleton = skeletons::compose(
    skeletons::zip<1>(detail::edges_to_adjacency_list(reverse_edges)),
    skeletons::butterfly<true>(detail::merge_adjacencies(g.size())),
    skeletons::zip<1>(detail::write_adjacencies_to_graph<GraphView>(&g))
  );

  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    skeleton, edge_list
  );
}

} // namespace stapl

#endif
