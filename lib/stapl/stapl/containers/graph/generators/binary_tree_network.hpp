/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_BINARY_TREE_NETWORK_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_BINARY_TREE_NETWORK_HPP

#include <stapl/containers/graph/generators/generator_base.hpp>

namespace stapl {

namespace generators {

//////////////////////////////////////////////////////////////////////
/// @brief Graph generator for a binary tree network graph.
/// @tparam G Type of the graph view used to construct the graph.
///
/// @see make_binary_tree_network
//////////////////////////////////////////////////////////////////////
template <typename G>
struct binary_tree_network
  : public generator_base<binary_tree_network<G> >
{
  typedef generator_base<binary_tree_network<G> > base_type;

protected:
  size_t m_levels;
  size_t m_n;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param levels Number of levels in each generated tree.
  //////////////////////////////////////////////////////////////////////
  binary_tree_network(size_t levels)
    : base_type(2*(pow(2, levels)-1)), m_levels(levels),
      m_n(pow(2, levels)-1)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param g View over the graph to generate.
  /// @param levels Number of levels in each generated tree.
  //////////////////////////////////////////////////////////////////////
  binary_tree_network(G& g, size_t levels)
    : base_type(g, 2*(pow(2, levels)-1)), m_levels(levels),
      m_n(pow(2, levels)-1)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function which is called to add vertices to the graph.
  //////////////////////////////////////////////////////////////////////
  virtual void add_vertices()
  {
    add_vertices_top();
    add_vertices_bottom();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function which is called to add edges to the graph.
  //////////////////////////////////////////////////////////////////////
  virtual void add_edges()
  {
    stapl_assert(this->graph().size() == 2*m_n,
      "binary_tree_network::add_edges(): incorrect number of vertices");

    add_edges_top();
    add_edges_bottom();
    add_edges_middle();
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Function to add vertices for the first tree.
  //////////////////////////////////////////////////////////////////////
  void add_vertices_top()
  { add_vertices_impl(0); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function to add vertices for the second tree.
  //////////////////////////////////////////////////////////////////////
  void add_vertices_bottom()
  { add_vertices_impl(pow(2, m_levels)-1); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function to add edges for the first tree.
  //////////////////////////////////////////////////////////////////////
  void add_edges_top()
  { add_edges_impl(0); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function to add edges for the second tree.
  //////////////////////////////////////////////////////////////////////
  void add_edges_bottom()
  { add_edges_impl(pow(2, m_levels)-1); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function to add edges to link the two trees.
  //////////////////////////////////////////////////////////////////////
  void add_edges_middle()
  {
    size_t top_start = pow(2, m_levels-1)-1;
    size_t top_end = pow(2, m_levels)-1;
    size_t offset = pow(2, m_levels)-1;

    if (get_location_id() == 0)
      for (size_t i = top_start; i < top_end; ++i)
        this->graph().add_edge_async(i, i+offset);
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Internal function which adds vertices for a tree.
  /// @param offset Starting id of the first added vertex.
  //////////////////////////////////////////////////////////////////////
  void add_vertices_impl(size_t offset)
  {
    const size_t p = get_num_locations();
    const size_t r = get_location_id();

    for (size_t i = 0; i < m_levels; ++i) {
      size_t vertices_at_level = pow(2, i);

      size_t level_start = vertices_at_level - 1 + offset;
      if (vertices_at_level < p) {
        for (size_t j = 0; j < vertices_at_level; ++j)
          if (r == 2*j)
            this->graph().add_vertex(level_start+j,
                                     typename G::vertex_property());
      } else {
        size_t block_size = vertices_at_level/get_num_locations();

        size_t subview_start = block_size*get_location_id() + level_start;
        size_t subview_end = subview_start + block_size;

        for (size_t i = subview_start; i < subview_end; ++i)
          this->graph().add_vertex(i, typename G::vertex_property());
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Internal function which adds edges for a tree.
  /// @param offset Starting id of the first vertex of the tree.
  //////////////////////////////////////////////////////////////////////
  void add_edges_impl(size_t offset)
  {
    size_t block_size = this->graph().size()/get_num_locations();

    size_t subview_start = block_size*get_location_id();
    size_t subview_end = subview_start + block_size;

    size_t last_level = m_n;

    for (size_t i = subview_start; i < subview_end; ++i) {
      if (i*2+2 < last_level) {
        for (size_t j = 1; j <= 2; ++j) {
          size_t src = offset == 0 ? i + offset : i*2+j+offset;
          size_t dst = offset == 0 ? i*2+j+offset : i + offset;

          this->graph().add_edge_async(src, dst);
        }
      }
    }
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_levels);
  }

};


//////////////////////////////////////////////////////////////////////
/// @brief Generate a binary tree network.
///
/// The binary tree network will contain 2*num_levels levels.
/// Network consists of two num_level binary trees which share the same leaves.
/// The root of the first tree forms the source, and the root of the second
/// forms the sink of the network.
///
/// This function mutates the input graph.
///
/// @note The type of the graph needs to be dynamic.
///
/// @param g A view over the graph to generate.
/// @param num_levels Number of levels in each of the 2 generated trees.
/// @return The original view, now containing the generated graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_binary_tree_network(GraphView& g, size_t num_levels)
{
  return binary_tree_network<GraphView>(g, num_levels)();
}

//////////////////////////////////////////////////////////////////////
/// @brief Generate a binary tree network.
///
/// The binary tree network will contain 2*num_levels levels.
/// Network consists of two num_level binary trees which share the same leaves.
/// The root of the first tree forms the source, and the root of the second
/// forms the sink of the network.
///
/// The returned view owns its underlying container.
///
/// @note The type of the graph needs to be dynamic.
///
/// @param num_levels Number of levels in each of the 2 generated trees.
/// @return A view over the generated graph.
///
/// ! https://i.imgur.com/jUtBdpa.png
///
/// @b Example
/// @snippet binary_tree_network.cc Example
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_binary_tree_network(size_t num_levels)
{
//  using graph_type = typename view_traits<GraphView>::container;
//  GraphView view(new graph_type(std::pow(2, num_levels)-1));
  return binary_tree_network<GraphView>(num_levels)();
}

} // namespace generators

} // namespace stapl

#endif
