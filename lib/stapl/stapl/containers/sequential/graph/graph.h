/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_HPP

#include <vector>
#include "adjacency_list_core_graph.h"
#include "undirected_util.h"
#include "graph_traits.h"

namespace stapl {

namespace sequential {

//////////////////////////////////////////////////////////////////////
/// @brief Class implementing a directed graph.
/// @ingroup graphBase
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the descriptor, model, storage, etc.
//////////////////////////////////////////////////////////////////////
template <class Traits>
class DG
  : public Traits::core_graph_type
{
  typedef typename Traits::core_graph_type base_type;
public:
  typedef typename base_type::vertex_descriptor vertex_descriptor;
  typedef typename base_type::edge_descriptor edge_descriptor;

  typedef typename base_type::vertex_iterator vertex_iterator;
  typedef typename base_type::const_vertex_iterator const_vertex_iterator;

  typedef typename base_type::adj_edge_iterator adj_edge_iterator;
  typedef typename base_type::const_adj_edge_iterator const_adj_edge_iterator;


  //////////////////////////////////////////////////////////////////////
  /// @brief Create a directed graph with the specified number of vertices.
  //////////////////////////////////////////////////////////////////////
  DG(size_t sz)
    : base_type(sz)
  {
  }

  virtual ~DG() = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Method used to check if the graph is directed or not.
  /// This class always returns true;
  //////////////////////////////////////////////////////////////////////
  bool is_directed() const
  {
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the out degree of the specified vertex.
  //////////////////////////////////////////////////////////////////////
  size_t get_out_degree(vertex_descriptor const& vd)
  {
    const_vertex_iterator cvi = this->find_vertex(vd);
    if (cvi != this->end())
      return (*cvi).size();
    else
      return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of successors of the specified vertex,
  /// and initializes the provided vector with their corresponding descriptors.
  /// @param vd The specified vertex.
  /// @param succ The output list of the descriptors of the successors.
  /// @return The number of successors.
  //////////////////////////////////////////////////////////////////////
  size_t get_successors(vertex_descriptor& vd,
                        std::vector<vertex_descriptor>& succ) const
  {
    succ.clear();
    const_vertex_iterator cvi = this->find_vertex(vd);
    if (cvi != this->end()) {
      const_adj_edge_iterator cei = (*cvi).begin();
      while (cei != (*cvi).end()) {
        succ.push_back((*cei).target());
        ++cei;
      }
      return succ.size();
    } else
      return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reverse all edges in the directed graph
  /// @note Uses O(E) extra storage, O(E) time.
  //////////////////////////////////////////////////////////////////////
  void reverse_all_edges(void)
  {
    //is there a more efficient way ?
    //first collect all edges
    std::vector<edge_descriptor> edges;
    edges.reserve(this->get_num_edges());
    for (const_vertex_iterator cvi = this->begin(); cvi != this->end(); ++cvi) {
      const_adj_edge_iterator cei = (*cvi).begin();
      while (cei != (*cvi).end()) {
        edges.push_back(*cei);
        ++cei;
      }
    }
    //delete all edges
    for (typename std::vector<edge_descriptor>::iterator it = edges.begin();
        it != edges.end(); ++it) {
      base_type::delete_edge(*it);
    }
    //reinsert edges with source/target reversed
    for (typename std::vector<edge_descriptor>::iterator it = edges.begin();
        it != edges.end(); ++it) {
      base_type::add_edge(*it.reverse);
    }
  }

};



//////////////////////////////////////////////////////////////////////
/// @brief Class implementing an undirected graph.
/// @ingroup graphBase
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the descriptor, model, storage, etc.
//////////////////////////////////////////////////////////////////////
template <class Traits>
class UG
  : public Traits::core_graph_type
{
  typedef typename Traits::core_graph_type base_type;
public:
  typedef typename base_type::vertex_descriptor vertex_descriptor;
  typedef typename base_type::edge_descriptor edge_descriptor;
  typedef typename base_type::vertex_property vertex_property;
  typedef typename base_type::edge_property edge_property;

  typedef typename base_type::vertex_iterator vertex_iterator;
  typedef typename base_type::const_vertex_iterator const_vertex_iterator;

  typedef typename base_type::adj_edge_iterator adj_edge_iterator;
  typedef typename base_type::const_adj_edge_iterator const_adj_edge_iterator;

  typedef typename base_type::edge_iterator edge_iterator;
  typedef typename base_type::const_edge_iterator const_edge_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an undirected graph with the specified number of vertices.
  //////////////////////////////////////////////////////////////////////
  UG(size_t sz)
    : base_type(sz)
  {
  }

  UG(UG const& other)
  {
    // in principle this can be done faster
    copy_graph(const_cast<UG&>(other), *this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Method used to check if the graph is directed or not.
  /// This class always returns false;
  //////////////////////////////////////////////////////////////////////
  bool is_directed() const
  {
    return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator over all the edges of the graph, pointing
  /// to the starting edge in the graph.
  /// Skips sibling edges (where target < source), to avoid duplicating
  /// edges in undirected graph.
  //////////////////////////////////////////////////////////////////////
  edge_iterator edges_begin()
  {
    //false tells edge iterator to skip sister edges
    return edge_iterator(base_type::edges_begin(), false);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator over all the edges of the graph, pointing
  /// to the starting edge in the graph.
  /// Skips sibling edges (where target < source), to avoid duplicating
  /// edges in undirected graph.
  //////////////////////////////////////////////////////////////////////
  const_edge_iterator edges_begin() const
  {
    return const_edge_iterator(base_type::edges_begin(), false);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator over all the edges of the graph, pointing
  /// to one past the ending edge in the graph.
  /// Skips sibling edges (where target < source), to avoid duplicating
  /// edges in undirected graph.
  //////////////////////////////////////////////////////////////////////
  edge_iterator edges_end()
  {
    return edge_iterator(base_type::edges_end(), false);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator over all the edges of the graph, pointing
  /// to one past the ending edge in the graph.
  /// Skips sibling edges (where target < source), to avoid duplicating
  /// edges in undirected graph.
  //////////////////////////////////////////////////////////////////////
  const_edge_iterator edges_end() const
  {
    return const_edge_iterator(base_type::edges_end(), false);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the specified edge in both directions.
  ///
  /// Undirected graph doesn't allow for self loops; this is
  /// consistent with other graph libraries like BGL, LEDA.
  /// @param ed The descriptor of the edge to be added.
  /// @return The edge descriptor of the added edge. If successful, the
  /// id() of the descriptor is populated with the actual edge's id, otherwise
  /// set to std::numeric_limits<size_t>::max().
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& ed)
  {
    return add_edge(ed, edge_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the specified edge in both directions, with the
  /// given property.
  ///
  /// Undirected graph doesn't allow for self loops; this is
  /// consistent with other graph libraries like BGL, LEDA.
  /// @param ed The descriptor of the edge to be added.
  /// @return The edge descriptor of the added edge. If successful, the
  /// id() of the descriptor is populated with the actual edge's id, otherwise
  /// set to std::numeric_limits<size_t>::max().
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& ed, edge_property const& p)
  {
    if (ed.source() == ed.target())
      return edge_descriptor(ed.source(), ed.target(), INVALID_VALUE );
    return add_edge_pair(*this, ed, p);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of undirected edges in the graph. The number
  /// of edges is half the number of directed edges in the graph.
  //////////////////////////////////////////////////////////////////////
  size_t get_num_edges(void) const
  {
    return base_type::get_num_edges() / 2;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the specified undirected edge (removes the directed
  /// edge and its sibling edge).
  /// @return True if the delete was successful, or false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool delete_edge(edge_descriptor const& ed)
  {
    vertex_iterator vi;
    adj_edge_iterator ei;
    bool res, res1, res2;
    res = res1 = res2 = true;
    res = base_type::find_edge(ed, vi, ei);
    if (res) {
      //next r_ed is the real the edge descriptor with the edge id
      //initialized for the case when there may be duplicated edges
      edge_descriptor r_ed = (*ei).descriptor();
      //clear the property in a special way since it is shared by two edges
      undirected_edge_helper<edge_descriptor, adj_edge_iterator,
          typename base_type::edge_property>::clear(ei);
      res1 = base_type::delete_edge(r_ed);          // delete the direct edge
      res2 = base_type::delete_edge(reverse(r_ed)); // and the reverse one
      assert(res1 && res2);
    }
    return res;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds one of the two edges defining an undirected edge.
  ///
  /// As a rule, we report the edge where source is smaller than the
  /// destination.
  /// @param ed The descriptor of the edge to be found.
  /// @param vi The output vertex iterator pointing to the source-vertex
  /// of the edge.
  /// @param ei The output edge iterator pointing to the specified edge.
  /// @return True if the edge was found, or false otherwise.
  /// @todo Return a pair<bool, pair<vertex_iterator, adj_edge_iterator> >
  /// instead of passing them in as parameters.
  //////////////////////////////////////////////////////////////////////
  bool find_edge(edge_descriptor const& ed, vertex_iterator& vi,
                 adj_edge_iterator& ei)
  {
    if (ed.source() < ed.target())
      return base_type::find_edge(ed, vi, ei);
    else
      return base_type::find_edge(reverse(ed), vi, ei);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds one of the two edges defining an undirected edge.
  ///
  /// As a rule, we report the edge where source is smaller than the
  /// destination.
  /// @param ed The descriptor of the edge to be found.
  /// @param vi The output vertex iterator pointing to the source-vertex
  /// of the edge.
  /// @param ei The output edge iterator pointing to the specified edge.
  /// @return True if the edge was found, or false otherwise.
  /// @todo Return a pair<bool, pair<vertex_iterator, adj_edge_iterator> >
  /// instead of passing them in as parameters.
  //////////////////////////////////////////////////////////////////////
  bool find_edge(edge_descriptor const& ed, const_vertex_iterator& vi,
                 const_adj_edge_iterator& ei) const
  {
    if (ed.source() < ed.target())
      return base_type::find_edge(ed, vi, ei);
    else
      return base_type::find_edge(reverse(ed), vi, ei);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erase all vertices and edges in the graph,
  /// and any associated properties.
  //////////////////////////////////////////////////////////////////////
  void clear()
  {
    //first erase all properties
    vertex_iterator vi = this->begin();
    while (vi != this->end()) {
      adj_edge_iterator ei = (*vi).begin();
      while (ei != (*vi).end()) {
        if ((*ei).source() < (*ei).target())
          undirected_edge_helper<edge_descriptor, adj_edge_iterator,
              typename base_type::edge_property>::clear(ei);
        ++ei;
      }
      ++vi;
    }
    //second erase all vertices and edges
    base_type::clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the specified vertex and all edges pointing to it
  /// and from it.
  /// @return True if vertex was deleted, or false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool delete_vertex(vertex_descriptor const& vd)
  {
    vertex_iterator vi = this->find_vertex(vd);
    if (vi != this->end()) {
      //we need to store them because they will be deleted two at a
      //time and there may be some inconsistencies due to vector
      //implementation for the edgelist
      std::vector<edge_descriptor> v_vd;
      v_vd.reserve((*vi).size());
      adj_edge_iterator ei = (*vi).begin();
      while (ei != (*vi).end()) {
        v_vd.push_back((*ei).descriptor());
        ++ei;
      }
      for (typename std::vector<edge_descriptor>::iterator it = v_vd.begin();
          it != v_vd.end(); ++it) {
        this->delete_edge(*it);
      }
      //next call can be optimized by adding another method in the
      //base that just removes the vertex without trying to remove
      //the edges again;
      base_type::delete_vertex(vd);
      return true;
    } else
      return false;
  }

};



//////////////////////////////////////////////////////////////////////
/// @brief Class implementing a multiedges graph.
/// @ingroup graphBase
///
/// A multiedges graph allows multiple edges between the same source and
/// target vertices.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the descriptor, model, storage, etc.
//////////////////////////////////////////////////////////////////////
template <class Traits>
class MG
  : public Traits::directness_type
{
  typedef typename Traits::directness_type base_type;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Create a multiedges graph with the specified number of vertices.
  //////////////////////////////////////////////////////////////////////
  MG(size_t sz)
    : base_type(sz)
  { }

  virtual ~MG(void) = default;
};



//////////////////////////////////////////////////////////////////////
/// @brief Class implementing a non-multiedges graph.
/// @ingroup graphBase
///
/// A non-multiedges graph does not allow multiple edges between the
/// same source and target vertices.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the descriptor, model, storage, etc.
//////////////////////////////////////////////////////////////////////
template <class Traits>
class NMG
  : public Traits::directness_type
{
  typedef typename Traits::directness_type base_type;
public:
  typedef typename base_type::edge_descriptor edge_descriptor;
  typedef typename base_type::edge_property edge_property;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a non-multiedges graph with the specified number of
  /// vertices.
  //////////////////////////////////////////////////////////////////////
  NMG(size_t sz)
      : base_type(sz)
  {
  }

  virtual ~NMG() = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the specified edge to the graph, if it doesn't already exist.
  /// @param ed The descriptor of the edge to be added.
  /// @return The edge descriptor of the added edge. If successful, the
  /// id() of the descriptor is populated with the actual edge's id, otherwise
  /// set to std::numeric_limits<size_t>::max().
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& ed)
  {
    return add_edge(ed, edge_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the specified edge to the graph, if it doesn't already exist,
  /// with the given property.
  /// @param ed The descriptor of the edge to be added.
  /// @return The edge descriptor of the added edge. If successful, the
  /// id() of the descriptor is populated with the actual edge's id, otherwise
  /// set to std::numeric_limits<size_t>::max().
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& ed, edge_property const& p)
  {
    typename base_type::vertex_iterator vi;
    typename base_type::adj_edge_iterator ei;
    // can we optimize to do only one search? either have
    // add_edge in the base to check if the edge exist or have
    // another add_edge that takes the vertex iterator as argument
    if (this->find_edge(ed, vi, ei))
      return edge_descriptor(ed.source(), ed.target(), INVALID_VALUE );
    else
      return base_type::add_edge(ed, p);
  }
};



//////////////////////////////////////////////////////////////////////
/// @brief Helper class using partial template specialization to translate
/// @ingroup graphBaseUtil
/// from a flag (DIRECTED, UNDIRECTED, MULTIEDGES, NONMULTIEDGES) to
/// its corresponding class (DG, UG, MG, NMG)
//////////////////////////////////////////////////////////////////////
template <class Traits, graph_attributes type>
struct graph_type
{
  //compiler error will be generated if invalid type is used
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper class using partial template specialization to translate
/// from a flag (DIRECTED, UNDIRECTED, MULTIEDGES, NONMULTIEDGES) to
/// its corresponding class (DG, UG, MG, NMG)
/// Specialized for Directed graph.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <class Traits>
struct graph_type<Traits, DIRECTED>
{
  typedef DG<Traits> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper class using partial template specialization to translate
/// from a flag (DIRECTED, UNDIRECTED, MULTIEDGES, NONMULTIEDGES) to
/// its corresponding class (DG, UG, MG, NMG)
/// Specialized for UnDirected graph.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <class Traits>
struct graph_type<Traits, UNDIRECTED>
{
  typedef UG<Traits> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper class using partial template specialization to translate
/// from a flag (DIRECTED, UNDIRECTED, MULTIEDGES, NONMULTIEDGES) to
/// its corresponding class (DG, UG, MG, NMG)
/// Specialized for Multiedges graph.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <class Traits>
struct graph_type<Traits, MULTIEDGES>
{
  typedef MG<Traits> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper class using partial template specialization to translate
/// from a flag (DIRECTED, UNDIRECTED, MULTIEDGES, NONMULTIEDGES) to
/// its corresponding class (DG, UG, MG, NMG)
/// Specialized for Non-Multiedges graph.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <class Traits>
struct graph_type<Traits, NONMULTIEDGES>
{
  typedef NMG<Traits> type;
};



//////////////////////////////////////////////////////////////////////
/// @brief The STAPL sequential graph class.
/// @ingroup graph
///
/// Inherits from a number of classes as specified in the traits class.
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP type of property for the vertex. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the descriptor, model, storage, etc. The default traits
/// class is @ref adj_graph_traits.
//////////////////////////////////////////////////////////////////////
template <graph_attributes D, graph_attributes M,
          class VertexP = properties::no_property,
          class EdgeP = properties::no_property,
          class Traits = adj_graph_traits<D, M, VertexP, EdgeP> >
class graph
  : public Traits::multiplicity_type
{
  typedef typename Traits::multiplicity_type base_type;
  typedef typename Traits::core_graph_type core_base_type;
public:
  static const graph_attributes d_type = D; //directedness
  static const graph_attributes m_type = M; //edge multiplicity

  typedef VertexP vertex_property;
  typedef EdgeP edge_property;

  typedef typename core_base_type::vertex_descriptor vertex_descriptor;
  typedef typename core_base_type::edge_descriptor edge_descriptor;

  typedef typename core_base_type::vertex_iterator vertex_iterator;
  typedef typename core_base_type::const_vertex_iterator const_vertex_iterator;

  typedef typename core_base_type::adj_edge_iterator adj_edge_iterator;
  typedef typename core_base_type::const_adj_edge_iterator
    const_adj_edge_iterator;

  typedef typename core_base_type::edge_iterator edge_iterator;
  typedef typename core_base_type::const_edge_iterator const_edge_iterator;

  typedef typename vertex_iterator::value_type vertex_reference;
  typedef typename const_vertex_iterator::value_type const_vertex_reference;

  typedef typename vertex_iterator::value_type reference;
  typedef typename const_vertex_iterator::value_type const_reference;

  typedef typename edge_iterator::value_type edge_reference;
  typedef typename const_edge_iterator::value_type const_edge_reference;

  typedef typename adj_edge_iterator::value_type adj_edge_reference;
  typedef typename const_adj_edge_iterator::value_type const_adj_edge_reference;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a graph with the specified number of vertices.
  //////////////////////////////////////////////////////////////////////
  graph(size_t sz = 0)
      : base_type(sz)
  {
  }

  ~graph()
  {
    this->clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the specified edge to the graph.
  /// @param ed The descriptor of the edge to be added.
  /// @return The edge descriptor of the added edge. If successful, the
  /// id() of the descriptor is populated with the actual edge's id, otherwise
  /// set to std::numeric_limits<size_t>::max().
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& ed)
  {
    return base_type::add_edge(ed);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the specified edge with the given property to the graph.
  /// @param ed The descriptor of the edge to be added.
  /// @param p The edge property.
  /// @return The edge descriptor of the added edge. If successful, the
  /// id() of the descriptor is populated with the actual edge's id, otherwise
  /// set to std::numeric_limits<size_t>::max().
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& ed, edge_property const& p)
  {
    return base_type::add_edge(ed, p);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the edge with the specified source and target, with a
  /// default property to the graph.
  /// @param source The vertex descriptor of the source of the edge to be added.
  /// @param target The vertex descriptor of the target of the edge to be added.
  /// @return The edge descriptor of the added edge. If successful, the
  /// id() of the descriptor is populated with the actual edge's id, otherwise
  /// set to std::numeric_limits<size_t>::max().
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(vertex_descriptor const& source,
                           vertex_descriptor const& target)
  {
    edge_descriptor ed(source, target);
    return base_type::add_edge(ed, edge_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the edge with the specified source and target, with the
  /// given property to the graph.
  /// @param source The vertex descriptor of the source of the edge to be added.
  /// @param target The vertex descriptor of the target of the edge to be added.
  /// @param p The edge property.
  /// @return The edge descriptor of the added edge. If successful, the
  /// id() of the descriptor is populated with the actual edge's id, otherwise
  /// set to std::numeric_limits<size_t>::max().
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(vertex_descriptor const& source,
                           vertex_descriptor const& target,
                           edge_property const& p)
  {
    edge_descriptor ed(source, target);
    return base_type::add_edge(ed, p);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the specified edge
  /// @return True if the delete was successful, or false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool delete_edge(edge_descriptor const& ed)
  {
    return base_type::delete_edge(ed);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes an edge between the specified source and target vertices.
  /// @return True if the delete was successful, or false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool delete_edge(vertex_descriptor const& source,
                   vertex_descriptor const& target)
  {
    return base_type::delete_edge(edge_descriptor(source, target));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Checks if an edge exists between specified source and destination
  /// vertices.
  //////////////////////////////////////////////////////////////////////
  bool is_edge(vertex_descriptor const& source,
               vertex_descriptor const& target) const
  {
    edge_descriptor ed(source, target);
    const_vertex_iterator vi;
    const_adj_edge_iterator ei;
    return base_type::find_edge(ed, vi, ei);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of adjacent edges (out_degree) of the
  /// specified vertex.
  //////////////////////////////////////////////////////////////////////
  size_t get_degree(vertex_descriptor const& vd) const
  {
    const_vertex_iterator cvi = this->find_vertex(vd);
    if (cvi != this->end())
      return (*cvi).size();
    else
      return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the list and number of adjacents of the specified vertex,
  /// @param vd The specified vertex.
  /// @param adj The output vector of the descriptors of the adjacent vertices.
  /// @return The number of adjacent edges (out_degree) of the specified vertex.
  //////////////////////////////////////////////////////////////////////
  size_t get_adjacent_vertices(vertex_descriptor const& vd,
                               std::vector<vertex_descriptor>& adj) const
  {
    adj.clear();
    const_vertex_iterator cvi = this->find_vertex(vd);
    if (cvi != this->end()) {
      const_adj_edge_iterator cei = (*cvi).begin();
      while (cei != (*cvi).end()) {
        adj.push_back((*cei).target());
        ++cei;
      }
      return adj.size();
    } else
      return 0;
  }

}; // graph class

} // namespace sequential

} // namespace stapl

#endif
