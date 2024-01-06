/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_ADJACENCY_LIST_CORE_GRAPH_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_ADJACENCY_LIST_CORE_GRAPH_HPP

#include "runtime/stapl_assert.hpp"
#include "adj_list_vertex_edge.h"
#include "graph_iterator.h"

namespace stapl {

namespace sequential {

//////////////////////////////////////////////////////////////////////
/// @brief Adjacency list implementation of the sequential graph.
/// @tparam Traits Traits class which specifies types of descriptors, storage,
///   etc.
/// @ingroup graphBase
///
/// This is a base class and it implements the following graph concepts:
/// adjacency_graph, static_graph, dynamic_graph. Other graph concepts, such
/// as directed, undirected, multiedges, and non-multiedges are implemented as
/// derived classes.
//////////////////////////////////////////////////////////////////////
template <class Traits>
class adjacency_list_graph
{
  typedef adjacency_list_graph<Traits> this_type;
 public:
  typedef typename Traits::vertex_descriptor         vertex_descriptor;
  typedef typename Traits::simple_vertex_descriptor  simple_vertex_descriptor;
  typedef typename Traits::edge_descriptor           edge_descriptor;
  typedef typename Traits::vertex_property           vertex_property;
  typedef typename Traits::edge_property             edge_property;

 protected:
  typedef typename Traits::edge_type                 edge_type;
  typedef adjacency_descriptor_impl<edge_type>       edgelist_type;
  typedef typename select_vertex<vertex_descriptor,
    vertex_property, edgelist_type>::type            vertex_impl_type;

  /// Vertex storage type.
  typedef typename Traits::storage_type              vertex_set_type;

  /// Set of vertices.
  vertex_set_type m_vertices;

  /// Number of edges.
  size_t m_ne;

  /// Used to generate unique edge ids.
  size_t m_eid;

 public:
  typedef typename select_vertex_iterator<typename vertex_set_type::iterator,
      vertex_property>::type                           vertex_iterator;
  typedef typename
      select_const_vertex_iterator<typename vertex_set_type::const_iterator,
      vertex_property>::type                           const_vertex_iterator;
  typedef vertex_iterator                              iterator;
  typedef const_vertex_iterator                        const_iterator;

  /// Iterator over adjacent edges of a vertex.
  typedef typename edgelist_type::iterator             adj_edge_iterator;
  /// Const iterator over adjacent edges of a vertex.
  typedef typename edgelist_type::const_iterator       const_adj_edge_iterator;

  /// Edge iterator over all edges in the graph.
  typedef edge_iterator_adaptor<vertex_iterator>       edge_iterator;
  /// Const edge iterator over all edges in the graph.
  typedef edge_iterator_adaptor<const_vertex_iterator> const_edge_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @param size Number of vertices in the new graph.
  //////////////////////////////////////////////////////////////////////
  adjacency_list_graph(size_t size=0)
    : m_vertices(size), m_ne(0), m_eid(0)
  { }

  vertex_iterator begin()
  { return vertex_iterator(this->m_vertices.begin()); }

  const_vertex_iterator begin() const
  { return const_vertex_iterator(this->m_vertices.begin()); }

  vertex_iterator end()
  { return vertex_iterator(this->m_vertices.end());}

  const_vertex_iterator end() const
  { return const_vertex_iterator(this->m_vertices.end()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an edge iterator to the first edge in the entire
  /// adjacency-list.
  //////////////////////////////////////////////////////////////////////
  edge_iterator edges_begin()
  {
    //find first vertex with edges
    typename vertex_set_type::iterator it = this->m_vertices.begin();
    while (it != this->m_vertices.end() && it->edgelist().size() == 0)
      ++it;
    //check if its not the end
    if (it == this->m_vertices.end()) {
      return edges_end();
    } else {
      return edge_iterator(it->edgelist().begin(),
                         this->m_vertices.begin(),
                         it,
                         this->m_vertices.end()
                         );
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an edge iterator to the first edge in the adjacency-list.
  //////////////////////////////////////////////////////////////////////
  const_edge_iterator edges_begin() const
  {
    //find first vertex with edges
    typename vertex_set_type::const_iterator it = this->m_vertices.begin();
    while (it != this->m_vertices.end() && it->edgelist().size() == 0)
      ++it;
    //check if its not the end
    if (it == this->m_vertices.end()) {
      return edges_end();
    } else {
      return const_edge_iterator(it->edgelist().begin(),
                         this->m_vertices.begin(),
                         it,
                         this->m_vertices.end()
                         );
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an edge iterator to one past the last edge in the
  /// adjacency-list.
  //////////////////////////////////////////////////////////////////////
  edge_iterator edges_end()
  {
    typename vertex_set_type::iterator it = this->m_vertices.end();
    if (this->m_vertices.size() == 0) {
      return edge_iterator(adj_edge_iterator(), it, it, it);
    } else {
      --it;
      return edge_iterator((it)->edgelist().end(),
                           this->m_vertices.begin(),
                           it,
                           this->m_vertices.end() );
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an edge iterator to the last edge in the adjacency-list.
  //////////////////////////////////////////////////////////////////////
  const_edge_iterator edges_end() const
  {
    typename vertex_set_type::const_iterator it = this->m_vertices.end();
    if (this->m_vertices.size() == 0) {
      return const_edge_iterator(const_adj_edge_iterator(), it, it, it);
    } else {
      --it;
      return const_edge_iterator((it)->edgelist().end(),
                                 this->m_vertices.begin(),
                                 it,
                                 this->m_vertices.end() );
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the total number of vertices.
  //////////////////////////////////////////////////////////////////////
  size_t get_num_vertices() const
  {
    return this->m_vertices.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the largest vertex id.
  //////////////////////////////////////////////////////////////////////
  size_t get_max_descriptor() const
  {
    return this->m_vertices.get_max_descriptor();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the total number of edges.
  //////////////////////////////////////////////////////////////////////
  size_t get_num_edges() const
  {
    return m_ne;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds vertex with the default vertex property.
  ///  A descriptor is automatically generated for the new vertex.
  /// @return The descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex()
  {
    return add_vertex(vertex_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds vertex with the specified vertex property.
  /// A descriptor is automatically generated for the new vertex.
  /// @param vp The vertex property of the added vertex.
  /// @return The descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(const vertex_property& vp)
  {
    return this->m_vertices.add_vertex(vp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds vertex to the storage with the given descriptor and property.
  /// Useful, for example, when reading the graph from the file or when the user
  /// wants to explicitly control the descriptors.
  /// @param vd The explicit descriptor of the added vertex.
  /// @param vp The vertex property of the added vertex.
  /// @return The descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_descriptor vd, const vertex_property& vp)
  {
    return this->m_vertices.add_vertex(vd, vp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge with given descriptor and default edge property.
  /// @param ed Descriptor of the desired edge.
  /// @return edge_descriptor of the added edge.
  /// @ref edge_descriptor.id() is set to numeric_limits<size_t>::max() if edge
  /// was not added.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(const edge_descriptor& ed)
  {
    return add_edge(ed,edge_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge with given descriptor and edge property.
  /// @param ed Descriptor of the desired edge.
  /// @param p Property of the edge.
  /// @return edge_descriptor of the added edge.
  /// @ref edge_descriptor.id() is set to numeric_limits<size_t>::max() if edge
  /// was not added.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(const edge_descriptor& ed,const edge_property& p)
  {
    // don't use reference for the two lines below.
    // this is done for versioning, to prevent adding edges
    // between invalid iterators from another graph.
    simple_vertex_descriptor source = ed.source();
    simple_vertex_descriptor target = ed.target();
    // source.version(-1);
    // target.version(-1);

    vertex_iterator vi = this->find_vertex(source);
    if (vi != this->end()) {
      size_t id = m_eid++;
      edge_descriptor ned(source, target, id);
      // edgelist is a method of the vertex.
      vi.base()->edgelist().add(edge_type(ned,p));
      ++m_ne;
      return ned;
    } else {
      return edge_descriptor(vertex_descriptor(INVALID_VALUE),
                             ed.target(),
                             INVALID_VALUE);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets i-th neighbor of v to the specified neighbor vertex.
  /// @param v Descriptor of the source vertex.
  /// @param i The index of the target neighbor vertex.
  /// @param neighbor The descriptor of the target neighbor vertex.
  /// @return True if the vertex and edge exist, False otherwise.
  //////////////////////////////////////////////////////////////////////
  bool set_edge(const vertex_descriptor& v, const size_t i,
                const vertex_descriptor& neighbor)
  {
    vertex_iterator vi = find_vertex(v);
    if (vi == this->end() || (*vi).size() <= i)
      return false;
    (vi.base()->edgelist()).m_data[i] = edge_type(edge_descriptor(v, neighbor));
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets i-th neighbor of v to the specified neighbor vertex.
  /// Expands the edgelist to the specified max, if current size is smaller.
  /// @param v Descriptor of the source vertex.
  /// @param i The index of the target neighbor vertex.
  /// @param max The maximum size to expand the edgelist to.
  /// @param neighbor The descriptor of the target neighbor vertex.
  /// @return True if the vertex exists, False otherwise.
  //////////////////////////////////////////////////////////////////////
  bool set_edge(const vertex_descriptor& v, const size_t i, const size_t max,
                const vertex_descriptor& neighbor)
  {
    vertex_iterator vi = find_vertex(v);
    if (vi == this->end()) {
      return false;
    }
    for (int j = (*vi).size(); j < max; ++j) {
      //edgelist() is a method of the vertex
      vi.base()->edgelist().add(
          edge_type(edge_descriptor(INVALID_VALUE, INVALID_VALUE)));
    }
    (vi.base()->edgelist()).m_data[i] = edge_type(edge_descriptor(v, neighbor));
    return true;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Erases all vertices and edges in the graph.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->m_ne  = 0;
    this->m_eid = 0;
    this->m_vertices.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes all edges with the specified vertex as their target.
  /// For internal use by @ref delete_vertex.
  /// @param vd The descriptor of the vertex.
  /// @todo This should be a protected member method of this class.
  //////////////////////////////////////////////////////////////////////
  void delete_all_edges_to_v(const vertex_descriptor& vd)
  {
    //for all vertices check if they have edges pointing to vd
    for (vertex_iterator vi = this->begin(); vi != this->end(); ++vi) {
      adj_edge_iterator ei = graph_find((*vi).begin(), (*vi).end(),
                                        eq_target<vertex_descriptor>(vd));;
      while (ei != (*vi).end()) {
        //base() can only be called in this class.
        vi.base()->edgelist().erase(ei);
        this->m_ne--;
        ei = graph_find((*vi).begin(), (*vi).end(),
                        eq_target<vertex_descriptor>(vd));
      }
    }
  }

  public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the specified vertex and all edges pointing to and from it.
  /// @param vd The descriptor of the vertex.
  /// @return Whether or not the vertex was successfully deleted.
  //////////////////////////////////////////////////////////////////////
  bool delete_vertex(const vertex_descriptor& vd)
  {
    delete_all_edges_to_v(vd);
    vertex_iterator vi = find_vertex(vd);
    if (vi != this->end()) {
      this->m_ne -= (*vi).size();
      vi.base()->clear();//free the edge list
      this->m_vertices.delete_vertex(vd, vi.base());
      return true;
    }
    else
      return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the edge with given descriptor.
  /// @param ed Descriptor of the desired edge.
  /// @return Whether or not the edge was successfully deleted.
  //////////////////////////////////////////////////////////////////////
  bool delete_edge(const edge_descriptor& ed)
  {
    vertex_iterator vi;
    adj_edge_iterator ei;
    find_edge(ed,vi,ei);
    if (vi != this->end()) {
      if (ei != (*vi).end()) {
        vi.base()->edgelist().erase(ei);
        this->m_ne--;
        return true;
      }
      else
        return false;
    }
    else
      return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds the edge with the specified descriptor, and sets an
  /// iterator to its source vertex and an adj_edge_iterator pointing to the
  /// edge.
  /// @param ed Descriptor of the edge.
  /// @param vi vertex_iterator pointing to the source vertex of the edge,
  /// populated by the method.
  /// @param aei adj_edge_iterator pointing to the specified edge,
  /// populated by the method.
  /// @return Whether or not the edge was found.
  /// @todo Return a pair<bool, pair<vertex_iterator, adj_edge_iterator> >
  /// instead of passing them in as parameters.
  //////////////////////////////////////////////////////////////////////
  bool find_edge(const edge_descriptor& ed, vertex_iterator& vi,
                 adj_edge_iterator& ei)
  {
    vi = find_vertex(ed.source());
    if (vi != this->end()) {
      // if the edge id is missing we look for target, source
      // and delete the first encountered
      if (ed.id() == (size_t)INVALID_VALUE)
        ei = graph_find((*vi).begin(), (*vi).end(),
                        eq_target<vertex_descriptor>(ed.target()));
      else {
        //if edge descriptor is complete, edge-id uniquely identifies an edge
        //!!!the next check is too strong for directed; it can be relaxed by
        //specializing for directed and undirected
        ei = graph_find((*vi).begin(), (*vi).end(),
                        eq_ed<edge_descriptor>(ed));
        //ei = graph_find(vi.begin(), vi.end(),
        //                eq_eid<vertex_descriptor>(ed.id()));
      }
      if (ei != (*vi).end()) {
        return true;
      } else
        return false;
    } else
      return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds the edge with the specified descriptor, and sets an
  ///   iterator to its source vertex and an adj_edge_iterator pointing to the
  ///   edge.
  /// @param ed Descriptor of the edge.
  /// @param vi vertex_iterator pointing to the source vertex of the edge,
  ///   populated by the method.
  /// @param aei adj_edge_iterator pointing to the specified edge,
  ///   populated by the method.
  /// @return Whether or not the edge was found.
  /// @todo Return a pair<bool, pair<vertex_iterator, adj_edge_iterator> >
  /// instead of passing them in as parameters.
  //////////////////////////////////////////////////////////////////////
  bool find_edge(const edge_descriptor& ed, const_vertex_iterator& vi,
                 const_adj_edge_iterator& ei) const
  {
    vertex_iterator   i_vi;
    adj_edge_iterator i_ei;
    bool res = const_cast<this_type*>(this)->find_edge(ed,i_vi,i_ei);
    vi = const_vertex_iterator(i_vi.base());
    ei = const_adj_edge_iterator(i_ei.base());
    return res;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds the vertex with the specified descriptor, and returns a
  /// vertex_iterator pointing to it. If not found, the end of the graph is
  /// returned.
  /// @param vd Descriptor of the vertex.
  /// @return A vertex_iterator to the specified vertex, if found, or a
  /// vertex_iterator to the end of the graph otherwise.
  //////////////////////////////////////////////////////////////////////
  const_vertex_iterator find_vertex(vertex_descriptor const& vd) const
  {
    vertex_iterator vi = const_cast<this_type*>(this)->find_vertex(vd);
    return const_vertex_iterator(vi.base());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds the vertex with the specified descriptor, and returns a
  /// vertex_iterator pointing to it. If not found, the end of the graph is
  /// returned.
  /// @param vd Descriptor of the vertex.
  /// @return A vertex_iterator to the specified vertex, if found, or a
  /// vertex_iterator to the end of the graph otherwise.
  //////////////////////////////////////////////////////////////////////
  vertex_iterator find_vertex(vertex_descriptor const& vd)
  {
    return vertex_iterator(this->m_vertices.find_vertex(vd));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Updates given descriptor's version info, provided the iterator
  /// is valid. If iterator is not valid, it is updated with a call to find().
  /// @param vd Descriptor of the edge.
  /// @param vi A vertex_iterator pointing to the specified vertex.
  //////////////////////////////////////////////////////////////////////
  void update_descriptor(vertex_descriptor& vd, vertex_iterator& vi)
  {
    this->m_vertices.update_descriptor(vd, vi.base());
  }

#ifdef _STAPL
  void define_type(stapl::typer& t)
  {
    t.member(m_vertices);
    t.member(m_ne);
  }
#endif

 private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the specified edge to the vertex pointed to by the provided
  /// iterator.
  /// @param vi Iterator pointing to the vertex to which the edge will be added.
  /// @param edge The edge to add.
  //////////////////////////////////////////////////////////////////////
  void add_internal_edge(vertex_iterator vi, const edge_type& edge)
  {
    vi.base()->edgelist().add(edge);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge with the given edge_descriptor.
  /// The edge_descriptor provided must have the id field assigned prior to
  /// the call to this method. This is used by the undirected graph
  /// to add sibling edges which share the same edge id.
  /// @param g The graph to which the edge will be added.
  /// @param ed The descriptor of the edge to be added. Id of the descriptor
  /// must have been externally assigned.
  /// @param p The property of the edge being added.
  //////////////////////////////////////////////////////////////////////
  friend void add_internal_edge(this_type& g, const edge_descriptor& ed,
                                const edge_property& p)
  {
    vertex_iterator vi = g.find_vertex(ed.source());
    stapl_assert(vi != g.end(), "Failed to add internal edge.");

    edge_type edge(ed,p);
    g.add_internal_edge(vi,edge);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds two edges: one from source to target of the specified
  /// descriptor and the other from the target to source. Adds first to
  /// the vertex with the smaller id of source() and target().
  /// This will generate the id of the edge and this id will be the same
  /// in the second (sibling) edge added. This is important to match the
  /// edges when there are duplicates.
  /// Uses @ref add_internal_edge to add the sibling edge.
  /// @param g The graph to which the edge will be added.
  /// @param edr The descriptor of the edge to be added. Id of the descriptor
  /// must have been externally assigned.
  /// @param epy The property of the edge being added.
  //////////////////////////////////////////////////////////////////////
  friend edge_descriptor add_edge_pair(this_type& g,
                                       const edge_descriptor& edr,
                                       const edge_property& epy)
  {
    edge_descriptor n_edr;
    bool reversed = false;
    if (edr.source() > edr.target()) {
      n_edr = reverse(edr);
      reversed = true;
    } else
      n_edr = edr;
    // this is done for versioning, to prevent adding edges
    // between invalid iterators from another graph.
    simple_vertex_descriptor n_edr_source = n_edr.source();
    simple_vertex_descriptor n_edr_target = n_edr.target();
    // n_edr_target.version(-1);
    // n_edr_source.version(-1);

    vertex_iterator vi1 = g.find_vertex(n_edr_source);
    vertex_iterator vi2 = g.find_vertex(n_edr_target);

    vertex_descriptor v1, v2;
    if (vi1 == g.end())
      v1 = vertex_descriptor(INVALID_VALUE);
    else
      v1 = edr.source();
    if (vi2 == g.end())
      v2 = vertex_descriptor(INVALID_VALUE);
    else
      v2 = edr.target();
    if (vi1 != g.end() && vi2 != g.end()){
      size_t eid = g.m_eid++;
      edge_descriptor ied(n_edr_source, n_edr_target, eid);
      edge_type edge(ied,epy);
      // it has to be like this
      g.add_internal_edge(vi1, edge);
      //because sister edges have to share same property
      g.add_internal_edge(vi2,reverse(edge) );
      g.m_ne += 2;
      return ied;
    }
    //here error encountered
    if (!reversed)
      return edge_descriptor(v1, v2, INVALID_VALUE);
    else
      return edge_descriptor(v2, v1, INVALID_VALUE);
  }

}; // class adjacency_list_graph

} } //namespace stapl::sequential

#endif
