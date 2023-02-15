/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_ITERATOR_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_ITERATOR_HPP

#include "graph_util.h"

#include <assert.h>
#include <boost/iterator/iterator_adaptor.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_same.hpp>

//////////////////////////////////////////////////////////////////////
/// @file graph_iterator.h
/// This file contains the iterator implementations for vertex
/// iterator, adj_edge_iterator and edge_iterator. While these can be
/// classes by themselves we implement them as boost iterator adaptors
//////////////////////////////////////////////////////////////////////

namespace stapl {

namespace sequential {

//////////////////////////////////////////////////////////////////////
/// @brief Helper to check if the vertex/edge has a property.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <class Property>
void test_if_property_available(
    typename boost::disable_if<boost::is_same<Property,
                               properties::no_property> >::type* = 0)
{
}

//////////////////////////////////////////////////////////////////////
/// @brief Helper to infer the return property (const or non const)
/// depending on the constness of the edge iterator.
/// @ingroup graphBaseUtil
/// @todo Remove leading underscore from the name to conform to coding
/// standards.
//////////////////////////////////////////////////////////////////////
template <typename Reference>
struct _hei
{
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper to infer the return property (non const)
/// if edge iterator is non const.
/// @ingroup graphBaseUtil
/// @todo Remove leading underscore from the name to conform to coding
/// standards.
//////////////////////////////////////////////////////////////////////
template <typename Value>
struct _hei<Value&>
{
  typedef typename Value::property_type type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper to infer the return property (const)
/// if edge iterator is const.
/// @ingroup graphBaseUtil
/// @todo Remove leading underscore from the name to conform to coding
/// standards.
//////////////////////////////////////////////////////////////////////
template <typename Value>
struct _hei<Value const&>
{
  typedef const typename Value::property_type type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper to infer the return property (const or non const)
/// depending on the constness of the edge iterator.
/// @ingroup graphBaseUtil
/// @todo Remove leading underscore from the name to conform to coding
/// standards.
//////////////////////////////////////////////////////////////////////
template <typename Reference>
struct _hei_it
{
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper to infer the return property (non const)
/// if edge iterator is non const.
/// @ingroup graphBaseUtil
/// @todo Remove leading underscore from the name to conform to coding
/// standards.
//////////////////////////////////////////////////////////////////////
template <typename Value>
struct _hei_it<Value&>
{
  typedef typename Value::edgelist_type edgelist_type;
  typedef typename edgelist_type::iterator iterator;
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper to infer the return property (const)
/// if edge iterator is const.
/// @ingroup graphBaseUtil
/// @todo Remove leading underscore from the name to conform to coding
/// standards.
//////////////////////////////////////////////////////////////////////
template <typename Value>
struct _hei_it<Value const&>
{
  typedef typename Value::edgelist_type edgelist_type;
  typedef typename edgelist_type::const_iterator iterator;
};


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper to provide an edge view for the adjacents of a vertex.
/// @ingroup graphBaseUtil
/// @tparam C The edgelist container type.
/// @tparam Iter The type of the adjacent edge iterator.
//////////////////////////////////////////////////////////////////////
template <class C, class Iter>
class ve_view
{
  C& m_cref;
public:
  typedef typename Iter::value_type reference;
  typedef typename Iter::value_type value_type;
  typedef Iter iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an edge view around the specified edgelist.
  //////////////////////////////////////////////////////////////////////
  ve_view(C& cref)
      : m_cref(cref)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the beginning of the edgelist.
  //////////////////////////////////////////////////////////////////////
  iterator begin() const
  {
    return iterator(m_cref.begin());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the end of the edgelist.
  //////////////////////////////////////////////////////////////////////
  iterator end() const
  {
    return iterator(m_cref.end());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the size of the edgelist.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    return m_cref.size();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Class to provide a reference to a vertex object.
/// @ingroup graphBaseUtil
///
/// Provides the interface of a graph vertex.
/// This is the type returned when a vertex iterator is dereferenced.
/// @tparam BaseIterator The type of the vertex iterator.
//////////////////////////////////////////////////////////////////////
template <class BaseIterator>
class vertex_reference
{
  typedef typename std::iterator_traits<BaseIterator>::value_type
    internal_value_type;
  typedef typename std::iterator_traits<BaseIterator>::reference reference;
  typedef typename internal_value_type::edgelist_type edgelist_type;
protected:
  BaseIterator m_ref;
public:
  typedef typename internal_value_type::vertex_descriptor vertex_descriptor;
  typedef typename internal_value_type::property_type property_type;
  typedef typename _hei<reference>::type& property_reference;
  //next the edge iterator is const or not const depending on the constness of
  //the vertex reference.
  typedef typename _hei_it<reference>::iterator adj_edge_iterator;
  typedef ve_view<edgelist_type, adj_edge_iterator> adj_edges_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a vertex reference based on the provided iterator.
  //////////////////////////////////////////////////////////////////////
  vertex_reference(BaseIterator ref)
      : m_ref(ref)
  {
  }

  vertex_reference(vertex_reference const& other)
      : m_ref(other.m_ref)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the descriptor of the vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor descriptor() const
  {
    return this->m_ref->descriptor();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the property of the vertex.
  //////////////////////////////////////////////////////////////////////
  property_reference property()
  {
    return this->m_ref->property();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the beginning of the vertex's edgelist.
  //////////////////////////////////////////////////////////////////////
  adj_edge_iterator begin()
  {
    return this->m_ref->edgelist().begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the end of the vertex's edgelist.
  //////////////////////////////////////////////////////////////////////
  adj_edge_iterator end()
  {
    return this->m_ref->edgelist().end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the size of the vertex's edgelist.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return this->m_ref->edgelist().size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a view over the vertex's edgelist.
  //////////////////////////////////////////////////////////////////////
  adj_edges_type edges() const
  {
    return adj_edges_type(this->m_ref->edgelist());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Class to provide a pointer to a vertex object.
/// @ingroup graphBaseUtil
/// @tparam Reference The type of the vertex reference.
//////////////////////////////////////////////////////////////////////
template <class Reference>
class vertex_pointer
{
  Reference m_ref;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Create a vertex pointer based on the provided reference.
  //////////////////////////////////////////////////////////////////////
  vertex_pointer(Reference ref)
      : m_ref(ref)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Arrow operator for the vertex pointer.
  //////////////////////////////////////////////////////////////////////
  Reference* operator->()
  {
    return &m_ref;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Class to provide an adaptor for the vertex iterator.
/// @ingroup graphBaseUtil
/// @tparam BaseIterator The type of the vertex iterator.
//////////////////////////////////////////////////////////////////////
template <typename BaseIterator>
class vertex_iterator_adaptor
  : public boost::iterator_adaptor<vertex_iterator_adaptor<BaseIterator>,
                                   BaseIterator>
{
private:
  typedef boost::iterator_adaptor<vertex_iterator_adaptor<BaseIterator>,
      BaseIterator> base_t;
  typedef typename std::iterator_traits<BaseIterator>::value_type
    internal_value_type;
  typedef typename internal_value_type::edgelist_type edgelist_type;
  typedef vertex_pointer<vertex_reference<BaseIterator> > pointer_type;
public:
  typedef vertex_reference<BaseIterator> value_type;
  typedef typename internal_value_type::vertex_descriptor vertex_descriptor;
  typedef typename internal_value_type::property_type property_type;

  typedef typename edgelist_type::iterator adj_edge_iterator;
  typedef typename edgelist_type::const_iterator const_adj_edge_iterator;
  typedef ve_view<edgelist_type, adj_edge_iterator> adj_edges_type;

  vertex_iterator_adaptor() = default;

  vertex_iterator_adaptor(BaseIterator const& iterator)
      : base_t(iterator)
  {
  }

  pointer_type operator->() const
  {
    return pointer_type(value_type(this->base_reference()));
  }

  value_type operator*() const
  {
    return value_type(this->base_reference());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Class to provide a const adaptor for the vertex iterator.
/// @ingroup graphBaseUtil
/// @tparam BaseIterator The type of the vertex iterator.
//////////////////////////////////////////////////////////////////////
template <typename BaseIterator>
class const_vertex_iterator_adaptor
 : public boost::iterator_adaptor<const_vertex_iterator_adaptor<BaseIterator>,
                                  BaseIterator>
{
private:
  typedef boost::iterator_adaptor<const_vertex_iterator_adaptor<BaseIterator>,
      BaseIterator> base_t;
  typedef typename std::iterator_traits<BaseIterator>::value_type
    internal_value_type;
  typedef typename internal_value_type::edgelist_type edgelist_type;
  typedef vertex_pointer<vertex_reference<BaseIterator> > pointer_type;
public:
  typedef vertex_reference<BaseIterator> value_type;
  typedef typename internal_value_type::vertex_descriptor vertex_descriptor;
  typedef typename internal_value_type::property_type property_type;
  typedef typename edgelist_type::const_iterator adj_edge_iterator;
  typedef typename edgelist_type::const_iterator const_adj_edge_iterator;
  typedef ve_view<edgelist_type, const_adj_edge_iterator> adj_edges_type;

  const_vertex_iterator_adaptor() = default;

  const_vertex_iterator_adaptor(BaseIterator const& iterator)
    : base_t(iterator)
  {
  }

  //iterator and const_iterator interoperability
  template <typename Iter>
  inline const_vertex_iterator_adaptor(
    vertex_iterator_adaptor<Iter> const& other)
    : base_t(other.base())
  {
  }

  pointer_type operator->() const
  {
    return pointer_type(value_type(this->base_reference()));
  }
  value_type operator*() const
  {
    return value_type(this->base_reference());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Selector class for vertex iterator.
/// @ingroup graphBaseUtil
///
/// Other graphs may specialize the iterator type, and in order to do
/// that they have to specialize based on the particular property type.
/// @tparam Iterator The type of the iterator.
/// @tparam Property The type of the vertex property.
//////////////////////////////////////////////////////////////////////
template <class Iterator, class Property>
struct select_vertex_iterator
{
  typedef vertex_iterator_adaptor<Iterator> type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Selector class for const vertex iterator.
/// @ingroup graphBaseUtil
///
/// Other graphs may specialize the iterator type, and in order to do
/// that they have to specialize based on the particular property type.
/// @tparam Iterator The type of the iterator.
/// @tparam Property The type of the vertex property.
//////////////////////////////////////////////////////////////////////
template <class Iterator, class Property>
struct select_const_vertex_iterator
{
  typedef const_vertex_iterator_adaptor<Iterator> type;
};




//////////////////////////////////////////////////////////////////////
/// @brief Class to provide a reference to an edge object.
/// @ingroup graphBaseUtil
///
/// Provides the interface of a graph edge.
/// This is the type returned when a adj/edge iterator is dereferenced.
/// @tparam BaseIterator The type of the edge iterator.
//////////////////////////////////////////////////////////////////////
template <class BaseIterator>
class edge_reference
{
  typedef typename std::iterator_traits<BaseIterator>::value_type
    internal_value_type;
  typedef typename std::iterator_traits<BaseIterator>::reference reference;
  BaseIterator m_ref;
public:
  typedef typename
    std::iterator_traits<BaseIterator>::value_type::edge_descriptor_type
      edge_descriptor;
  typedef typename std::iterator_traits<BaseIterator>::value_type::property_type
    property_type;
  typedef typename _hei<reference>::type& property_reference;
  typedef typename edge_descriptor::vertex_descriptor vertex_descriptor;
  typedef typename edge_descriptor::edge_id_type edge_id_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an edge reference based on the provided iterator.
  //////////////////////////////////////////////////////////////////////
  edge_reference(BaseIterator ref)
      : m_ref(ref)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the descriptor of the edge.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor descriptor() const
  {
    return this->m_ref->descriptor();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the vertex descriptor of the source vertex of the edge.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor source() const
  {
    return this->m_ref->descriptor().source();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the vertex descriptor of the target vertex of the edge.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor target() const
  {
    return this->m_ref->descriptor().target();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the ID of the edge.
  //////////////////////////////////////////////////////////////////////
  edge_id_type id() const
  {
    return this->m_ref->descriptor().id();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the property of the edge.
  //////////////////////////////////////////////////////////////////////
  property_reference property() const
  {
    return this->m_ref->property();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Class to provide an adaptor for the adjacent edge iterator.
/// @ingroup graphBaseUtil
/// @tparam BaseIterator The type of the iterator over the edge container
/// in the vertex (eg: std::vector<Edges>::iterator).
//////////////////////////////////////////////////////////////////////
template <typename BaseIterator>
class adj_edge_iterator_adaptor
  : public boost::iterator_adaptor<adj_edge_iterator_adaptor<BaseIterator>,
                                   BaseIterator>
{
private:
  typedef boost::iterator_adaptor<adj_edge_iterator_adaptor<BaseIterator>,
      BaseIterator> base_t;
  typedef typename std::iterator_traits<BaseIterator>::reference ref_prop_type;

public:
  typedef typename
    std::iterator_traits<BaseIterator>::value_type::edge_descriptor_type
      edge_descriptor;
  typedef typename std::iterator_traits<BaseIterator>::value_type::property_type
    property_type;
  typedef typename _hei<ref_prop_type>::type property_reference;
  typedef typename edge_descriptor::vertex_descriptor vertex_descriptor;
  typedef edge_reference<BaseIterator> value_type;
  typedef edge_reference<BaseIterator> reference_type;

  adj_edge_iterator_adaptor() = default;

  adj_edge_iterator_adaptor(BaseIterator iterator)
      : base_t(iterator)
  {
  }

  //iterator and const_iterator interoperability
  template <typename Iter>
  inline adj_edge_iterator_adaptor(
      adj_edge_iterator_adaptor<Iter> const& other)
      : base_t(other.base())
  {
  }

  value_type operator*() const
  {
    return value_type(this->base_reference());
  }

  BaseIterator& operator->()
  {
    return this->base_reference();
  }

#ifdef _STAPL
  void define_type(typer&)
  {
    abort("adj_edge_iterator_adaptor: Incorrect define_type()");
  }
#endif
};


//////////////////////////////////////////////////////////////////////
/// @brief Selector class for adjacent edge iterator.
/// @ingroup graphBaseUtil
/// @tparam Iterator The type of the iterator.
/// @tparam Edge The type of the edge.
//////////////////////////////////////////////////////////////////////
template <class Iterator, class Edge>
struct select_adj_edge_iterator
{
  typedef adj_edge_iterator_adaptor<Iterator> type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Selector class for const adjacent edge iterator.
/// @ingroup graphBaseUtil
/// @tparam Iterator The type of the iterator.
/// @tparam Edge The type of the edge.
//////////////////////////////////////////////////////////////////////
template <class Iterator, class Edge>
struct select_const_adj_edge_iterator
{
  typedef adj_edge_iterator_adaptor<Iterator> type;
};



//////////////////////////////////////////////////////////////////////
/// @brief Class to provide an adaptor for the edge iterator.
/// @ingroup graphBaseUtil
///
/// An edge iterator allows traversal of all (out)edges in the graph.
/// It does this by iterating over all adjacent edges of one vertex, moves to
/// the next vertex and so on. There is no guaranteed order of edge traversal.
/// In undirected graphs only one edge of a pair of sibling edges will be
/// traversed.
/// @tparam VertexIterator The type of the vertex iterator.
//////////////////////////////////////////////////////////////////////
template <typename VertexIterator>
class edge_iterator_adaptor
  : public boost::iterator_adaptor<edge_iterator_adaptor<VertexIterator>,
                                   typename VertexIterator::adj_edge_iterator>
{
private:
  typedef edge_iterator_adaptor<VertexIterator> this_type;
  typedef boost::iterator_adaptor<edge_iterator_adaptor<VertexIterator>,
      typename VertexIterator::adj_edge_iterator> base_t;
  //the native edge iterator type
  typedef typename VertexIterator::adj_edge_iterator n_edge_iterator_type;

  VertexIterator m_begin;
  VertexIterator m_current;
  VertexIterator m_end;
  bool m_is_directed;

public:
  typedef typename n_edge_iterator_type::edge_descriptor edge_descriptor;
  typedef typename n_edge_iterator_type::property_type property_type;
  typedef typename n_edge_iterator_type::property_reference property_reference;
  typedef typename edge_descriptor::vertex_descriptor vertex_descriptor;

  typedef typename base_t::value_type value_type;

  edge_iterator_adaptor() = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used by the UG class to signal the fact that the
  /// iterator needs to skip sibling edges. E.g., if there is an edge
  /// from 1->2 the edge iterator will not report the edge from 2->1.
  //////////////////////////////////////////////////////////////////////
  edge_iterator_adaptor(edge_iterator_adaptor const& other, bool is_directed)
    : base_t(other.base()), m_begin(other.m_begin),
      m_current(other.m_current), m_end(other.m_end),
      m_is_directed(is_directed)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an edge iterator based on the specified iterators.
  /// @param iterator The native edge iterator.
  /// @param b A vertex iterator to the beginning of the graph.
  /// @param c A vertex iterator to the current vertex whose edges are being
  /// currently traversed.
  /// @param e A vertex iterator to the end of the graph.
  /// @param is_directed True for directed graph, false for undirected graph.
  //////////////////////////////////////////////////////////////////////
  edge_iterator_adaptor(n_edge_iterator_type iterator, VertexIterator b,
                        VertexIterator c, VertexIterator e,
                        bool is_directed = true)
    : base_t(iterator), m_begin(b), m_current(c), m_end(e),
      m_is_directed(is_directed)
  {
  }

  //iterator and const_iterator interoperability
  template <class Other>
  edge_iterator_adaptor(edge_iterator_adaptor<Other> const& other)
    : base_t(other.base()), m_begin(other.m_begin),
      m_current(other.m_current), m_end(other.m_end),
      m_is_directed(other.m_is_directed)
  {
  }

private:
  //for const/non const interoperability
  template <class > friend class edge_iterator_adaptor;

  //////////////////////////////////////////////////////////////////////
  /// @brief Increments the iterator, skipping over vertices with no edges.
  /// @todo Rename to remove leading underscore.
  //////////////////////////////////////////////////////////////////////
  void _increment()
  {
    ++(this->base_reference());
    if (this->base_reference() == (*(this->m_current)).end()) {
      // move to the next vertex; if its edgelist is empty keep skipping
      ++(this->m_current);
      while (this->m_current != this->m_end && (*(this->m_current)).size() == 0)
        ++(this->m_current);
      //check if this is the last vertex
      if (m_current != m_end)
        this->base_reference() = (*(this->m_current)).begin();
      else {
        m_current = --(this->m_end);
        //return global end; which is the end of the last vertex adjacency list
        this->base_reference() = (*m_current).end();
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Decrements the iterator, skipping over vertices with no edges.
  /// @todo Rename to remove leading underscore.
  //////////////////////////////////////////////////////////////////////
  void _decrement()
  {
    if (this->base_reference() != (*(this->m_current)).begin()) {
      --(this->base_reference());
    } else {
      //move to the prev vertex if there is one
      if (m_current == m_begin)
        return; //we reached global begin
      --(this->m_current);
      //next skip vertices with no edges
      while (m_current != m_begin && (*m_current).size() == 0)
        --(this->m_current);
      if ((*m_current).size() != 0) {
        this->base_reference() = (*(this->m_current)).end() - 1;
      } else {
        //std::cout<<"edge_iterator out of bounds while decrementing\n";
        //assert(false);
        this->base_reference() = (*(this->m_current)).begin();
        return;
      }
    }
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Increments the iterator, skipping over vertices with no edges,
  /// and skipping over siblings-edges (where target < source) for
  /// undirected graphs.
  //////////////////////////////////////////////////////////////////////
  void increment()
  {
    _increment();
    if (m_is_directed)
      return;
    else {
      if (this->m_begin == this->m_end)
        return;
      VertexIterator m_end_1 = this->m_end;
      --m_end_1;
      while (this->base_reference() != (*m_end_1).end()
          && (*(this->base_reference())).source()
              > (*(this->base_reference())).target())
        _increment();
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Increments the iterator, skipping over vertices with no edges,
  /// and skipping over siblings-edges (where target < source) for
  /// undirected graphs.
  //////////////////////////////////////////////////////////////////////
  void decrement()
  {
    _decrement();
    if (m_is_directed)
      return;
    else {
      if (this->m_begin == this->m_end)
        return;
      while (this->base_reference() != (*(this->m_begin)).begin()
          && (*(this->base_reference())).source()
              > (*(this->base_reference())).target())
        _decrement();
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Advances the iterator by the specified number of steps.
  /// Used to provide random-access behavior.
  //////////////////////////////////////////////////////////////////////
  void advance(typename base_t::difference_type n)
  {
    if (n >= 0) {
      while (n-- > 0) {
        this->increment();
      }
    } else {
      while (n++ > 0) {
        this->decrement();
      }
    }
  }

  value_type operator*() const
  {
    return this->base_reference().operator*();
  }

  n_edge_iterator_type operator->()
  {
    return this->base_reference();
  }

#ifdef _STAPL
  bool equal(this_type const& rhs) const
  {
    return this->m_current == rhs.m_current &&
    this->base() == rhs.base();
  }

  friend edge_descriptor gid_of(this_type const& ei)
  { return (*ei).descriptor(); }

  void define_type(typer& t)
  {
    t.member(m_begin);
    t.member(m_current);
    t.member(m_end);
    t.member(m_is_directed);
  }
#endif

}; // class edge_iterator_adaptor

} // namespace sequential
} // namespace stapl

#endif
