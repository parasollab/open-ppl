/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_STORAGE_HPP
#define STAPL_CONTAINERS_GRAPH_STORAGE_HPP

#include <stapl/containers/graph/local_accessor_graph.hpp>
#include <stapl/containers/graph/functional.hpp>
#include <stapl/containers/sequential/graph/adj_list_vertex_edge.h>
#include <stapl/containers/sequential/graph/graph_iterator.h>
#include <stapl/containers/sequential/graph/graph_util.h>
#include <stapl/containers/sequential/graph/undirected_util.h>
#include <stapl/containers/sequential/graph/vdg_intVD.h>
#include <stapl/views/iterator/member_iterator.h>
#include <boost/unordered_map.hpp>
#include <boost/iterator/transform_iterator.hpp>
#ifdef _STAPL
# include <stapl/runtime/serialization.hpp>
#endif

#include <ostream>

namespace stapl {
//to be used in friend declarations
template<class Traits> class adjacency_list_model;

//////////////////////////////////////////////////////////////////////
/// @brief Vertex for adjacency list for the graph.
/// @ingroup pgraphAdjacencyVertex
/// @tparam VD Vertex descriptor type for the graph.
/// @tparam Property Property type of the vertex.
/// @tparam AdjList Type of the edgelist for storing edges.
//////////////////////////////////////////////////////////////////////
template<class VD, class Property, class AdjList>
class vertex_adj_list_impl
  : public sequential::select_vertex<VD,Property,AdjList>::type
{
  template<class Traits> friend class adjacency_list_model;
  typedef typename sequential::select_vertex
           <VD,Property,AdjList>::type           base_type;
 public:
  typedef VD                                     vertex_descriptor;
  typedef AdjList                                edgelist_type;
  typedef AdjList                                adj_edges_type;
  typedef typename AdjList::edge_descriptor      edge_descriptor;
  typedef typename AdjList::iterator             edgelist_it;
  typedef typename AdjList::const_iterator       const_edgelist_it;
  typedef edgelist_it                            adj_edge_iterator;
  typedef const_edgelist_it                      const_adj_edge_iterator;
  typedef Property                               property_type;
  typedef Property&                              property_reference;
  typedef Property const&                        const_property_reference;

  vertex_adj_list_impl(void)
    : base_type()
  { }

  vertex_adj_list_impl(vertex_adj_list_impl const& other)
    : base_type(other)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a vertex for the graph with the given descriptor
  /// and property.
  /// @param vd The descriptor of this vertex.
  /// @param p The property of this vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_adj_list_impl(vertex_descriptor vd, Property const& p)
    : base_type(vd,p)
  { }


  template <typename Comp>
  void sort_edges(Comp const& comp)
  {
    std::sort(this->m_edgelist.begin(), this->m_edgelist.end(), comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the beginning of the edgelist of this vertex.
  //////////////////////////////////////////////////////////////////////
  edgelist_it begin(void)
  { return this->m_edgelist.begin(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the end of the edgelist of this vertex.
  //////////////////////////////////////////////////////////////////////
  const_edgelist_it begin(void) const
  { return this->m_edgelist.begin(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the beginning of the edgelist of this vertex.
  //////////////////////////////////////////////////////////////////////
  edgelist_it end(void)
  { return this->m_edgelist.end(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the end of the edgelist of this vertex.
  //////////////////////////////////////////////////////////////////////
  const_edgelist_it end(void) const
  { return this->m_edgelist.end(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the edge with the specified target.
  /// @param vd The descriptor of the target vertex.
  //////////////////////////////////////////////////////////////////////
  edgelist_it find_edge(vertex_descriptor const& vd)
  { return this->m_edgelist.find(vd); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the specified edge.
  /// @param ed The descriptor of the edge.
  //////////////////////////////////////////////////////////////////////
  edgelist_it find_edge(edge_descriptor const& ed)
  { return this->m_edgelist.find(ed); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of outgoing edges of this vertex.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  { return this->m_edgelist.size(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reserves space for specified number of edges.
  //////////////////////////////////////////////////////////////////////
  void reserve(size_t n)
  { this->m_edgelist.reserve(n); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erases all edges from index n until the end.
  /// @param n The starting index of the range of edges to be deleted.
  //////////////////////////////////////////////////////////////////////
  void erase(size_t n)
  { this->m_edgelist.erase(n); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc adjacency_list_impl::erase_at(size_t)
  //////////////////////////////////////////////////////////////////////
  void erase_at(size_t n)
  { this->m_edgelist.erase_at(n); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc adjacency_list_impl::erase_at(size_t, size_t)
  //////////////////////////////////////////////////////////////////////
  void erase(size_t start, size_t end)
  { this->m_edgelist.erase(start, end); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc adjacency_list_impl::erase_at(Iter)
  //////////////////////////////////////////////////////////////////////
  template <typename Iter>
  void erase(Iter it)
  { this->m_edgelist.erase(it); }

#ifdef _STAPL
  void define_type(typer& t)
  { t.base<base_type>(*this); }

  size_t memory_size(void) const
  { return base_type::memory_size(); }
#endif
};

namespace graph_storage_impl {
  template <typename Accessor>
  struct is_const_accessor
    : public boost::true_type
  { };

  template <typename View>
  struct is_const_accessor<detail::index_accessor<View>>
    : public boost::false_type
  { };

  template <typename Iterator, typename ParentAccessor>
  struct is_const_accessor<
           detail::remote_pointer_accessor<Iterator, ParentAccessor>>
    : public boost::false_type
  { };

  template <typename Container>
  struct is_const_accessor<
           graph_accessor<Container>>
    : public boost::false_type
  { };

  template <typename Iterator, typename ConstIterator, typename Accessor,
            bool = is_const_accessor<Accessor>::value>
  struct select_iterator
  {
    typedef Iterator type;
  };

  template <typename Iterator, typename ConstIterator, typename Accessor>
  struct select_iterator<Iterator, ConstIterator, Accessor, true>
  {
    typedef ConstIterator type;
  };
}

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref vertex_adj_list_impl.
/// @ingroup pgraphAdjacencyVertex
/// @tparam VD Vertex descriptor type for the graph.
/// @tparam Property Property type of the vertex.
/// @tparam AdjList Type of the edgelist for storing edges.
/// @tparam Accessor Type of the accessor for the proxy.
//////////////////////////////////////////////////////////////////////
template<class VD, class Property, class AdjList, typename Accessor>
class proxy<vertex_adj_list_impl<VD, Property, AdjList>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef vertex_adj_list_impl<VD, Property, AdjList> target_t;
  typedef typename graph_storage_impl::select_iterator<
            typename target_t::adj_edge_iterator,
            typename target_t::const_adj_edge_iterator,
            Accessor
          >::type adj_edge_iter_t;
  typedef typename target_t::const_adj_edge_iterator  const_adj_edge_iter_t;

public:
  typedef typename target_t::property_type            property_type;
  typedef typename Accessor::property_reference       property_reference;
  typedef typename Accessor::const_property_reference const_property_reference;
  typedef typename target_t::vertex_descriptor        vertex_descriptor;
  typedef typename target_t::edge_descriptor          edge_descriptor;
  typedef typename target_t::adj_edges_type           adj_edges_type;

  typedef member_iterator<adj_edge_iter_t, Accessor>  adj_edge_iterator;
  typedef member_iterator<const_adj_edge_iter_t, Accessor>
                                                      const_adj_edge_iterator;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  proxy const& operator=(proxy const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  proxy const& operator=(target_t const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Is this more efficient than accessor_t::read()?
  //////////////////////////////////////////////////////////////////////
  // operator target_t() const { return Accessor::ref(); }
  operator target_t(void) const
  { return Accessor::read(); }

  vertex_descriptor descriptor() const
  { return Accessor::index(); }

  property_reference property(void)
  { return Accessor::property(); }

  const_property_reference property(void) const
  { return Accessor::const_property(); }

  const_property_reference const_property(void) const
  { return Accessor::const_property(); }

  template <typename Comp>
  void sort_edges(Comp const& comp)
  { Accessor::invoke(&target_t::template sort_edges<Comp>, comp); }

  adj_edge_iterator begin(void)
  { return adj_edge_iterator(Accessor::invoke(&target_t::begin), *this); }

  adj_edge_iterator end(void)
  { return adj_edge_iterator(Accessor::invoke(&target_t::end), *this); }

  adj_edge_iterator find_edge(vertex_descriptor const& vd)
  { return adj_edge_iterator(Accessor::invoke(&target_t::find, vd), *this); }

  adj_edge_iterator find_edge(edge_descriptor const& ed)
  { return adj_edge_iterator(Accessor::invoke(&target_t::find, ed), *this); }

  const_adj_edge_iterator begin(void) const
  {
    return const_adj_edge_iterator(Accessor::const_invoke(&target_t::begin),
                                   *this);
  }

  const_adj_edge_iterator end(void) const
  {
    return const_adj_edge_iterator(Accessor::const_invoke(&target_t::end),
                                   *this);
  }


  const_adj_edge_iterator find_edge(vertex_descriptor const& vd) const
  {
    return const_adj_edge_iterator(Accessor::const_invoke(&target_t::find, vd),
                                   *this);
  }

  const_adj_edge_iterator find_edge(edge_descriptor const& ed) const
  {
    return const_adj_edge_iterator(Accessor::const_invoke(&target_t::find, ed),
                                   *this);
  }

  STAPL_PROXY_METHOD_RETURN(size, size_t)

  adj_edges_type edges(void)
  { return Accessor::ref().edges();}

  void reserve(size_t n)
  { Accessor::ref().reserve(n);}

  void erase(size_t n)
  { Accessor::ref().erase(n);}

  void erase_at(size_t n)
  { Accessor::ref().erase_at(n);}

  void erase(size_t start, size_t end)
  { Accessor::ref().erase(start, end);}

  template <typename Iter>
  void erase(Iter it)
  { Accessor::ref().erase(it);}

  void clear(void)
  { Accessor::ref().clear(); }
}; //struct proxy


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref vertex_adj_list_impl,
/// specialized for @ref local_accessor_graph, which lets us use a
/// faster iterator over the edges than the @ref member_iterator
/// needed for the @ref container_accessor.
/// @ingroup pgraphAdjacencyVertex
/// @tparam VD Vertex descriptor type for the graph.
/// @tparam Property Property type of the vertex.
/// @tparam AdjList Type of the edgelist for storing edges.
/// @tparam Accessor Type of the accessor for the proxy.
//////////////////////////////////////////////////////////////////////
template<class VD, class Property, class AdjList, typename Container>
class proxy<vertex_adj_list_impl<VD, Property, AdjList>,
            local_accessor_graph<Container> >
  : public local_accessor_graph<Container>
{
private:
  typedef local_accessor_graph<Container> accessor_t;
  friend class proxy_core_access;
  typedef vertex_adj_list_impl<VD, Property, AdjList> target_t;
public:
  typedef typename target_t::property_type            property_type;
  typedef typename accessor_t::property_reference     property_reference;
  typedef typename accessor_t::const_property_reference
                                                       const_property_reference;
  typedef typename target_t::vertex_descriptor        vertex_descriptor;
  typedef typename target_t::edge_descriptor          edge_descriptor;
  typedef typename target_t::adj_edge_iterator        adj_edge_iterator;
  typedef typename target_t::const_adj_edge_iterator  const_adj_edge_iterator;
  typedef typename target_t::adj_edges_type           adj_edges_type;

  explicit proxy(accessor_t const& acc)
    : accessor_t(acc)
  { }

  proxy const& operator=(proxy const& rhs)
  {
    accessor_t::write(rhs);
    return *this;
  }

  proxy const& operator=(target_t const& rhs)
  {
    accessor_t::write(rhs);
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Is this more efficient than accessor_t::read()?
  //////////////////////////////////////////////////////////////////////
  // operator target_t() const { return accessor_t::ref(); }
  operator target_t(void) const
  { return accessor_t::read(); }

  vertex_descriptor descriptor()  const
  { return accessor_t::ref().descriptor(); }

  property_reference property(void)
  { return accessor_t::property(); }

  const_property_reference property(void) const
  { return accessor_t::const_property(); }

  const_property_reference const_property(void) const
  { return accessor_t::const_property(); }

  template <typename Comp>
  void sort_edges(Comp const& comp)
  { accessor_t::ref().sort_edges(comp); }

  adj_edge_iterator begin(void)
  { return accessor_t::ref().begin(); }

  adj_edge_iterator end(void)
  { return accessor_t::ref().end(); }

  adj_edge_iterator find_edge(vertex_descriptor const& vd)
  { return accessor_t::ref().find(vd); }

  adj_edge_iterator find_edge(edge_descriptor const& ed)
  { return accessor_t::ref().find(ed); }

  const_adj_edge_iterator begin(void) const
  { return accessor_t::ref().begin(); }

  const_adj_edge_iterator end(void) const
  { return accessor_t::ref().end(); }

  const_adj_edge_iterator find_edge(vertex_descriptor const& vd) const
  { return accessor_t::ref().find(vd); }

  const_adj_edge_iterator find_edge(edge_descriptor const& ed) const
  { return accessor_t::ref().find(ed); }

  size_t size(void) const
  { return accessor_t::ref().size();}

  adj_edges_type edges(void)
  { return accessor_t::ref().edges();}

  void reserve(size_t n)
  { accessor_t::ref().reserve(n);}

  void erase(size_t n)
  { accessor_t::ref().erase(n);}

  void erase_at(size_t n)
  { accessor_t::ref().erase_at(n);}

  void erase(size_t start, size_t end)
  { accessor_t::ref().erase(start, end);}

  template <typename Iter>
  void erase(Iter it)
  { accessor_t::ref().erase(it);}

  void clear(void)
  { accessor_t::ref().clear(); }
}; //struct proxy


//////////////////////////////////////////////////////////////////////
/// @brief This class is a (specialization of) @ref proxy for
/// @ref sequential::graph_edge.
/// @ingroup pgraphDistObj
//////////////////////////////////////////////////////////////////////
STAPL_PROXY_HEADER_TEMPLATE(sequential::graph_edge, VD)
{
  STAPL_PROXY_DEFINES(sequential::graph_edge<VD>)

  STAPL_PROXY_REFLECT_TYPE(vertex_descriptor)
  STAPL_PROXY_REFLECT_TYPE(edge_descriptor_type)
  STAPL_PROXY_REFLECT_TYPE(edge_id_type)

  STAPL_PROXY_METHOD_RETURN(descriptor, edge_descriptor_type)
  STAPL_PROXY_METHOD_RETURN(id, edge_id_type)
  STAPL_PROXY_METHOD_RETURN(source, vertex_descriptor)
  STAPL_PROXY_METHOD_RETURN(target, vertex_descriptor)
}; //struct proxy

//////////////////////////////////////////////////////////////////////
/// @brief This class is a (specialization of) @ref proxy for
/// @ref sequential::short_graph_edge.
/// @ingroup pgraphDistObj
//////////////////////////////////////////////////////////////////////
STAPL_PROXY_HEADER_TEMPLATE(sequential::short_graph_edge, VD)
{
  STAPL_PROXY_DEFINES(sequential::short_graph_edge<VD>)

  STAPL_PROXY_REFLECT_TYPE(vertex_descriptor)
  STAPL_PROXY_REFLECT_TYPE(edge_descriptor_type)

  STAPL_PROXY_METHOD_RETURN(target, vertex_descriptor)
}; //struct proxy

//////////////////////////////////////////////////////////////////////
/// @brief This class is a (specialization of) @ref proxy for
/// @ref sequential::graph_property_edge.
/// @ingroup pgraphDistObj
//////////////////////////////////////////////////////////////////////
STAPL_PROXY_HEADER_TEMPLATE(sequential::graph_property_edge, VD, Property)
{
  STAPL_PROXY_TYPES(
    STAPL_PROXY_CONCAT(sequential::graph_property_edge<VD, Property>),
    Accessor)
  STAPL_PROXY_METHODS(
    STAPL_PROXY_CONCAT(sequential::graph_property_edge<VD, Property>),
    Accessor)

  proxy(Accessor const& acc)
    : Accessor(acc) { }

  STAPL_PROXY_REFLECT_TYPE(vertex_descriptor)
  STAPL_PROXY_REFLECT_TYPE(edge_descriptor_type)
  STAPL_PROXY_REFLECT_TYPE(edge_id_type)
  STAPL_PROXY_REFLECT_TYPE(property_type)

  STAPL_PROXY_REFERENCE_METHOD_0(property, property_type)
  STAPL_PROXY_METHOD_RETURN(descriptor, edge_descriptor_type)
  STAPL_PROXY_METHOD_RETURN(id, edge_id_type)
  STAPL_PROXY_METHOD_RETURN(source, vertex_descriptor)
  STAPL_PROXY_METHOD_RETURN(target, vertex_descriptor)
}; //struct proxy

//////////////////////////////////////////////////////////////////////
/// @brief This class is a (specialization of) @ref proxy for
/// @ref sequential::undirected_graph_edge_property.
/// @ingroup pgraphDistObj
//////////////////////////////////////////////////////////////////////
STAPL_PROXY_HEADER_TEMPLATE(sequential::undirected_graph_edge_property,
                            VD, Property)
{
  STAPL_PROXY_TYPES(
    STAPL_PROXY_CONCAT(sequential::undirected_graph_edge_property<VD,
                                                                  Property>),
    Accessor)
  STAPL_PROXY_METHODS(
    STAPL_PROXY_CONCAT(sequential::undirected_graph_edge_property<VD,
                                                                  Property>),
    Accessor)

  proxy(Accessor const& acc)
    : Accessor(acc) { }

  STAPL_PROXY_REFLECT_TYPE(vertex_descriptor)
  STAPL_PROXY_REFLECT_TYPE(edge_descriptor_type)
  STAPL_PROXY_REFLECT_TYPE(edge_id_type)
  STAPL_PROXY_REFLECT_TYPE(property_type)

  STAPL_PROXY_REFERENCE_METHOD_0(property, property_type)
  STAPL_PROXY_METHOD_RETURN(descriptor, edge_descriptor_type)
  STAPL_PROXY_METHOD_RETURN(id, edge_id_type)
  STAPL_PROXY_METHOD_RETURN(source, vertex_descriptor)
  STAPL_PROXY_METHOD_RETURN(target, vertex_descriptor)
}; //struct proxy


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref memory_used for @ref vertex_adj_list_impl.
/// @ingroup pgraphAdjacencyVertex
/// @tparam VD Vertex descriptor type for the graph.
/// @tparam Property Property type of the vertex.
/// @tparam AdjList Type of the edgelist for storing edges.
//////////////////////////////////////////////////////////////////////
template<class VD, class Property, class AdjList>
struct memory_used<vertex_adj_list_impl<VD,Property,AdjList> >
{
  typedef vertex_adj_list_impl<VD,Property,AdjList> vertex_type;
  static size_t size(vertex_type const& v)
  { return v.memory_size(); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Adjacency list of a vertex implemented as a vector of edges.
/// @ingroup pgraphAdjacencyImpl
///
/// Used inside the @ref vertex_adj_list_impl to store adjacent edges.
/// @tparam Edge The type of edge to be stored.
//////////////////////////////////////////////////////////////////////
template <class Edge>
class adjacency_list_impl
{
  template<class Traits> friend class adjacency_list_model;
 public:
  //infer property
  typedef Edge                                                edge_type;
  typedef typename Edge::vertex_descriptor                    vertex_descriptor;
  typedef typename Edge::edge_descriptor_type                 edge_descriptor;
  typedef typename std::vector<Edge>::iterator                iterator;
  typedef typename std::vector<Edge>::const_iterator          const_iterator;
  typedef typename std::iterator_traits<iterator>::reference  reference;
  typedef typename std::iterator_traits<iterator>::value_type value_type;

  /// Create an edgelist with specified number of edges.
  adjacency_list_impl(size_t n=0)
    : m_data(n)
  { }

  adjacency_list_impl(adjacency_list_impl const& other)
  { m_data = other.m_data; }

  adjacency_list_impl& operator=(adjacency_list_impl const& other)
  {
    m_data = other.m_data;
    return *this;
  }

  iterator begin(void)
  { return m_data.begin(); }

  const_iterator begin(void) const
  { return m_data.begin(); }

  iterator end(void)
  { return m_data.end(); }

  const_iterator end(void) const
  { return m_data.end(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Not used, provided for compatibility.
  //////////////////////////////////////////////////////////////////////
  iterator remote_begin(void)
  { return m_data.begin(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Not used, provided for compatibility.
  //////////////////////////////////////////////////////////////////////
  const_iterator remote_begin(void) const
  { return m_data.begin(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the edge with the specified target.
  /// @param vd The descriptor of the target vertex.
  //////////////////////////////////////////////////////////////////////
  iterator find(vertex_descriptor const& vd)
  {
    return graph_find(m_data.begin(), m_data.end(),
                      eq_target<vertex_descriptor>(vd));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the specified edge.
  /// @param ed The descriptor of the edge.
  //////////////////////////////////////////////////////////////////////
  iterator find(edge_descriptor const& ed)
  {
    return graph_find(m_data.begin(), m_data.end(),
                      eq_ed<edge_descriptor>(ed));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the edge with the specified target.
  /// @param vd The descriptor of the target vertex.
  //////////////////////////////////////////////////////////////////////
  const_iterator find(vertex_descriptor const& vd) const
  {
    return graph_find(m_data.begin(), m_data.end(),
                      eq_target<vertex_descriptor>(vd));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the specified edge.
  /// @param ed The descriptor of the edge.
  //////////////////////////////////////////////////////////////////////
  const_iterator find(edge_descriptor const& ed) const
  {
    return graph_find(m_data.begin(), m_data.end(),
                      eq_ed<edge_descriptor>(ed));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of edges in the edgelist.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  { return m_data.size(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the storage capacity of the edgelist.
  //////////////////////////////////////////////////////////////////////
  size_t capacity(void) const
  { return m_data.capacity(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reserves space for the specified number of edges.
  //////////////////////////////////////////////////////////////////////
  void reserve(size_t n)
  { m_data.reserve(n); }

#ifdef _STAPL
  void define_type(typer& t)
  { t.member(m_data); }

  size_t memory_size(void) const
  { return memory_used<std::vector<Edge> >::size(m_data); }
#endif

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge to the front of edgelist.
  //////////////////////////////////////////////////////////////////////
  void add_dummy(void)
  { m_data.push_front(Edge()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clears the edgelist.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  { m_data.clear(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erases all edges from index n until the end.
  /// @param n The starting index of the range of edges to be deleted.
  //////////////////////////////////////////////////////////////////////
  void erase(size_t n)
  { m_data.erase(m_data.begin()+n, m_data.end()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erases edge at index n.
  /// @param n The index of the edge to be deleted.
  //////////////////////////////////////////////////////////////////////
  void erase_at(size_t n)
  { m_data.erase(m_data.begin()+n); }

    //////////////////////////////////////////////////////////////////////
  /// @brief Erases all edges from index start until one before index end.
  /// @param start The starting index of the range of edges to be deleted.
  /// @param end The index right after the range of edges to be deleted.
  //////////////////////////////////////////////////////////////////////
  void erase(size_t start, size_t end)
  { m_data.erase(m_data.begin()+start, m_data.begin()+end); }

 protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the specified edge to the back of edgelist.
  /// This interface is specific to adjacency list and is accessible only
  /// from @ref sequential::adjacency_list_graph, as we need to make sure
  /// the edge's source is the correct vertex.
  /// @param ed The edge to be inserted into the edgelist.
  //////////////////////////////////////////////////////////////////////
  void add(const Edge& ed)
  { m_data.push_back(ed); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the specified edge before the edge which causes the comp
  /// function to return false (requires the edges to already be sorted).
  /// This interface is specific to adjacency list and is accessible only
  /// from @ref sequential::adjacency_list_graph, as we need to make sure
  /// the edge's source is the correct vertex.
  /// @param ed The edge to be inserted into the edgelist.
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  void insert(Edge const& ed, Comp const& comp)
  {
    if (m_data.size() == 0 || comp(*(m_data.end()-1), ed)) {
      m_data.push_back(ed);
    } else {
      auto it = std::lower_bound(m_data.begin(), m_data.end(), ed, comp);
      m_data.insert(it, ed);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erases the edge pointed to by the specified iterator.
  /// @param it An iterator pointing to the edge to be deleted.
  //////////////////////////////////////////////////////////////////////
  void erase(iterator it)
  { m_data.erase(it); }

  void erase(iterator itb, iterator ite)
  { m_data.erase(itb, ite); }

  /// The actual list of edges is stored in an std::vector.
  std::vector<Edge> m_data;
};


//////////////////////////////////////////////////////////////////////
/// @brief Adjacency list of a vertex implemented as an unordered map of edges.
/// @ingroup pgraphAdjacencyImpl
///
/// Used inside the @ref vertex_adj_list_impl to store adjacent edges.
/// @tparam Edge The type of edge to be stored.
//////////////////////////////////////////////////////////////////////
template <class Edge>
class unordered_map_adjacency_list_impl
{
  template<class Traits> friend class adjacency_list_model;

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper to return the edge stored in the unordered map.
  /// @tparam E The type of edge.
  //////////////////////////////////////////////////////////////////////
  template<typename E>
  struct select_2nd
  {
    typedef E& result_type;
    template<typename T>
    typename T::second_type& operator()(T& t) const
    { return t.second; }
  };

 public:
  //infer property
  typedef Edge                                                edge_type;
  typedef typename Edge::vertex_descriptor                    vertex_descriptor;
  typedef typename Edge::edge_descriptor_type                 edge_descriptor;
  typedef boost::unordered_map<vertex_descriptor, edge_type>  storage_type;
  typedef boost::transform_iterator<select_2nd<edge_type>,
                                    typename storage_type::iterator>   iterator;
  typedef boost::transform_iterator<select_2nd<edge_type>,
                        typename storage_type::const_iterator>   const_iterator;
  typedef typename std::iterator_traits<iterator>::reference  reference;
  typedef typename std::iterator_traits<iterator>::value_type value_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an edgelist with specified number of edges.
  //////////////////////////////////////////////////////////////////////
  unordered_map_adjacency_list_impl(size_t n=0)
    : m_data(n)
  { }

  unordered_map_adjacency_list_impl(unordered_map_adjacency_list_impl
                                    const& other)
  { m_data = other.m_data; }

  unordered_map_adjacency_list_impl& operator=(unordered_map_adjacency_list_impl
                                               const& other)
  {
    m_data = other.m_data;
    return *this;
  }

  iterator begin(void)
  {
    return boost::make_transform_iterator(m_data.begin(),
                                          select_2nd<edge_type>());
  }

  const_iterator begin(void) const
  {
    return boost::make_transform_iterator(m_data.begin(),
                                          select_2nd<edge_type>());
  }

  iterator end(void)
  {
    return boost::make_transform_iterator(m_data.end(),
                                          select_2nd<edge_type>());
  }

  const_iterator end(void) const
  {
    return boost::make_transform_iterator(m_data.end(),
                                          select_2nd<edge_type>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Not used, provided for compatibility.
  //////////////////////////////////////////////////////////////////////
  iterator remote_begin(void)
  { return this->begin(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Not used, provided for compatibility.
  //////////////////////////////////////////////////////////////////////
  const_iterator remote_begin(void) const
  { return this->begin(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the edge with the specified target.
  /// @param vd The descriptor of the target vertex.
  //////////////////////////////////////////////////////////////////////
  iterator find(vertex_descriptor const& vd)
  {
    return boost::make_transform_iterator(m_data.find(vd),
                                          select_2nd<edge_type>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the specified edge.
  /// @param ed The descriptor of the edge.
  //////////////////////////////////////////////////////////////////////
  iterator find(edge_descriptor const& ed)
  {
    return boost::make_transform_iterator(m_data.find(ed.target()),
                                          select_2nd<edge_type>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the edge with the specified target.
  /// @param vd The descriptor of the target vertex.
  //////////////////////////////////////////////////////////////////////
  const_iterator find(vertex_descriptor const& vd) const
  {
    return boost::make_transform_iterator(m_data.find(vd),
                                          select_2nd<edge_type>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the specified edge.
  /// @param ed The descriptor of the edge.
  //////////////////////////////////////////////////////////////////////
  const_iterator find(edge_descriptor const& ed) const
  {
    return boost::make_transform_iterator(m_data.find(ed.target()),
                                          select_2nd<edge_type>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of edges in the edgelist.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  { return m_data.size(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the storage capacity of the edgelist.
  //////////////////////////////////////////////////////////////////////
  size_t capacity(void) const
  { return m_data.size(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Not used, provided for compatibility.
  //////////////////////////////////////////////////////////////////////
  void reserve(size_t n)
  { ; }

#ifdef _STAPL
  void define_type(typer& t)
  { t.member(m_data); }

  size_t memory_size(void) const
  { return memory_used<std::vector<Edge> >::size(m_data); }
#endif

  //////////////////////////////////////////////////////////////////////
  /// @brief Clears the edgelist.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  { m_data.clear(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erases all edges from index n until the end.
  /// @param n The starting index of the range of edges to be deleted.
  //////////////////////////////////////////////////////////////////////
  void erase(size_t n)
  { m_data.erase(m_data.begin()+n, m_data.end()); }

 protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the specified edge to the front of edgelist.
  /// This interface is specific to adjacency list and is accessible only
  /// from adjacency_list_graph, as we need to make sure the edge's source
  /// is the correct vertex.
  /// @param ed The edge to be inserted into the edgelist.
  //////////////////////////////////////////////////////////////////////
  void add(const Edge& ed)
  { m_data[ed.target()] = ed; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erases the edge pointed to by the specified iterator.
  /// @param it An iterator pointing to the edge to be deleted.
  //////////////////////////////////////////////////////////////////////
  void erase(iterator it)
  { m_data.erase(it); }

  /// Data: Stores the list of edges
  storage_type m_data;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref memory_used for @ref adjacency_list_impl.
/// @ingroup pgraphAdjacencyImpl
/// @tparam Edge Type of the Edge.
//////////////////////////////////////////////////////////////////////
template <class Edge>
struct memory_used<adjacency_list_impl<Edge> >
{
  typedef adjacency_list_impl<Edge> edge_type;
  static size_t size(edge_type const& v)
  { return v.memory_size(); }
};




//////////////////////////////////////////////////////////////////////
/// @brief An adjacency list using an std::vector for storing vertices.
/// @ingroup pgraphAdjacencyImpl
///
/// Used inside the @ref adjacency_list_model to store vertices.
/// @tparam Traits The traits class for specifying the types of descriptors,
/// storages, edges and vertices.
//////////////////////////////////////////////////////////////////////
template <class Traits>
class adjacency_list_vector_storage
{
 public:
  typedef typename Traits::vertex_descriptor        vertex_descriptor;
  typedef typename Traits::vertex_property          vertex_property;
  typedef typename Traits::edge_type                edge_type;
  typedef typename Traits::edgelist_type            edgelist_type;
  typedef typename Traits::vertex_impl_type         vertex_impl_type;

  /// Type of the vertex descriptor generator.
  typedef sequential::vdg_base_int<vertex_descriptor> vdg_type;

  /// The vertices are stored in an std::vector.
  typedef std::vector<vertex_impl_type>               vertex_set_type;


 protected:
  // Data members.
  /// The vertex descriptor generator.
  vdg_type         m_vdg;

  /// The container for storing vertices.
  vertex_set_type  m_storage;

  /// The descriptor of the starting vertex in this storage.
  size_t           my_local_start_vd;

 public:
  typedef typename vertex_set_type::iterator       iterator;
  typedef typename vertex_set_type::const_iterator const_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a storage with the given number of vertices starting
  /// from the specified descriptor, with the given default value.
  /// Vertices have contiguous descriptors.
  /// @param start_vd The starting descriptor of the vertices.
  /// @param num_vds The number of vertices to be initially stored.
  /// @param default_value The default value for the vertices.
  //////////////////////////////////////////////////////////////////////
  adjacency_list_vector_storage(size_t const& start_vd,
                                size_t const& num_vds,
                                vertex_impl_type const& default_value)
    : m_vdg(start_vd), my_local_start_vd(start_vd)
  {
    // init the vertex descriptors and adj lists
    m_storage.reserve(num_vds);
    for (size_t i=0; i <num_vds; ++i) {
      vertex_descriptor vd = m_vdg.next();
      m_storage.push_back(vertex_impl_type(vd, default_value.property()));
    }
  }

  adjacency_list_vector_storage(adjacency_list_vector_storage const& other)
    : m_vdg(other.m_vdg)
  {
    this->m_storage = other.m_storage;
    // the edge list is a pointer; next clone the edge lists
    typename vertex_set_type::iterator storage_it = this->m_storage.begin();
    typename vertex_set_type::const_iterator
      storage_it_other = other.m_storage.begin();
    typename vertex_set_type::const_iterator
      storage_it_other_end = other.m_storage.end();
    for (typename vertex_set_type::const_iterator it = storage_it_other;
         it != storage_it_other_end; ++it, ++storage_it) {
      storage_it->edgelist() = (it->edgelist());
    }
    my_local_start_vd = get_location_id()*m_storage.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Resize the internal storage to accommodate specified number of
  /// vertices.
  //////////////////////////////////////////////////////////////////////
  void resize(size_t nv)
  {
    this->m_storage.clear();
    vertex_property p = vertex_property(); // default constructed property;
    for (size_t i=0; i<nv; ++i) {
      vertex_descriptor vd = m_vdg.next();
      m_storage.push_back(vertex_impl_type(vd, p));
    }
  }

  iterator begin(void)
  { return this->m_storage.begin(); }

  iterator end(void)
  { return this->m_storage.end(); }

  const_iterator begin(void) const
  { return this->m_storage.begin(); }

  const_iterator end()   const
  { return this->m_storage.end(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of vertices in the storage; for use in
  /// g.num_vertices().
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  { return this->m_storage.size(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds vertex to the storage with the given vertex descriptor.
  /// @param vd The descriptor of the added vertex.
  /// @return The descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_descriptor& vd)
  {
    return add_vertex(vd, vertex_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds vertex to the storage with the given vertex property.
  /// The descriptor is assigned automatically using the descriptor generator.
  /// @param vp The vertex property of the added vertex.
  /// @return The descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_property const& vp)
  {
    return add_vertex(m_vdg.next(), vp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds vertex to the storage with the given descriptor and property.
  /// @param vd The explicit descriptor of the added vertex.
  /// @param vp The vertex property of the added vertex.
  /// @return The descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_descriptor vd, vertex_property const& vp)
  {
    vertex_impl_type v(vd, vp);
    m_vdg.update(vd);
    m_storage.push_back(v);
    return vd;
  }

  vertex_descriptor next_free_descriptor()
  {
    return m_vdg.next();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reserves space for storing specified number of edges in the
  /// specified vertex.
  /// @param vd The descriptor of the vertex.
  /// @param num_adjacents The number of adjacents to be stored.
  //////////////////////////////////////////////////////////////////////
  void reserve_adjacency(vertex_descriptor const& vd, size_t num_adjacents)
  {
    (*(this->find_vertex(vd))).reserve(num_adjacents);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the vertex pointed to by the specified iterator.
  /// @param vi The iterator of the vertex to be deleted.
  /// @return Whether or not the vertex was successfully deleted.
  /// @note As this is the static storage, deletion of vertices is not allowed.
  //////////////////////////////////////////////////////////////////////
  bool delete_vertex(vertex_descriptor const&, iterator const& vi)
  { return false; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clears the storage container of all vertices and edges.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    for (typename vertex_set_type::iterator i=this->m_storage.begin();
        i!=this->m_storage.end();++i)
      i->clear();

    this->m_storage.clear();
    this->m_vdg.reset();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the specified vertex
  //////////////////////////////////////////////////////////////////////
  iterator find_vertex(vertex_descriptor const& vd)
  {
    return m_storage.begin() + (vd - my_local_start_vd);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the specified vertex
  //////////////////////////////////////////////////////////////////////
  const_iterator find_vertex(vertex_descriptor const& vd) const
  {
    return m_storage.begin() + (vd - my_local_start_vd);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Not used. Provided for compatibility.
  //////////////////////////////////////////////////////////////////////
  void update_descriptor(vertex_descriptor& vd, iterator const& vi)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the current max descriptor.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor get_max_descriptor(void)
  { return m_vdg.curr_vd(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Updates the vertex descriptor generator with the next free
  ///        descriptor
  //////////////////////////////////////////////////////////////////////
  void update_next_descriptor(vertex_descriptor const& vd)
  {
    this->m_vdg.update(vd);
  }

#ifdef _STAPL
  void define_type(typer& t)
  {
    t.member(m_storage);
    t.member(m_vdg);
  }

  size_t memory_size(void) const
  {
    size_t sz=0;
    sz+=sizeof(m_vdg);
    for (size_t i=0; i < m_storage.size(); ++i) {
      sz+=m_storage[i].memory_size();
    }
    return sz;
  }
#endif
}; // vec_adj_lists storage class



//////////////////////////////////////////////////////////////////////
/// @brief This model describes operations on a graph that
/// is modeled as an adjacency-list. Adjacency-lists store a list of vertices
/// where each vertex has a list of edges to its adjacents.
/// @ingroup pgraphAdjacency
/// @tparam Traits The traits class for specifying the types of descriptors,
/// storages, edges and vertices, etc.
///
/// Methods like find_vertex, find_edge are optimized for this particular
/// storage.
//////////////////////////////////////////////////////////////////////
template <class Traits>
class adjacency_list_model
{
public:
  typedef typename Traits::vertex_descriptor  vertex_descriptor;
  typedef typename Traits::simple_vertex_descriptor  simple_vertex_descriptor;
  typedef typename Traits::edge_descriptor    edge_descriptor;
  typedef typename Traits::vertex_property    vertex_property;
  typedef typename Traits::edge_property      edge_property;
  /// The type of the edge.
  typedef typename Traits::edge_type          edge_type;
  /// The type of storage for vertices.
  typedef typename Traits::storage_type     vertex_set_type;
  /// The type of storage for edges on a vertex.
  typedef typename Traits::edgelist_type    edgelist_type;
  /// The type of the vertex.
  typedef typename Traits::vertex_impl_type vertex_impl_type;
  typedef vertex_impl_type                  value_type;

protected:
  /// The container for storing the vertices.
  vertex_set_type m_vertices;

  /// The number of edges in this adjacency-list, excluding self edges.
  size_t m_ne;

  // The number of self edges in this adjacency-list.
  size_t m_se;

  /// The number to keep track of unique edge-descriptors generated.
  size_t m_eid;

public:

  typedef typename vertex_set_type::iterator           iterator;
  typedef typename vertex_set_type::const_iterator     const_iterator;
  typedef iterator                                     vertex_iterator;
  typedef const_iterator                               const_vertex_iterator;

  /// Type of the iterator over adjacent edges of a vertex.
  typedef typename edgelist_type::iterator             adj_edge_iterator;
  typedef typename edgelist_type::const_iterator       const_adj_edge_iterator;

  /// Type of the edge iterator over all edges stored in the adjacency-list.
  typedef sequential::edge_iterator_adaptor<
                      vertex_iterator>                 edge_iterator;
  typedef sequential::edge_iterator_adaptor<
                      const_vertex_iterator>           const_edge_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an adjacency-list with the given number of vertices
  /// starting from the specified descriptor, with the given default value.
  /// Vertices have contiguous descriptors.
  /// @param start_vd The starting descriptor of the vertices.
  /// @param num_vds The number of vertices to be initially stored.
  /// @param default_value The default value for the vertices.
  //////////////////////////////////////////////////////////////////////
  adjacency_list_model(size_t const&  start_vd, size_t const&  num_vds,
                       value_type const& default_value = value_type())
    : m_vertices(start_vd, num_vds, default_value), m_ne(0), m_se(0), m_eid(0)
  { }

  adjacency_list_model(adjacency_list_model const& other)
    : m_vertices(other.m_vertices)
  {
    this->m_eid = other.m_eid;
    this->m_ne  = other.m_ne;
    this->m_se  = other.m_se;
  }

  vertex_iterator begin(void)
  { return this->m_vertices.begin(); }

  vertex_iterator end(void)
  { return this->m_vertices.end(); }

  const_vertex_iterator begin(void) const
  { return this->m_vertices.begin(); }

  const_vertex_iterator end(void) const
  { return this->m_vertices.end(); }

  vertex_descriptor next_free_descriptor()
  {
    return m_vertices.next_free_descriptor();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an edge iterator to the first edge in the entire
  /// adjacency-list.
  //////////////////////////////////////////////////////////////////////
  edge_iterator edges_begin(void)
  {
    //find first vertex with edges
    typename vertex_set_type::iterator it = this->m_vertices.begin();
    while (it != this->m_vertices.end() && it->edgelist().size() == 0) ++it;
    //check if its not the end
    if (it == this->m_vertices.end())
      return edges_end();
    else
      return edge_iterator(it->edgelist().begin(),
                         this->m_vertices.begin(), it,
                         this->m_vertices.end());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an edge iterator to the first edge in the adjacency-list.
  //////////////////////////////////////////////////////////////////////
  const_edge_iterator edges_begin(void) const
  {
    //find first vertex with edges
    typename vertex_set_type::const_iterator it = this->m_vertices.begin();
    while (it != this->m_vertices.end() && it->edgelist().size() == 0) ++it;
    //check if its not the end
    if (it == this->m_vertices.end())
      return edges_end();
    else
      return const_edge_iterator(it->edgelist().begin(),
                         this->m_vertices.begin(), it,
                         this->m_vertices.end());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an edge iterator to one past the last edge in the
  /// adjacency-list.
  //////////////////////////////////////////////////////////////////////
  edge_iterator edges_end(void)
  {
    typename vertex_set_type::iterator it = this->m_vertices.end();
    --it;
    return edge_iterator((it)->edgelist().end(),
                         this->m_vertices.begin(), it,
                         this->m_vertices.end());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an edge iterator to the last edge in the adjacency-list.
  //////////////////////////////////////////////////////////////////////
  const_edge_iterator edges_end(void) const
  {
    typename vertex_set_type::const_iterator it = this->m_vertices.end();
    --it;
    return const_edge_iterator((it)->edgelist().end(),
                               this->m_vertices.begin(), it,
                               this->m_vertices.end());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the total number of vertices.
  //////////////////////////////////////////////////////////////////////
  size_t num_vertices(void) const
  {
    return this->m_vertices.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the total number of vertices.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return this->m_vertices.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the current max descriptor.
  //////////////////////////////////////////////////////////////////////
  size_t get_max_descriptor(void)
  {
    return this->m_vertices.get_max_descriptor();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Updates the vertex descriptor generator with the next free
  ///        descriptor
  //////////////////////////////////////////////////////////////////////
  void update_next_descriptor(vertex_descriptor const& vd)
  {
    this->m_vertices.update_next_descriptor(vd);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the total number of edges.
  //////////////////////////////////////////////////////////////////////
  size_t num_edges(void) const
  {
    return m_ne;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the total number of self edges.
  //////////////////////////////////////////////////////////////////////
  size_t num_self_edges(void) const
  {
    return m_se;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reserves space for storing specified number of edges in the
  /// specified vertex.
  /// @param vd The descriptor of the vertex.
  /// @param num_adjacents The number of adjacents to be stored.
  //////////////////////////////////////////////////////////////////////
  void reserve_adjacency(vertex_descriptor const& vd, size_t num_adjacents)
  {
    m_vertices.reserve_adjacency(vd, num_adjacents);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds vertex with the default vertex property.
  /// A descriptor is automatically generated for the new vertex.
  /// @return The descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(void)
  {
    return add_vertex(vertex_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds vertex with the specified vertex property.
  /// A descriptor is automatically generated for the new vertex.
  /// @param vp The vertex property of the added vertex.
  /// @return The descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_property const& vp)
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
  vertex_descriptor add_vertex(vertex_descriptor vd, vertex_property const& vp)
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
  edge_descriptor add_edge(edge_descriptor const& ed)
  {
    return add_edge(ed,edge_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge with given descriptor and edge property.
  /// @param ed Descriptor of the desired edge.
  /// @param ep Property of the edge.
  /// @return edge_descriptor of the added edge.
  /// @ref edge_descriptor.id() is set to numeric_limits<size_t>::max() if edge
  /// was not added.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& ed, edge_property const& p)
  {
    // don't use reference for the two lines below.
    // this is done for versioning, to prevent adding edges
    // between invalid iterators from another graph.
    vertex_iterator vi = this->find_vertex(ed.source());
    if (vi != this->end()) {
      edge_descriptor ned;
      if (ed.id()!=INVALID_VALUE) {
        //we use the same edge id as the sibling edge
        vi->edgelist().add(edge_type(ed,p));
        ned = ed;
      } else {
        //we allocate a new edge id by incrementing a local counter
        size_t id = m_eid++;
        ned = edge_descriptor(ed.source(), ed.target(), id);
        //edgelist is a method of the vertex
        vi->edgelist().add(edge_type(ned, p));
      }
      if (ed.source() == ed.target()) {
        ++m_se;
      } else {
        ++m_ne;
      }
      return ned;
    } else {
      return edge_descriptor(vertex_descriptor(INVALID_VALUE), ed.target(),
                             INVALID_VALUE);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts an edge with given descriptor and edge property.
  /// @param ed Descriptor of the desired edge.
  /// @param ep Property of the edge.
  /// @return edge_descriptor of the added edge.
  /// @ref edge_descriptor.id() is set to numeric_limits<size_t>::max() if edge
  /// was not added.
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  edge_descriptor insert_edge(edge_descriptor const& ed, edge_property const& p,
                              Comp const& comp)
  {
    // don't use reference for the two lines below.
    // this is done for versioning, to prevent adding edges
    // between invalid iterators from another graph.
    vertex_iterator vi = this->find_vertex(ed.source());
    if (vi != this->end()) {
      edge_descriptor ned;
      if (ed.id()!=INVALID_VALUE) {
        //we use the same edge id as the sibling edge
        vi->edgelist().insert(edge_type(ed,p), comp);
        ned = ed;
      } else {
        //we allocate a new edge id by incrementing a local counter
        size_t id = m_eid++;
        ned = edge_descriptor(ed.source(), ed.target(), id);
        //edgelist is a method of the vertex
        vi->edgelist().insert(edge_type(ned, p), comp);
      }
      if (ed.source() == ed.target()) {
        ++m_se;
      } else {
        ++m_ne;
      }
      return ned;
    } else {
      return edge_descriptor(vertex_descriptor(INVALID_VALUE), ed.target(),
                             INVALID_VALUE);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Checks and adds an edge with given descriptor and property.
  /// The method checks if the edge exists before adding it. If it doesn't,
  /// the edge is added, otherwise it is discarded.
  /// @param ed Descriptor of the desired edge.
  /// @param ep Property of the edge.
  /// @return edge_descriptor of the added edge.
  /// @ref edge_descriptor.id() is set to numeric_limits<size_t>::max() if edge
  /// was not added.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor check_add_edge(edge_descriptor const& ed,
                                 edge_property const& ep)
  {
    vertex_iterator vi;
    adj_edge_iterator ei;
    if (!find_edge(edge_descriptor(ed.source(), ed.target()), vi, ei))
      return add_edge(ed, ep);
    else return edge_descriptor(INVALID_VALUE, INVALID_VALUE, INVALID_VALUE);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Checks and adds an edge with given descriptor and default property.
  /// The method checks if the edge exists before adding it. If it doesn't,
  /// the edge is added, otherwise it is discarded.
  /// @param ed Descriptor of the desired edge.
  /// @return edge_descriptor of the added edge.
  /// @ref edge_descriptor.id() is set to numeric_limits<size_t>::max() if edge
  /// was not added.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor check_add_edge(edge_descriptor const& ed)
  {
    return check_add_edge(ed, edge_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets i-th neighbor of v to the specified neighbor vertex.
  /// @param v Descriptor of the source vertex.
  /// @param i The index of the target neighbor vertex.
  /// @param neighbor The descriptor of the target neighbor vertex.
  /// @return True if the vertex and edge exist, False otherwise.
  //////////////////////////////////////////////////////////////////////
  bool set_edge(vertex_descriptor const& v,
                const size_t i,
                vertex_descriptor const& neighbor)
  {
    vertex_iterator vi = find_vertex(v);
    if (vi == this->end() || (*vi).size() <= i)
      return false;
    (vi->edgelist()).m_data[i] = edge_type(edge_descriptor(v, neighbor));
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
  bool set_edge(vertex_descriptor const& v,
                const size_t i, const size_t max,
                vertex_descriptor const& neighbor)
  {
    vertex_iterator vi = find_vertex(v);
    if (vi == this->end())
      return false;
    for (int j = vi.size(); j < max; ++j) {
      //edgelist is a method of the vertex
      vi->edgelist().add(edge_type(edge_descriptor(INVALID_VALUE,
                         INVALID_VALUE)));
    }
    (vi->edgelist()).m_data[i] = edge_type(edge_descriptor(v, neighbor));
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erases all vertices and edges in the graph.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->m_ne  = 0;
    this->m_se  = 0;
    this->m_eid = 0;
    this->m_vertices.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes all edges with the specified vertex as their target.
  /// For internal use by @ref delete_vertex.
  /// @param vd The descriptor of the vertex.
  //////////////////////////////////////////////////////////////////////
  void delete_all_edges_to_v(vertex_descriptor const& vd)
  {
    //for all vertices check if they have edges pointing to vd
    for (vertex_iterator vi =this->begin(); vi != this->end();++vi) {
      adj_edge_iterator ei = (*vi).find_edge(vd);

      while (ei != (*vi).end()) {
        vi->edgelist().erase(ei);//base() can only be called in this class
        if (ei->target() == vi->descriptor()) {
          --m_se;
        } else {
          --m_ne;
        }
        ei = (*vi).find_edge(vd);
      }
    }
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the specified vertex and all edges pointing to and from it.
  /// @param vd The descriptor of the vertex.
  /// @return Whether or not the vertex was successfully deleted.
  //////////////////////////////////////////////////////////////////////
  bool delete_vertex(vertex_descriptor const& vd)
  {
    delete_all_edges_to_v(vd);
    return suspend_vertex(vd);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the specified vertex, but preserves all edges pointing to
  /// it. Used by migration.
  /// @param vd The descriptor of the vertex.
  /// @return Whether or not the vertex was successfully deleted.
  //////////////////////////////////////////////////////////////////////
  bool suspend_vertex(vertex_descriptor const& vd)
  {
    vertex_iterator vi = find_vertex(vd);
    if (vi != this->end()) {
      for (adj_edge_iterator ei = vi->begin(); ei != vi->end(); ++ei) {
        if (ei->source() == ei->target()) {
          --m_se;
        } else {
          --m_ne;
        }
      }
      vi->clear();//free the edge list
      this->m_vertices.delete_vertex(vd, vi);
      return true;
    }
    else return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the edge with given descriptor.
  /// @param ed Descriptor of the desired edge.
  /// @return Whether or not the edge was successfully deleted.
  //////////////////////////////////////////////////////////////////////
  bool delete_edge(edge_descriptor const& ed)
  {
    vertex_iterator vi;
    adj_edge_iterator ei;
    find_edge(ed, vi, ei);
    if (vi != this->end()) {
      if (ei != (*vi).end()) {
        if (ei->source() == ei->target()) {
          --m_se;
        } else {
          --m_ne;
        }
        vi->edgelist().erase(ei);
        return true;
      }
      else return false;
    }
    else return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erases all edges that match a user-defined predicate
  /// @param pred A unary predicate that receives a single edge
  //////////////////////////////////////////////////////////////////////
  template<typename Pred>
  void erase_edges_if(Pred&& pred)
  {
    for (auto& v : *this) {
      auto it = std::remove_if(v.begin(), v.end(), pred);
      std::size_t start_of_remove = it-v.begin();
      v.erase(start_of_remove);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds the edge with the specified descriptor, and returns an
  /// iterator to its source vertex and an adj_edge_iterator pointing to the
  /// edge.
  /// @param ed Descriptor of the edge.
  /// @param vi vertex_iterator pointing to the source vertex of the edge,
  /// populated by the method.
  /// @param aei adj_edge_iterator pointing to the specified edge,
  /// populated by the method.
  /// @return Whether or not the edge was found.
  //////////////////////////////////////////////////////////////////////
  bool find_edge(edge_descriptor const& ed, vertex_iterator& vi,
                 adj_edge_iterator& ei)
  {
    vi = find_vertex(ed.source());
    if (vi != this->end()) {
      //if the edge id is missing we look for target,
      //source and delete the first encountered
      if (ed.id() == (size_t)INVALID_VALUE)
        ei = (*vi).find_edge(ed.target());
      else { //if the edge descriptor is complete
        //the edge id uniquely identifies an edge
        //!!!the next check is too strong for directed; it can be relaxed by
        //specializing for directed and undirected
        ei = (*vi).find_edge(ed);
      }
      if (ei != (*vi).end())
        return true;
      else return false;
    }
    else return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds the edge with the specified descriptor, and returns an
  /// iterator to its source vertex and an adj_edge_iterator pointing to the
  /// edge.
  /// @param ed Descriptor of the edge.
  /// @param vi vertex_iterator pointing to the source vertex of the edge,
  /// populated by the method.
  /// @param aei adj_edge_iterator pointing to the specified edge,
  /// populated by the method.
  /// @return Whether or not the edge was found.
  //////////////////////////////////////////////////////////////////////
  bool find_edge(edge_descriptor const& ed, const_vertex_iterator& vi,
                 const_adj_edge_iterator& ei) const
  {
    vertex_iterator   i_vi;
    adj_edge_iterator i_ei;
    bool res = const_cast<adjacency_list_model*>(this)->find_edge(ed,i_vi,i_ei);
    vi = const_vertex_iterator(i_vi);
    ei = const_adj_edge_iterator(i_ei.base());
    return res;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds a local edge with the specified descriptor, and returns an
  /// iterator to its source vertex and an adj_edge_iterator pointing to the
  /// edge.
  /// @param ed Descriptor of the edge.
  /// @param vi vertex_iterator pointing to the source vertex of the edge,
  /// populated by the method.
  /// @param aei adj_edge_iterator pointing to the specified edge,
  /// populated by the method.
  /// @return Whether or not the edge was found.
  //////////////////////////////////////////////////////////////////////
  bool find_local_edge(edge_descriptor const& ed, vertex_iterator& vi,
                       adj_edge_iterator& ei)
  {
    vi = find_vertex(ed.source());
    if (vi != this->end()) {
      //if the edge id is missing we look for target,
      //source and delete the first encountered
      if (ed.id() == (size_t)INVALID_VALUE) {
        ei = graph_find(vi.begin(), vi->remote_begin(),
                        eq_target<vertex_descriptor>(ed.target()));
      } else { //if the edge descriptor is complete
        //the edge id uniquely identifies an edge
        //!!!the next check is too strong for directed; it can be relaxed by
        //specializing for directed and undirected
        ei = graph_find(vi.begin(), vi->remote_begin(),
                        eq_ed<edge_descriptor>(ed));
        //ei = graph_find((*vi).begin(), (*vi).end(),
        //eq_eid<vertex_descriptor>(_ed.id()));
      }
      if (ei != vi->remote_begin())
        return true;
      else return false;
    }
    else return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds a local edge with the specified descriptor, and returns an
  /// iterator to its source vertex and an adj_edge_iterator pointing to the
  /// edge.
  /// @param ed Descriptor of the edge.
  /// @param vi vertex_iterator pointing to the source vertex of the edge,
  /// populated by the method.
  /// @param aei adj_edge_iterator pointing to the specified edge,
  /// populated by the method.
  /// @return Whether or not the edge was found.
  //////////////////////////////////////////////////////////////////////
  bool find_local_edge(edge_descriptor const& ed, const_vertex_iterator& vi,
                       const_adj_edge_iterator& ei) const
  {
    vertex_iterator   i_vi;
    adj_edge_iterator i_ei;
    bool res =
        const_cast<adjacency_list_model*>(this)->find_local_edge(ed,i_vi,i_ei);
    vi = const_vertex_iterator(i_vi);
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
  const_vertex_iterator find_vertex(vertex_descriptor& vd) const
  {
    return const_cast<adjacency_list_model*>(this)->find_vertex(vd);
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
    return const_cast<adjacency_list_model*>(this)->find_vertex(vd);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds the vertex with the specified descriptor, and returns a
  /// vertex_iterator pointing to it. If not found, the end of the graph is
  /// returned.
  /// @param vd Descriptor of the vertex.
  /// @return A vertex_iterator to the specified vertex, if found, or a
  /// vertex_iterator to the end of the graph otherwise.
  //////////////////////////////////////////////////////////////////////
  vertex_iterator find_vertex(vertex_descriptor& vd)
  {
    return this->m_vertices.find_vertex(vd);
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
    return this->m_vertices.find_vertex(vd);
  }

  //////////////////////////////////////////////////////////////////////
  /// Same as @ref find_vertex, provided for compatibility.
  //////////////////////////////////////////////////////////////////////
  vertex_iterator find(vertex_descriptor const& vd)
  {
    return this->m_vertices.find_vertex(vd);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Updates given descriptor's version info, provided the iterator
  /// is valid. If iterator is not valid, it is updated with a call to find().
  /// @param vd Descriptor of the edge.
  /// @param vi A vertex_iterator pointing to the specified vertex.
  //////////////////////////////////////////////////////////////////////
  void update_descriptor(vertex_descriptor& vd,
                         vertex_iterator& vi)
  {
    this->m_vertices.update_descriptor(vd, vi);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sorts edges of each vertex according to provided comparator.
  /// @param comp The comparator for sorting edges of a vertex.
  //////////////////////////////////////////////////////////////////////
  template<typename Comp>
  void sort_edges(Comp const& comp)
  {
    typename vertex_set_type::iterator it = this->m_vertices.begin();
    for (; it != this->m_vertices.end(); ++it)
      std::sort(it->begin(), it->end(), comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes all duplicate edges with the same (source, target) pair
  //////////////////////////////////////////////////////////////////////
  void remove_duplicate_edges()
  {
    for (auto& v : *this) {
      auto begin = v.begin();
      std::sort(begin, v.end(), detail::edge_target_comp{});
      auto new_end = std::unique(begin, v.end(), detail::edge_target_equal{});
      v.erase(new_end-begin);
    }
  }

#ifdef _STAPL
  void define_type(typer& t)
  {
    t.member(m_vertices);
    t.member(m_ne);
    t.member(m_se);
  }

  size_t memory_size(void) const
  {
    size_t sz=0;
    sz+=sizeof(m_ne) + sizeof(m_se) + sizeof(m_eid) + m_vertices.memory_size();
    return sz;
  }
#endif

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the specified edge to the vertex pointed to by the provided
  /// iterator.
  //////////////////////////////////////////////////////////////////////
  void add_internal_edge(vertex_iterator vi, edge_type const& edge)
  {
    vi->edgelist().add( edge );
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
  friend void add_internal_edge(adjacency_list_model& g,
                                edge_descriptor const& ed,
                                edge_property const& p)
  {
    vertex_iterator vi = g.find_vertex(ed.source());
    if (vi == g.end())
      std::cout<<"ERROR while adding internal edge\n";

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
  /// @param ed The descriptor of the edge to be added. Id of the descriptor
  /// must have been externally assigned.
  /// @param p The property of the edge being added.
  //////////////////////////////////////////////////////////////////////
  friend edge_descriptor add_edge_pair(adjacency_list_model& g,
                                       edge_descriptor const& ed,
                                       edge_property const& p)
  {
    edge_descriptor n_ed;
    bool reversed = false;
    if (ed.source() > ed.target()) {
      n_ed = reverse(ed);
      reversed = true;
    } else {
      n_ed = ed;
    }
    // this is done for versioning, to prevent adding edges
    // between invalid iterators from another graph.
    simple_vertex_descriptor n_ed_source = n_ed.source();
    simple_vertex_descriptor n_ed_target = n_ed.target();

    vertex_iterator vi1 = g.find_vertex(n_ed_source);
    vertex_iterator vi2 = g.find_vertex(n_ed_target);

    vertex_descriptor v1, v2;
    if (vi1 == g.end())
      v1 = vertex_descriptor(INVALID_VALUE);
    else
      v1 = ed.source();
    if (vi2 == g.end())
      v2 = vertex_descriptor(INVALID_VALUE);
    else
      v2 = ed.target();
    if (vi1 != g.end() && vi2 != g.end()) {
      size_t eid = g.m_eid++;
      edge_descriptor ied(n_ed_source, n_ed_target, eid);
      edge_type edge(ied, p);
      g.add_internal_edge(vi1, edge); // it has to be like this
      //because sibling edges have to share same property
      g.add_internal_edge(vi2,reverse(edge) );
      if (ed.target() == ed.source()) {
        g.m_se += 1;
      } else {
        g.m_ne += 2;
      }
      return ied;
    }
    // error was encountered
    if (!reversed)
      return edge_descriptor(v1, v2, INVALID_VALUE);
    else
      return edge_descriptor(v2, v1, INVALID_VALUE);
  }

}; // class adjacency_list_model

template<class VD, class Property, class AdjList>
std::ostream& operator<<(std::ostream& os,
                         vertex_adj_list_impl<VD, Property, AdjList> const& v)
{
  os << v.descriptor();
  return os;
}

} // namespace stapl

#endif
