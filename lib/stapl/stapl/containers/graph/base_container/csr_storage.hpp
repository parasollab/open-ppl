/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_BASE_CONTAINER_CSR_STORAGE_HPP
#define STAPL_CONTAINERS_GRAPH_BASE_CONTAINER_CSR_STORAGE_HPP

#include <stapl/containers/sequential/graph/graph_util.h>
#include <stapl/containers/sequential/graph/graph_iterator.h>
#include <stapl/containers/sequential/graph/adj_list_vertex_edge.h>
#include <stapl/containers/sequential/graph/vdg_intVD.h>
#include <stapl/runtime/serialization.hpp>

#include <stapl/utility/for_each_range.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Perform an exclusive prefix sum of the input range and store
///        it in the output range, with the first value being an initial
///        value
/// @todo Replace with std::exclusive_scan when using C++17.
//////////////////////////////////////////////////////////////////////
template<typename InputIt, typename OutputIt, typename T>
OutputIt exclusive_scan(InputIt first, InputIt last, OutputIt out, T init)
{
  while (first != last)
  {
    T tmp = *first++;
    *out++ = init;
    init += tmp;
  }

  return out;
}

//to be used in friend declarations
template<class Traits> class csr_model;

//////////////////////////////////////////////////////////////////////
/// @brief Helper class to compare two edges based on their source
/// vertex-descriptors.
/// @ingroup pgraphBaseCont
//////////////////////////////////////////////////////////////////////
struct edge_source_comp
{
  template <class Edge>
  bool operator()(Edge const& e1, Edge const& e2)
  {
    return e1.source() < e2.source();
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper class to compare two edges based on their target
/// vertex-descriptors.
/// @ingroup pgraphBaseCont
//////////////////////////////////////////////////////////////////////
struct edge_target_comp
{
  template <class Edge>
  bool operator()(Edge const& e1, Edge const& e2)
  {
    return e1.target() < e2.target();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper class to equate two edges based on their target
/// vertex-descriptors.
/// @ingroup pgraphBaseCont
//////////////////////////////////////////////////////////////////////
struct edge_target_equal
{
  template <class Edge>
  bool operator()(Edge const& e1, Edge const& e2)
  {
    return e1.target() == e2.target();
  }
};

struct edge_source_equal
{
  template <class Edge>
  bool operator()(Edge const& e1, Edge const& e2)
  {
    return e1.source() == e2.source();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex for a Compressed Sparse-Row (CSR) graph.
/// @ingroup pgraphBaseCont
/// @tparam VD Vertex descriptor type for the graph.
/// @tparam Property Property type of the vertex.
/// @tparam AdjList Type of the edgelist for storing edges.
//////////////////////////////////////////////////////////////////////
template<class VD, class Property, class AdjList>
class vertex_csr_impl
{
  template<class Traits> friend class csr_model;
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

protected:
  /// Iterator to the beginning of this vertex's edgelist.
  edgelist_it    m_begin;
  /// Property of this vertex.
  property_type  m_property;

public:
  vertex_csr_impl() = default;
  vertex_csr_impl(vertex_csr_impl const& other) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a vertex for the graph with the given descriptor
  /// and property.
  /// @param vd Provides the descriptor of this vertex.
  /// @param p Provides the property of this vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_csr_impl(vertex_descriptor vd, Property const& p)
    : m_property(p)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a copy of the outgoing edges.
  //////////////////////////////////////////////////////////////////////
  edgelist_type edges()
  { return edgelist_type(this->m_begin, this->end());  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the beginning and the end of the edgelist for this vertex.
  /// @param begin An iterator pointing to the start of the edgelist.
  //////////////////////////////////////////////////////////////////////
  void set_edges(edgelist_it begin)
  {
    m_begin = begin;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the beginning of the edgelist of this vertex.
  //////////////////////////////////////////////////////////////////////
  edgelist_it begin()
  { return this->m_begin; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the end of the edgelist of this vertex.
  //////////////////////////////////////////////////////////////////////
  edgelist_it end()
  {
    // Since we know that the end of our edge list is the begin of the
    // "neighbor" in the CSR array, we can use their begin.
    return (this + 1)->begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the beginning of the edgelist of this vertex.
  //////////////////////////////////////////////////////////////////////
  const_edgelist_it begin() const
  { return this->m_begin; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the end of the edgelist of this vertex.
  //////////////////////////////////////////////////////////////////////
  const_edgelist_it end() const
  {
    return (this + 1)->begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the edge with the specified target.
  /// @param vd The descriptor of the target vertex.
  /// @todo This class does not have an m_edgelist, needs fixing
  //////////////////////////////////////////////////////////////////////
  edgelist_it find_edge(vertex_descriptor const& vd)
  {
    return std::find_if(
      this->begin(), this->end(), edge_eq_target<vertex_descriptor>{vd}
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the specified edge.
  /// @param ed The descriptor of the edge.
  /// @todo This class does not have an m_edgelist, needs fixing
  //////////////////////////////////////////////////////////////////////
  edgelist_it find_edge(edge_descriptor const& ed)
  {
    return std::find_if(
      this->begin(), this->end(), edge_eq_target<vertex_descriptor>{ed.target()}
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of outgoing edges of this vertex.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  { return std::distance(this->begin(), this->end()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the property of this vertex.
  //////////////////////////////////////////////////////////////////////
  property_reference property()
  { return m_property; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the property of this vertex.
  //////////////////////////////////////////////////////////////////////
  const_property_reference property() const
  { return m_property; }

  void define_type(typer& t)
  {
    t.member(m_begin);
    t.member(m_property);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref vertex_csr_impl.
/// @ingroup pgraphdistObj
/// @tparam VD Vertex descriptor type for the graph.
/// @tparam Property Property type of the vertex.
/// @tparam AdjList Type of the edgelist for storing edges.
/// @tparam Accessor Type of the accessor for the proxy.
//////////////////////////////////////////////////////////////////////
template<class VD, class Property, class AdjList, typename Accessor>
class proxy<vertex_csr_impl<VD, Property, AdjList>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef vertex_csr_impl<VD, Property, AdjList> target_t;
  typedef typename target_t::adj_edge_iterator        adj_edge_iter_t;
  typedef typename target_t::const_adj_edge_iterator  const_adj_edge_iter_t;

public:
  typedef typename target_t::property_type            property_type;
  typedef typename Accessor::property_reference       property_reference;
  typedef typename Accessor::const_property_reference const_property_reference;
  typedef typename target_t::vertex_descriptor        vertex_descriptor;
  typedef typename target_t::edge_descriptor          edge_descriptor;
  typedef typename target_t::adj_edges_type           adj_edges_type;

  typedef member_iterator<adj_edge_iter_t, Accessor>  adj_edge_iterator;
  typedef member_iterator<const_adj_edge_iter_t,
                          Accessor>                   const_adj_edge_iterator;

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
  operator target_t() const
  { return Accessor::read(); }

  vertex_descriptor descriptor() const
  { return Accessor::index(); }

  property_reference property()
  { return Accessor::property(); }

  const_property_reference property() const
  { return Accessor::property(); }

  adj_edge_iterator begin()
  { return adj_edge_iterator(Accessor::invoke(&target_t::begin), *this); }

  adj_edge_iterator end()
  { return adj_edge_iterator(Accessor::invoke(&target_t::end), *this); }

  adj_edge_iterator find_edge(vertex_descriptor const& vd)
  { return adj_edge_iterator(Accessor::invoke(&target_t::find, vd), *this); }

  adj_edge_iterator find_edge(edge_descriptor const& ed)
  { return adj_edge_iterator(Accessor::invoke(&target_t::find, ed), *this); }

  const_adj_edge_iterator begin() const
  {
    return const_adj_edge_iterator(Accessor::const_invoke(&target_t::begin),
                                   *this);
  }

  const_adj_edge_iterator end() const
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

  size_t size(void) const
  {
    return Accessor::const_invoke(&target_t::size);
  }

  adj_edges_type edges() const
  { return Accessor::ref().edges();}
}; //struct proxy


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref vertex_csr_impl,
/// specialized for @ref local_accessor_graph, which lets us use a
/// faster iterator over the edges than the @ref member_iterator
/// needed for the @ref container_accessor.
/// @ingroup pgraphdistObj
/// @tparam VD Vertex descriptor type for the graph.
/// @tparam Property Property type of the vertex.
/// @tparam AdjList Type of the edgelist for storing edges.
/// @tparam Accessor Type of the accessor for the proxy.
//////////////////////////////////////////////////////////////////////
template<class VD, class Property, class AdjList, typename Container>
class proxy<vertex_csr_impl<VD, Property, AdjList>,
            local_accessor_graph<Container> >
  : public local_accessor_graph<Container>
{
private:
  typedef local_accessor_graph<Container> accessor_t;
  friend class proxy_core_access;
  typedef vertex_csr_impl<VD, Property, AdjList> target_t;
public:
  typedef typename target_t::property_type            property_type;
  typedef typename accessor_t::property_reference     property_reference;
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
  operator target_t() const
  { return accessor_t::read(); }

  vertex_descriptor descriptor() const
  { return accessor_t::index(); }

  property_reference property()
  { return accessor_t::property(); }

  adj_edge_iterator begin()
  { return accessor_t::ref().begin(); }

  adj_edge_iterator end()
  { return accessor_t::ref().end(); }

  const_adj_edge_iterator begin() const
  { return accessor_t::ref().begin(); }

  const_adj_edge_iterator end() const
  { return accessor_t::ref().end(); }

  const_adj_edge_iterator find_edge(vertex_descriptor const& vd) const
  { return accessor_t::ref().find(vd); }

  const_adj_edge_iterator find_edge(edge_descriptor const& ed) const
  { return accessor_t::ref().find(ed); }

  size_t size(void) const
  { return accessor_t::ref().size();}

  adj_edges_type edges() const
  { return accessor_t::ref().edges();}
}; //struct proxy


//////////////////////////////////////////////////////////////////////
/// @brief The edgelist for a CSR graph, implemented as a vector of edges.
/// @ingroup pgraphBaseCont
/// @tparam Edge The type of edge to be stored.
//////////////////////////////////////////////////////////////////////
template <class Edge>
class csr_edgelist_impl
{
  /// The actual list of edges is stored in an std::vector.
  std::vector<Edge> m_data;

 public:
  //infer property
  typedef Edge     edge_type;
  typedef typename Edge::vertex_descriptor           vertex_descriptor;
  typedef typename Edge::edge_descriptor_type        edge_descriptor;
  typedef typename std::vector<Edge>::iterator       iterator;
  typedef typename std::vector<Edge>::const_iterator const_iterator;
  typedef typename std::iterator_traits<iterator>::reference  reference;
  typedef typename std::iterator_traits<iterator>::value_type value_type;

  //////////////////////////////////////////////////////////////////////
  /// Create a csr edgelist with specified number of edges.
  //////////////////////////////////////////////////////////////////////
  csr_edgelist_impl(size_t n=0)
    : m_data(n)
  { }

  csr_edgelist_impl(csr_edgelist_impl const&) = default;
  csr_edgelist_impl& operator=(csr_edgelist_impl const&) = default;
  csr_edgelist_impl& operator=(csr_edgelist_impl&&) = default;

  csr_edgelist_impl(iterator begin, iterator end)
  : m_data(begin, end)
  { }

  iterator begin()
  { return m_data.begin(); }

  const_iterator begin() const
  { return m_data.begin(); }

  iterator end()
  { return m_data.end(); }

  const_iterator end() const
  { return m_data.end(); }

  size_t size() const
  { return m_data.size(); }

  void define_type(typer& t)
  { t.member(m_data); }

  //////////////////////////////////////////////////////////////////////
  /// Adds an edge to the front of edgelist.
  //////////////////////////////////////////////////////////////////////
  void add_dummy()
  { m_data.push_front(Edge()); }

  /// Clears the edgelist.
  void clear()
  {
    m_data.clear();
    m_data.shrink_to_fit();
  }

  size_t memory_size(void) const
  { return memory_used<std::vector<Edge> >::size(m_data); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the specified edge to the edgelist.
  /// @param ed The edge to be inserted into the edgelist.
  //////////////////////////////////////////////////////////////////////
  void add(Edge const& ed)
  { m_data.push_back(ed); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the specified edge to the edgelist.
  /// @note Needed for compatibility with std::back_inserter.
  /// @param ed The edge to be inserted into the edgelist.
  //////////////////////////////////////////////////////////////////////
  void push_back(Edge const& ed)
  { m_data.push_back(ed); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Emplaces an edge into the edgelist
  /// @param args... The set of arguments to construct the edge
  //////////////////////////////////////////////////////////////////////
  template<typename... Args>
  void emplace_back(Args&&... args)
  {
    m_data.emplace_back(std::forward<Args>(args)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erases the edges in an iterator range
  /// @param begin An iterator pointing to the begin of the range
  /// @param end An iterator pointing to the end of the range
  //////////////////////////////////////////////////////////////////////
  void erase(iterator begin, iterator end)
  { m_data.erase(begin, end); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Resize the edge list
  /// @param n The new size
  //////////////////////////////////////////////////////////////////////
  void resize(std::size_t n)
  { m_data.resize(n); }
};


//////////////////////////////////////////////////////////////////////
/// @brief An CSR using an std::vector for storing vertices.
/// Used inside the @ref csr_model to store vertices.
/// @ingroup pgraphBaseCont
/// @tparam Traits The traits class for specifying the types of descriptors,
/// storages, edges and vertices.
//////////////////////////////////////////////////////////////////////
template <class Traits>
class csr_vector_storage
{
 public:
  typedef typename Traits::vertex_descriptor        vertex_descriptor;
  using edge_property = typename Traits::edge_property;
  typedef typename Traits::vertex_property          vertex_property;
  typedef typename Traits::full_edge_type           full_edge_type;

  using uncommitted_edgelist_type = csr_edgelist_impl<full_edge_type>;
  using uncommitted_edge_iterator =
    typename uncommitted_edgelist_type::iterator;

  using edge_type = typename Traits::edge_type;
  using edge_descriptor_type = typename edge_type::edge_descriptor_type;
  using vertex_impl_type = typename Traits::vertex_impl_type;
  using edgelist_type = typename Traits::edgelist_type;

  /// Type of the vertex descriptor generator.
  typedef sequential::vdg_base_int<vertex_descriptor> vdg_type;

  /// The vertices are stored in an std::vector.
  typedef std::vector<vertex_impl_type>             vertex_set_type;

  static constexpr graph_attributes m_type = Traits::m_type;

 protected:
  // Data members.
  /// The vertex descriptor generator.
  vdg_type         m_vdg;

  /// The container for storing vertices.
  vertex_set_type  m_storage;

  /// The container for storing uncommitted edges.
  uncommitted_edgelist_type    m_uncommitted_edges;

  /// The container for storing edges.
  edgelist_type    m_edgelist;

  /// The descriptor of the starting vertex in this storage.
  vertex_descriptor my_local_start_vd;

  //////////////////////////////////////////////////////////////////////
  /// @brief The flag indicates if the CSR is frozen. Edges may only be added
  /// to the CSR as long as it is unfrozen. Once it has been frozen by calling
  /// the commit() method, no further addition of edges is allowed.
  /// The CSR graph starts off in an unfrozen state, and may only be committed
  /// once, and may not be uncommitted thereafter.
  //////////////////////////////////////////////////////////////////////
  bool             m_committed;

  /// The number of edges in the graph.
  size_t m_ne;

  /// The number of self edges in the graph.
  size_t m_se;

 public:
  typedef typename vertex_set_type::iterator       iterator;
  typedef typename vertex_set_type::const_iterator const_iterator;
  typedef typename edgelist_type::iterator         edge_iterator;
  typedef typename edgelist_type::const_iterator   const_edge_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a storage with the given number of vertices starting
  /// from the specified descriptor, with the given default value.
  /// Vertices have contiguous descriptors.
  /// @param start_vd The starting descriptor of the vertices.
  /// @param num_vds The number of vertices to be initially stored.
  /// @param default_value The default value for the vertices.
  //////////////////////////////////////////////////////////////////////
  csr_vector_storage(size_t const& start_vd, size_t const& num_vds,
                     vertex_impl_type const& default_value)
    : m_vdg(start_vd), my_local_start_vd(start_vd), m_committed(false),
      m_ne(0), m_se(0)
  {
    //init the vertex descriptors and adj lists
    m_storage.reserve(num_vds+1);
    for (size_t i=0; i<num_vds+1; ++i) {
      vertex_descriptor vd = m_vdg.next();
      m_storage.push_back(vertex_impl_type(vd, default_value.property()));
    }
  }

  csr_vector_storage(csr_vector_storage const& other)
    : m_vdg(other.m_vdg),
      m_storage(other.m_storage), m_edgelist(other.m_edgelist),
      my_local_start_vd(get_location_id()*m_storage.size()),
      m_committed(other.m_committed),
      m_ne(other.m_ne), m_se(other.m_se)
  { }

  //////////////////////////////////////////////////////////////////////
  /// Resize the internal storage to accommodate specified number of vertices.
  //////////////////////////////////////////////////////////////////////
  void resize(size_t nv)
  {
    this->m_storage.clear();
    vertex_property p = vertex_property(); // default constructed property;
    for (size_t i=0; i<nv+1; ++i)
    {
      vertex_descriptor vd = m_vdg.next();
      m_storage.push_back(vertex_impl_type(vd, p));
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the start of the vertices.
  //////////////////////////////////////////////////////////////////////
  iterator begin()
  { return this->m_storage.begin(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the end of the vertices.
  //////////////////////////////////////////////////////////////////////
  iterator end()
  { return std::prev(this->m_storage.end()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the start of the vertices.
  //////////////////////////////////////////////////////////////////////
  const_iterator begin() const
  { return this->m_storage.begin(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the end of the vertices.
  //////////////////////////////////////////////////////////////////////
  const_iterator end() const
  { return std::prev(this->m_storage.end()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an edge_iterator to the start of the edges.
  //////////////////////////////////////////////////////////////////////
  edge_iterator edges_begin()
  { return this->m_edgelist.begin(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an edge_iterator to the end of the edges.
  //////////////////////////////////////////////////////////////////////
  edge_iterator edges_end()
  { return this->m_edgelist.end(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an edge_iterator to the begin of the uncommitted edges.
  //////////////////////////////////////////////////////////////////////
  uncommitted_edge_iterator uncommitted_edges_begin()
  { return this->m_uncommitted_edges.begin(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an edge_iterator to the end of the uncommitted edges.
  //////////////////////////////////////////////////////////////////////
  uncommitted_edge_iterator uncommitted_edges_end()
  { return this->m_uncommitted_edges.end(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an edge_iterator to the start of the edges.
  //////////////////////////////////////////////////////////////////////
  const_edge_iterator edges_begin() const
  { return this->m_edgelist.begin(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an edge_iterator to the end of the edges.
  //////////////////////////////////////////////////////////////////////
  const_edge_iterator edges_end() const
  { return this->m_edgelist.end(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of vertices in the storage; for use
  /// in g.num_vertices().
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  { return this->m_storage.size()-1; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge with given descriptor and default edge property.
  /// @param ed Descriptor of the desired edge.
  //////////////////////////////////////////////////////////////////////
  void add_edge(edge_descriptor_type const& ed, edge_property const& ep)
  {
    stapl_assert(!m_committed,
      "Cannot add an edge to a committed graph. Invoke uncommit first.");
    this->m_uncommitted_edges.add(full_edge_type{ed, ep});
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge from a full edge.
  /// @param e The edge to add.
  //////////////////////////////////////////////////////////////////////
  void add_edge(full_edge_type const& e)
  {
    stapl_assert(!m_committed,
      "Cannot add an edge to a committed graph. Invoke uncommit first.");
    this->m_uncommitted_edges.add(e);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erases all vertices and edges in the graph.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->m_storage.clear();
    this->m_edgelist.clear();
    this->m_uncommitted_edges.clear();
    this->m_vdg.reset();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds the vertex with the specified descriptor, and returns an
  /// iterator pointing to it. If not found, the end of the graph is
  /// returned.
  /// @param vd Descriptor of the vertex.
  /// @return An iterator to the specified vertex, if found, or an
  /// iterator to the end of the graph otherwise.
  //////////////////////////////////////////////////////////////////////
  iterator find_vertex(vertex_descriptor const& vd)
  {
    return m_storage.begin() + (vd - my_local_start_vd);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Not used by CSR graph, provided for compatibility.
  //////////////////////////////////////////////////////////////////////
  void update_descriptor(vertex_descriptor const&, iterator const&)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Updates the vertex descriptor generator with the next free
  ///        descriptor
  //////////////////////////////////////////////////////////////////////
  void update_next_descriptor(vertex_descriptor const& vd)
  {
    this->m_vdg.update(vd);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the current max descriptor.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor get_max_descriptor()
  { return m_vdg.curr_vd(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Freezes the CSR graph so that no further addition of edges is
  /// possible.
  /// @note This method must be called before using the CSR graph, and only
  /// after all edges have been added to it.
  /// Edges may only be added to the CSR as long as it is unfrozen. Once it
  /// has been frozen by calling commit(), no further addition of edges is
  /// allowed. The CSR may only be committed once, and may not be uncommitted
  /// thereafter.
  ///
  /// The CSR graph starts off in an unfrozen state. Freezing it internally
  /// sorts and copies the edges to the CSR format and updates the vertices
  /// accordingly to point to their adjacent sections in the edgelist.
  ///
  /// @return A pair of the number of edges and the number of self edges
  /// after committing
  //////////////////////////////////////////////////////////////////////
  std::pair<std::size_t, std::size_t> commit()
  {
    // Only do this if graph is not already committed.
    if (m_committed)
      return std::make_pair(m_ne, m_se);

    // Sort the edges first to make sure correct ordering for CSR.
    this->sort_uncommitted_edges_by_source();

    // Remove multi-edges if a non-multiedges graph
    if (m_type == stapl::NONMULTIEDGES)
      this->squish_uncommitted_edges();

    m_ne = std::count_if(
      this->uncommitted_edges_begin(),
      this->uncommitted_edges_end(),
      [](full_edge_type const& e) { return e.source() != e.target(); }
    );
    m_se = std::count_if(
      this->uncommitted_edges_begin(),
      this->uncommitted_edges_end(),
      [](full_edge_type const& e) { return e.source() == e.target(); }
    );

    const std::size_t num_edges = std::distance(
      this->uncommitted_edges_begin(), this->uncommitted_edges_end()
    );
    this->m_edgelist.resize(num_edges);
    auto next_begin = this->m_edgelist.begin();

    // Set all vertices' edge pointers to be invalid
    for (auto& v : *this)
      v.set_edges(edge_iterator{});

    // Lambda to insert short edges into the edge list and update vertices
    auto copier = [this, &next_begin](uncommitted_edge_iterator b,
                                      uncommitted_edge_iterator e) {
      auto& v = *this->find_vertex(b->source());

      // For each full edge, transform it to a short edge if necessary
      std::transform(b, e, next_begin, [](full_edge_type const& full) {
        return edge_type{edge_descriptor_type{
          full.source(), full.target(), full.id()}, full.property()
        };
      });

      v.set_edges(next_begin);

      const std::size_t degree = std::distance(b, e);
      std::advance(next_begin, degree);
    };

    // For each uncommitted edge, copy it to the committed edge list and set
    // the source vertex's iterators
    utility::for_each_range(
      this->uncommitted_edges_begin(), this->uncommitted_edges_end(),
      copier, edge_source_equal{}
    );

    // Set the last vertex's edge list
    std::prev(this->m_storage.end())->set_edges(next_begin);

    // Run through the vertices backwards and cleanup any of them that had
    // no edges. If a vertex was not set (default initialized), then set its
    // begin to be the next vertex's begin.
    auto invalid_begin = edge_iterator{};
    for (auto it = this->end(); it-- != this->begin();) {
      if (it->begin() == invalid_begin)
        it->set_edges(std::next(it)->begin());
    }

    stapl_assert(std::prev(this->end())->end() == this->edges_end(),
      "Last vertex's end iterator should be last edge in the edge list.");

    m_committed = true;  // can only commit once
    m_uncommitted_edges.clear();
    return std::make_pair(m_ne, m_se);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Uncommit the CSR graph.
  ///
  /// Place the graph in a state so that edges can be added. While the graph
  /// is uncommited, it is not possible to access its edges.
  //////////////////////////////////////////////////////////////////////
  void uncommit()
  {
    stapl_assert(m_committed,
      "Trying to uncommit a graph that hasn't been committed");
    stapl_assert(m_uncommitted_edges.size() == 0,
      "There should be no uncommitted edges");

    m_committed = false;

    for (vertex_descriptor i = 0; i < this->size(); ++i) {
      auto const& v = m_storage[i];

      for (auto const& e : v) {
        const edge_descriptor_type ed{my_local_start_vd+i, e.target()};
        this->add_edge(full_edge_type{ed, e.property()});
      }
    }

    this->m_edgelist.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sorts edges of each vertex by user-defined comparison function.
  /// @note The graph must be committed in order to sort its edges
  //////////////////////////////////////////////////////////////////////
  template<typename Comp>
  void sort_edges(Comp&& cmp)
  {
    stapl_assert(m_committed, "Graph must be committed to sort its edges");

    for (auto& v : *this)
      std::sort(v.begin(), v.end(), std::forward<Comp>(cmp));

  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erases all edges that match a user-defined predicate
  /// @note The graph must be not committed
  /// @param pred A unary predicate that receives a single edge
  //////////////////////////////////////////////////////////////////////
  template<typename Pred>
  void erase_edges_if(Pred&& pred)
  {
    stapl_assert(!m_committed, "Graph must not be committed to erase edges");

    auto it = std::remove_if(
      this->uncommitted_edges_begin(), this->uncommitted_edges_end(),
      std::forward<Pred>(pred)
    );

    m_uncommitted_edges.erase(it, m_uncommitted_edges.end());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes all duplicate edges with the same (source, target) pair
  /// @note The graph must be not committed
  //////////////////////////////////////////////////////////////////////
  void remove_duplicate_edges()
  {
    stapl_assert(!m_committed,
      "Graph must not be committed to remove duplicate edges");
    this->squish_uncommitted_edges();
  }

  void define_type(typer& t)
  {
    t.member(m_storage);
    t.member(m_edgelist);
    t.member(m_uncommitted_edges);
    t.member(m_vdg);
    t.member(m_committed);
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Sort the uncommitted edges by their source. Uses a counting
  ///        sort, since the source values are guaranteed to be integral
  ///        types in a known range.
  //////////////////////////////////////////////////////////////////////
  void sort_uncommitted_edges_by_source()
  {
    const std::size_t num_vertices = m_storage.size();
    const std::size_t num_edges = std::distance(
      this->uncommitted_edges_begin(), this->uncommitted_edges_end()
    );


    // Compute each vertex's degree
    uncommitted_edgelist_type tmp_edges(num_edges);
    std::vector<std::size_t> degrees(num_vertices);

    for (auto const& e: m_uncommitted_edges) {
      degrees[e.source() - my_local_start_vd] += 1;
    }

    // Perform an exclusive scan to get the write offset for each vertex
    exclusive_scan(degrees.begin(), degrees.end(), degrees.begin(), 0ul);

    // Add the uncommitted edges to the appropriate offset
    for (auto const& e: m_uncommitted_edges) {
      const auto local_source = e.source() - my_local_start_vd;
      auto offset_it = tmp_edges.begin() + (degrees[local_source]++);
      *offset_it = e;
    }

    m_uncommitted_edges = std::move(tmp_edges);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove duplicate edges.
  //////////////////////////////////////////////////////////////////////
  void squish_uncommitted_edges()
  {
    uncommitted_edgelist_type squished_edgelist;
    auto insert_it = std::back_inserter(squished_edgelist);

    using edge_iterator_type = decltype(this->uncommitted_edges_begin());

    auto copier = [&](edge_iterator_type b, edge_iterator_type e) {
      // Sort edges based on target to get better aggregation when visiting
      // neighbors.
      std::sort(b, e, edge_target_comp{});

      // Insert only the unique edges into the squished list
      auto unique_end = std::unique(b, e, edge_target_equal{});

      std::copy(b, unique_end, insert_it);

    };

    // For each range of values that belong to the same source, only insert
    // the unique vertices into the squished list
    utility::for_each_range(
      this->uncommitted_edges_begin(), this->uncommitted_edges_end(),
      copier, edge_source_equal{}
    );

    m_uncommitted_edges = std::move(squished_edgelist);
  }
}; // csr_vector_storage.


//////////////////////////////////////////////////////////////////////
/// @brief This model describes operations on a graph that
/// is modeled as a Compressed Sparse-Row (CSR) graph. CSRs store a list
/// of vertices and a separate list of edges. Each vertex points to its adjacent
/// edges in the list of edges.
/// @ingroup pgraphBaseCont
/// @tparam Traits The traits class for specifying the types of descriptors,
/// storages, edges and vertices, etc.
///
/// Methods like find_vertex, find_edge are optimized for this particular
/// storage.
//////////////////////////////////////////////////////////////////////
template <class Traits>
class csr_model
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

  /// The number of edges in this CSR.
  size_t m_ne;

  /// The number of self edges in this CSR.
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
  typedef sequential::edge_iterator_adaptor<vertex_iterator>    edge_iterator;
  typedef sequential::edge_iterator_adaptor<const_vertex_iterator>
                                                          const_edge_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a CSR with the given number of vertices
  /// starting from the specified descriptor, with the given default value.
  /// Vertices have contiguous descriptors.
  /// @param start_vd The starting descriptor of the vertices.
  /// @param num_vds The number of vertices to be initially stored.
  /// @param default_value The default value for the vertices.
  //////////////////////////////////////////////////////////////////////
  csr_model(size_t const& start_vd, size_t const& num_vds,
            value_type const& default_value = value_type())
    : m_vertices(start_vd, num_vds, default_value), m_ne(0), m_se(0), m_eid(0)
  { }

  csr_model(csr_model const& other)
    : m_vertices(other.m_vertices), m_ne(other.m_ne), m_se(other.m_se),
      m_eid(other.m_eid)
  { }

  vertex_iterator begin()
  { return this->m_vertices.begin(); }

  vertex_iterator end()
  { return this->m_vertices.end(); }

  const_vertex_iterator begin() const
  { return this->m_vertices.begin(); }

  const_vertex_iterator end() const
  { return this->m_vertices.end(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the total number of vertices.
  //////////////////////////////////////////////////////////////////////
  size_t num_vertices() const
  {
    return this->m_vertices.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the total number of vertices.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    return this->m_vertices.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the current max descriptor.
  //////////////////////////////////////////////////////////////////////
  size_t get_max_descriptor()
  {
    return this->m_vertices.get_max_descriptor();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the total number of edges.
  //////////////////////////////////////////////////////////////////////
  size_t num_edges() const
  {
    return m_ne;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the total number of self edges.
  //////////////////////////////////////////////////////////////////////
  size_t num_self_edges() const
  {
    return m_se;
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
    return add_edge(ed, edge_property());
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
        this->m_vertices.add_edge(ed, p);
        ned = ed;
      } else {
        //we allocate a new edge id by incrementing a local counter
        size_t id = m_eid++;
        ned = edge_descriptor(ed.source(), ed.target(), id);
        //edgelist is a method of the vertex
        this->m_vertices.add_edge(ed, p);
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
  /// @brief Unconditionally adds the edge. Duplicate edges will be
  ///        handled in the commit phase, if required.
  ///
  /// @param ed Descriptor of the desired edge.
  /// @param ep Property of the edge.
  /// @return edge_descriptor of the added edge.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor check_add_edge(edge_descriptor const& ed,
                                 edge_property const& ep)
  {
    return add_edge(ed, ep);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Unconditionally adds the edge. Duplicate edges will be
  ///        handled in the commit phase, if required.
  ///
  /// @param ed Descriptor of the desired edge.
  /// @return edge_descriptor of the added edge.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor check_add_edge(edge_descriptor const& ed)
  {
    return check_add_edge(ed, edge_property());
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
  /// @brief Freezes the CSR graph so that no further addition of edges is
  /// possible.
  /// @note This method must be called before using the CSR graph, and only
  /// after all edges have been added to it.
  /// Edges may only be added to the CSR as long as it is unfrozen. Once it
  /// has been frozen by calling commit(), no further addition of edges is
  /// allowed. The CSR may only be committed once, and may not be uncommitted
  /// thereafter.
  ///
  /// The CSR graph starts off in an unfrozen state. Freezing it internally
  /// sorts and copies the edges to the CSR format and updates the vertices
  /// accordingly to point to their adjacent sections in the edgelist.
  //////////////////////////////////////////////////////////////////////
  void commit(void)
  {
    std::tie(m_ne, m_se) = this->m_vertices.commit();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Uncommit the CSR graph.
  ///
  /// Place the graph in a state so that edges can be added. While the graph
  /// is uncommited, it is not possible to access its edges.
  //////////////////////////////////////////////////////////////////////
  void uncommit(void)
  {
    this->m_vertices.uncommit();
    m_ne = 0;
    m_se = 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sorts edges of each vertex by user-defined comparison function.
  /// @note The graph must be committed in order to sort its edges
  //////////////////////////////////////////////////////////////////////
  template<typename Comp>
  void sort_edges(Comp&& cmp)
  {
    m_vertices.sort_edges(std::forward<Comp>(cmp));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erases all edges that match a user-defined predicate
  /// @note The graph must be not committed
  /// @param pred A unary predicate that receives a single edge
  //////////////////////////////////////////////////////////////////////
  template<typename Pred>
  void erase_edges_if(Pred&& pred)
  {
    m_vertices.erase_edges_if(std::forward<Pred>(pred));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes all duplicate edges with the same (source, target) pair
  /// @note The graph must be not committed
  //////////////////////////////////////////////////////////////////////
  void remove_duplicate_edges()
  {
    m_vertices.remove_duplicate_edges();
  }

public:
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
    //if the edge id is missing we look for target, source
    // and delete the first encountered
    if (ed.id() == (size_t)INVALID_VALUE) {
      vi = this->find_vertex(ed.source());
      if (vi != this->end()) {
        ei = vi->find_edge(ed.target());
      }
    }
    else { //if edge descriptor is complete, edgeid uniquely identifies an edge
      ei = graph_find(this->m_vertices.edges_begin(),
                      this->m_vertices.edges_end(),
                      eq_ed<edge_descriptor>(ed));
    }

    if (ei != this->m_vertices.edges_end())
      return true;
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
    bool res = const_cast<csr_model*>(this)->find_edge(ed, i_vi, i_ei);
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
  const_vertex_iterator find_vertex(vertex_descriptor const& vd) const
  {
    return const_cast<csr_model*>(this)->find_vertex(vd);
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
  /// @brief Same as @ref find_vertex, provided for compatibility.
  //////////////////////////////////////////////////////////////////////
  vertex_iterator find(vertex_descriptor const& vd)
  {
    return this->m_vertices.find_vertex(vd);
  }

  void define_type(typer& t)
  {
    t.member(m_vertices);
    t.member(m_ne);
    t.member(m_se);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Updates the vertex descriptor generator with the next free
  ///        descriptor
  //////////////////////////////////////////////////////////////////////
  void update_next_descriptor(vertex_descriptor const& vd)
  {
    this->m_vertices.update_next_descriptor(vd);
  }

 private:

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
  friend edge_descriptor add_edge_pair(csr_model& g, edge_descriptor const& ed,
                                       edge_property const& p)
  {
    edge_descriptor n_ed;
    bool reversed = false;
    if (ed.source() > ed.target()) {
      n_ed = reverse(ed);
      reversed = true;
    }
    else n_ed = ed;
    // this is done for versioning, to prevent adding edges
    // between invalid iterators from another graph.
    simple_vertex_descriptor n_ed_source = n_ed.source();
    simple_vertex_descriptor n_ed_target = n_ed.target();

    vertex_iterator vi1 = g.find_vertex(n_ed_source);
    vertex_iterator vi2 = g.find_vertex(n_ed_target);

    vertex_descriptor v1, v2;
    if (vi1 == g.end()) v1 = vertex_descriptor(INVALID_VALUE);
    else v1 = ed.source();

    if (vi2 == g.end()) v2 = vertex_descriptor(INVALID_VALUE);
    else v2 = ed.target();

    if (vi1 != g.end() && vi2 != g.end()) {
      size_t eid = g.m_eid++;
      edge_descriptor ied(n_ed_source, n_ed_target, eid);
      edge_type edge(ied, p);
      if (n_ed_source == n_ed_target) {
        g.add_edge(edge);
        g.m_se += 1;
      } else {
        g.add_edge(edge);           // it has to be like this
        g.add_edge(reverse(edge) ); //because sibling edges share same property
        g.m_ne += 2;
      }
      return ied;
    }
    //here error encountered
    if (!reversed)
      return edge_descriptor(v1, v2, INVALID_VALUE);
    else
      return edge_descriptor(v2, v1, INVALID_VALUE);
  }

}; // class csr_model

} // namespace stapl
#endif
