/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_HASH_MAP_STORAGE_HPP
#define STAPL_CONTAINERS_GRAPH_HASH_MAP_STORAGE_HPP

#include <stapl/utility/hash_table/stapl_hash_map.h>
#include <stapl/containers/graph/base_container/graph_storage.h>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Class for generating vertex descriptors with a strided pattern.
/// @ingroup pgraphImpl
///
/// Used by the dynamic graph to add vertices when descriptors are not provided
/// by users.
///
/// @todo Remove m_use_next if it is not used.
//////////////////////////////////////////////////////////////////////
template <class VD>
class vdg_strided
{
 protected:
  /// The current vertex-descriptor.
  size_t m_curr_vd;
  /// The block-size for generating descriptors.
  size_t m_block;
  /// The number of partitions in the pGraph globally.
  size_t m_parts;
  /// The index of this object in the partitions.
  size_t m_index;
  bool m_use_next;
 public:
  typedef VD vertex_descriptor;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor to initialize the strided generator.
  /// @param start The descriptor to start the allocation from.
  /// @param block The block size for generating descriptors.
  /// @param parts The number of global partitions in the pGraph.
  /// @param index The index of this partition among the global partitions.
  /// @param next (Unused).
  //////////////////////////////////////////////////////////////////////
  vdg_strided(size_t start = std::numeric_limits<size_t>::max(),
              size_t block = 10,
              size_t parts = stapl::get_num_locations(),
              size_t index = stapl::get_location_id(), bool next = true)
    : m_curr_vd(start == std::numeric_limits<size_t>::max() ?
                block*index : start),
      m_block(block), m_parts(parts), m_index(index),
      m_use_next(next)
  { }

  //////////////////////////////////////////////////////////////////////
  /// Generates and returns the next vertex and updates internal version.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor next()
  {
    // stapl_assert(m_use_next, "Trying to use next()
    //in an explicit vertex descriptor generator.");
    vertex_descriptor vd = m_curr_vd;

    if (!(++m_curr_vd % m_block))
      m_curr_vd += m_block * (m_parts - 1);

    return vd;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Possibly reclaims descriptors for future use from deleted vertices.
  /// Provided for compatibility.
  //////////////////////////////////////////////////////////////////////
  void free(VD const&)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compares the version numbers to see if the iterator is valid.
  /// Provided for compatibility with versioning generators.
  //////////////////////////////////////////////////////////////////////
  bool is_valid(VD const&)
  { return false; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Manually increment the version number, if needed for some storage.
  /// Provided for compatibility with versioning generators.
  //////////////////////////////////////////////////////////////////////
  void increment()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Integrates the current descriptor into the generated sequence;
  /// future invocations of next() should not produce this VD.
  //////////////////////////////////////////////////////////////////////
  void update(VD const& vd)
  {

    if (vd < m_curr_vd)
     return;

    // attempt to just use the next vd
    m_curr_vd = vd+1;

    // figure out which global block this vd belongs to
    const std::size_t global_block_index = m_curr_vd / m_block;

    // compute the location id (col) of the block and how many
    // times it has wrapped around (row)
    const std::size_t col = global_block_index % m_parts;
    const std::size_t row = global_block_index / m_parts;

    // if it's supposed to be on this location, we are done
    if (col == m_index)
      return;

    std::size_t local_block = 0;

    // if the location that it's supposed to be on appears
    // before this location
    if (col < m_index)
    {
      // compute the distance between the two locations
      // and place it in the next block that's on
      // this location
      const std::size_t distance = m_index - col;
      local_block = global_block_index + distance;
    }
    // otherwise we can just go to the block for this location
    // that is in the next row
    else if (col > m_index)
    {
      local_block = (row + 1)*m_parts + m_index;
    }

    // the next free vd is the first vd of the block we found 
    m_curr_vd = local_block * m_block;
  }

  //////////////////////////////////////////////////////////////////////
  /// Returns the current descriptor.
  //////////////////////////////////////////////////////////////////////
  VD curr_vd() const
  { return m_curr_vd; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Resets the generator to restart producing descriptors.
  /// To be used after clearing the graph.
  //////////////////////////////////////////////////////////////////////
  void reset()
  { m_curr_vd = m_block*m_index; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the version of the generator.
  /// Provided for compatibility with versioning generators.
  //////////////////////////////////////////////////////////////////////
  unsigned long version()
  { return 0; }

  void define_type(typer& t)
  {
    t.member(m_curr_vd);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief An adjacency list using an unordered map for storing vertices.
/// @ingroup pgraphAdjacency
///
/// Used inside the @ref adjacency_list_model to store vertices.
/// @tparam Traits The traits class for specifying the types of descriptors,
/// storages, edges and vertices, etc.
//////////////////////////////////////////////////////////////////////
template <class Traits>
class adjacency_list_hashmap_storage
{
public:
  typedef typename Traits::vertex_descriptor           vertex_descriptor;
  typedef typename Traits::vertex_property             vertex_property;
  typedef typename Traits::edge_type                   edge_type;
  typedef typename Traits::edgelist_type               edgelist_type;
  typedef typename Traits::vertex_impl_type            vertex_impl_type;

  /// Type of the vertex descriptor generator.
  typedef vdg_strided<vertex_descriptor>               vdg_type;

  /// The vertices are stored in a hash_map.
  typedef stapl::hash_map<vertex_descriptor,vertex_impl_type> vertex_set_type;

  /// Internal value type for the hash_map storage.
  typedef std::pair<vertex_descriptor,vertex_impl_type>       iv_type;

 protected:
  // Data members.
  /// The vertex descriptor generator.
  vdg_type         m_vdg;

  /// The container for storing vertices.
  vertex_set_type  m_storage;

 public:
  typedef sequential::mit_adaptor
            <typename vertex_set_type::iterator>       iterator;
  typedef sequential::mit_adaptor
            <typename vertex_set_type::const_iterator> const_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a storage with the given number of vertices starting
  /// from the specified descriptor, with the given default value.
  /// Vertices have contiguous descriptors.
  /// @param start_vd The starting descriptor of the vertices.
  /// @param num_vds The number of vertices to be initially stored.
  /// @param default_value The default value for the vertices.
  //////////////////////////////////////////////////////////////////////
  adjacency_list_hashmap_storage(size_t const&  start_vd,
                                 size_t const&  num_vds,
                                 vertex_impl_type const& default_value)
   : m_vdg()
  {
    //init the vertex descriptors and adj lists
    for (size_t i=start_vd; i<start_vd+num_vds; ++i) {
      m_storage.insert(iv_type(i, vertex_impl_type(i,
                                                   default_value.property())));
    }
  }

  adjacency_list_hashmap_storage(adjacency_list_hashmap_storage const& other)
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
      storage_it->second.edgelist() = (it->second.edgelist());
    }
  }

  /// Resize the internal storage to accommodate more vertices.
  void resize(size_t nv)
  {
    this->m_storage.clear();
    vertex_property p = vertex_property(); // default constructed property;
    for (size_t i=0;i<nv;++i) {
      vertex_descriptor vd = m_vdg.next();
      m_storage.insert(iv_type(vd, vertex_impl_type(vd, p)));
    }
  }

  iterator begin()
  { return iterator(this->m_storage.begin()); }

  iterator end()
  { return iterator(this->m_storage.end()); }

  const_iterator begin() const
  { return const_iterator(this->m_storage.begin()); }

  const_iterator end() const
  { return const_iterator(this->m_storage.end()); }

  //////////////////////////////////////////////////////////////////////
  /// Returns the size of storage; for use in g.num_vertices().
  //////////////////////////////////////////////////////////////////////
  size_t size() const
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
  /// @brief Adds vertex to the storage with the given descriptor & property.
  /// @param vd The explicit descriptor of the added vertex.
  /// @param vp The vertex property of the added vertex.
  /// @return The descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_descriptor vd, vertex_property const& vp)
  {
    vertex_impl_type v(vd, vp);
    m_vdg.update(vd);
    m_storage.insert(iv_type(vd,v));
    return vd;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Updates the vertex descriptor generator with the next free
  ///        descriptor
  //////////////////////////////////////////////////////////////////////
  void update_next_descriptor(vertex_descriptor const& vd)
  {
    m_vdg.update(vd);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the vertex pointed to by the specified iterator.
  /// @param vi The iterator of the vertex to be deleted.
  /// @return Whether or not the vertex was successfully deleted.
  //////////////////////////////////////////////////////////////////////
  bool delete_vertex(vertex_descriptor const&, iterator const& vi)
  {
    if (vi.base() == m_storage.end())
    {
      return false;
    }
    this->m_storage.erase(vi.base());
    m_vdg.free(vi->descriptor());
    return true;
  }

  vertex_descriptor next_free_descriptor()
  {
    return m_vdg.next();
  }

  //////////////////////////////////////////////////////////////////////
  /// Clears the storage container of all vertices and edges.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    for (typename vertex_set_type::iterator i=this->m_storage.begin();
        i!=this->m_storage.end();++i)
    {
      i->second.clear();
    }
    this->m_storage.clear();
    this->m_vdg.reset();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the specified vertex
  //////////////////////////////////////////////////////////////////////
  iterator find_vertex(vertex_descriptor const& vd)
  {
    return iterator(m_storage.find(vd));
  }

  //////////////////////////////////////////////////////////////////////
  /// Not used. Provided for compatibility.
  //////////////////////////////////////////////////////////////////////
  void update_descriptor(vertex_descriptor& vd,
                         iterator const& vi)
  { }

  //////////////////////////////////////////////////////////////////////
  /// Returns the current max descriptor.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor get_max_descriptor()
  { return m_vdg.curr_vd(); }

#ifdef _STAPL
  size_t memory_size(void) const
  {
    return memory_used<vertex_set_type>::size(m_storage) + sizeof(m_vdg);
  }

  void define_type(typer& t)
  {
    t.member(m_storage);
    t.member(m_vdg);
  }
#endif
}; // adjacency_list_hashmap_storage storage class


} // namespace stapl

#endif
