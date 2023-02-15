/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_VDG_STORAGE_VECTOR_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_VDG_STORAGE_VECTOR_HPP

#include <vector>
#include <iostream>

namespace stapl {
namespace sequential {

//////////////////////////////////////////////////////////////////////
/// @brief Self-maintaining vertex descriptors.
/// @ingroup graphBaseStorage
///
/// These descriptors implement versioning and keep an iterator to the
/// vertex. They can be used by the versioned graphs to extract the
/// vertex object in constant-time if their version matches the graph's.
/// Versioned graphs store additional versioning information that is updated
/// whenever the graph performs operations that may invalidate iterators.
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam VD_Base The type of the base vertex descriptor.
/// @tparam VertexP The type of the vertex property.
/// @tparam EdgeP The type of the edge property.
//////////////////////////////////////////////////////////////////////
template <graph_attributes D, class VD_Base, class VertexP, class EdgeP>
class vertex_descriptor_vector
{
private:
  typedef vertex_descriptor_vector<D, VD_Base, VertexP, EdgeP> this_type;
  typedef typename select_edge<this_type, EdgeP, D>::type edge_type;
  typedef adjacency_descriptor_impl<edge_type> edgelist_type;
  typedef typename select_vertex<this_type, VertexP,
                                 edgelist_type>::type vertex_impl_type;
  typedef std::vector<vertex_impl_type> vertex_set_type;

public:
  typedef typename vertex_set_type::iterator vertex_iterator;

private:
  VD_Base m_user_descriptor;
  vertex_iterator m_VI;
  long m_version;

public:
  vertex_descriptor_vector()
      : m_version(-1)
  {
  }

  vertex_descriptor_vector(VD_Base const& vd)
      : m_user_descriptor(vd), m_version(-1)
  {
  }

  vertex_descriptor_vector(VD_Base const& vd, vertex_iterator const& vi,
                           long const version)
      : m_user_descriptor(vd), m_VI(vi), m_version(version)
  {
  }

  vertex_iterator vi(void) const
  {
    return this->m_VI;
  }

  long version(void) const
  {
    return this->m_version;
  }

  VD_Base descriptor(void) const
  {
    return this->m_user_descriptor;
  }

  void vi(vertex_iterator vi)
  {
    this->m_VI = vi;
  }

  void version(long version)
  {
    this->m_version = version;
  }

  void descriptor(VD_Base vd)
  {
    this->m_user_descriptor = vd;
  }

  bool operator!=(this_type& other)
  {
    return (this->m_user_descriptor != other.descriptor());
  }

  bool operator!=(VD_Base& other)
  {
    return (this->m_user_descriptor != other);
  }

  bool operator==(this_type& other)
  {
    return (this->m_user_descriptor == other.descriptor());
  }

  bool operator==(VD_Base& other)
  {
    return (this->m_user_descriptor == other);
  }

  operator VD_Base() const
  {
    return (this)->m_user_descriptor;
  }

  friend std::ostream& operator<<(std::ostream& stream, this_type& vd)
  {
    stream << vd.descriptor();
    return stream;
  }

  friend std::istream& operator>>(std::istream& stream, this_type& vd)
  {
    stream >> vd.m_user_descriptor;
    return stream;
  }

#ifdef _STAPL
  void define_type(typer& t)
  {
    t.member(m_user_descriptor);
    t.member(m_VI);
    t.member(m_version);
  }
#endif
};


//////////////////////////////////////////////////////////////////////
/// @brief A vertex descriptor generator class that generates the
/// descriptors for the smart vector storage, and tracks the version
/// number of descriptors.
/// @ingroup graphBaseStorage
//////////////////////////////////////////////////////////////////////
template <class VD>
class vdg_base_vector
{
protected:
  long m_version;
  size_t m_curr_vd;

public:
  vdg_base_vector()
      : m_version(0), m_curr_vd(0)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a generator which produces descriptors starting from
  /// the specified initial offset.
  //////////////////////////////////////////////////////////////////////
  vdg_base_vector(long start)
      : m_version(0), m_curr_vd(start)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Generates and returns the next vertex descriptor, updating
  /// the internal version.
  //////////////////////////////////////////////////////////////////////
  VD next()
  {
    ++m_version;
    return VD(m_curr_vd++);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Possibly reclaim deleted VDs and update internal version.
  //////////////////////////////////////////////////////////////////////
  void free(VD const&)
  {
    ++m_version;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compares version numbers to see if the iterator inside is valid.
  //////////////////////////////////////////////////////////////////////
  bool is_valid(VD const& vd)
  {
    if (vd.version() == m_version)
      return true;
    return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Manually increment the version number, if needed for some storage.
  //////////////////////////////////////////////////////////////////////
  void increment()
  {
    ++m_version;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Integrate the current VD into the generated sequence --
  /// future invocations of next() should not produce this VD.
  //////////////////////////////////////////////////////////////////////
  void update(const VD& _vd)
  {
    if (_vd.descriptor() >= m_curr_vd)
      m_curr_vd = _vd + 1;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the current descriptor.
  //////////////////////////////////////////////////////////////////////
  VD curr_vd() const
  {
    return m_curr_vd;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reset the generator to start from descriptor zero (0).
  /// Also reset the versioning.
  /// @todo Reset to the initial given value instead of zero (0).
  //////////////////////////////////////////////////////////////////////
  void reset()
  {
    m_version = 0;
    m_curr_vd = 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the internal version of this generator.
  //////////////////////////////////////////////////////////////////////
  unsigned long version()
  {
    return m_version;
  }

#ifdef _STAPL
  void define_type(typer& t)
  {
    t.member(m_version);
    t.member(m_curr_vd);
  }
#endif
};


//////////////////////////////////////////////////////////////////////
/// @brief An adjacency list using an std::vector for storing vertices.
/// Used inside the @ref adjacency_list_model to store vertices.
/// @ingroup graphBaseStorage
/// @tparam Traits The traits class for specifying the types of descriptors,
/// storages, edges and vertices.
/// Contains the physical storage for the Graph, and provides functionality
/// to Add/Delete/Find vertices, and Iterators to access the storage from
/// outside.
/// @note This is a versioning storage, i.e., it maintains and supports
/// smart descriptors that can help faster vertex-lookups if they are
/// reused often.
/// Smart vertex descriptors store iterators to their vertices, and
/// return that iterator if the version matches the version of the graph.
//////////////////////////////////////////////////////////////////////
template <class Traits>
class vdg_storage_vector
{
  typedef vdg_storage_vector<Traits> this_type;

public:
  typedef typename Traits::vertex_descriptor vertex_descriptor;
  typedef typename Traits::vertex_property vertex_property;

  typedef typename Traits::edge_type edge_type;
  typedef adjacency_descriptor_impl<edge_type> edgelist_type;
  typedef typename select_vertex<vertex_descriptor, vertex_property,
      edgelist_type>::type vertex_impl_type;

  /// Type of the vertex descriptor generator.
  typedef vdg_base_vector<vertex_descriptor> vdg_type;

  /// The vertices are stored in an std::vector.
  typedef std::vector<vertex_impl_type> vertex_set_type;

protected:
  // Data members.
  /// The vertex descriptor generator.
  vdg_type m_vdg;

  /// The container for storing vertices.
  vertex_set_type m_storage;

public:
  typedef typename vertex_set_type::iterator iterator;
  typedef typename vertex_set_type::const_iterator const_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a storage with the given number of vertices starting
  /// from zero (0). Vertices have contiguous descriptors.
  /// @param nv The number of vertices to be initially stored.
  //////////////////////////////////////////////////////////////////////
  vdg_storage_vector(size_t nv = 0)
      : m_vdg(0)
  {
    m_storage.reserve(nv);
    //init the vertex descriptors and adj lists
    vertex_property p = vertex_property();  //default constructed property;
    for (size_t i = 0; i < nv; ++i) {
      m_storage.push_back(vertex_impl_type(m_vdg.next(), p));
    }
  }

  vdg_storage_vector(this_type const& other)
      : m_vdg(other.m_vdg)
  {
    this->m_storage = other.m_storage;
    // the edge list is a pointer; next clone the edge lists
    iterator storage_it = this->m_storage.begin();
    const_iterator storage_it_other = other.m_storage.begin();
    const_iterator storage_it_other_end = other.m_storage.end();

    for (const_iterator it = storage_it_other; it != storage_it_other_end;
        ++it, ++storage_it) {
      storage_it->edgelist(new edgelist_type(it->edgelist()));
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Resize the internal storage to accommodate specified number of
  /// vertices.
  //////////////////////////////////////////////////////////////////////
  void resize(size_t nv)
  {
    m_storage.resize(nv);
  }

  iterator begin()
  {
    return iterator(this->m_storage.begin());
  }

  const_iterator begin() const
  {
    return const_iterator(this->m_storage.begin());
  }

  iterator end()
  {
    return iterator(this->m_storage.end());
  }

  const_iterator end() const
  {
    return const_iterator(this->m_storage.end());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of vertices in the storage; for use in
  /// g.num_vertices().
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    return this->m_storage.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds vertex to the storage with the given vertex descriptor.
  /// @param vd The vertex descriptor of the added vertex.
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
  vertex_descriptor add_vertex(vertex_descriptor vd,
                               vertex_property const& vp)
  {
    vertex_impl_type v(vd, vp);
    m_vdg.update(vd);
    m_storage.push_back(v);
    return v.descriptor();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the vertex pointed to by the specified iterator.
  /// @param vi The iterator of the vertex to be deleted.
  /// @return Whether or not the vertex was successfully deleted.
  //////////////////////////////////////////////////////////////////////
  bool delete_vertex(vertex_descriptor const& vd, iterator const& vi)
  {
    if (vi == this->end()) {
      return false;
    }
    this->m_storage.erase(vi);
    m_vdg.free(vi->descriptor());
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clears the storage container of all vertices and edges, and
  /// resets the vertex descriptor generator.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    for (typename vertex_set_type::iterator i = this->m_storage.begin();
        i != this->m_storage.end(); ++i) {
      i->clear();
    }
    this->m_storage.clear();
    this->m_vdg.reset();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the specified vertex, or returns
  /// an iterator to @ref this->end() if the vertex does not exist.
  /// @note Checks with VDG_Base if iterator version of given descriptor
  /// is valid. Returns VD.Iterator() if valid (O(1)), or searches storage
  /// for correct iterator otherwise (O(n)).
  //////////////////////////////////////////////////////////////////////
  iterator find_vertex(vertex_descriptor const& vd_input)
  {
    vertex_descriptor& vd = const_cast<vertex_descriptor&>(vd_input);
    iterator vi, startvi;
    if (m_vdg.is_valid(vd)) {
      // cout << " valid!" << endl; cout.flush();
      return vd.vi();
    }
    // cout << " invalid" << endl; cout.flush();
    // find the spot to start looking, hopefully at the _vd (constant time!!)
    if (this->size() == 0)
      return this->end();
    if (this->size() > vd) {
      startvi = this->begin() + vd;
    } else {
      startvi = this->end() - 1;
    }
    // look back from v[vid]
    vi = startvi;
    while (vi >= this->begin()) {
      if ((*vi).descriptor() == vd) {
        vd.version(m_vdg.version());
        vd.vi(vi);
        return vi;
      } else {
        vi--;
      }
    }
    // look forward from v[vid]
    vi = startvi;
    while (vi != this->end()) {
      if ((*vi).descriptor() == vd) {
        vd.version(m_vdg.version());
        vd.vi(vi);
        return vi;
      } else {
        ++vi;
      }
    }
    // if didn't find it return this->end() like STL find
    return this->end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Updates a versioning descriptor with the new iterator.
  //////////////////////////////////////////////////////////////////////
  void update_descriptor(vertex_descriptor& vd, iterator const& vi)
  {
    if (vi->descriptor() != vd)
      vd.vi(find_vertex(vd));
    else
      vd.vi(vi);
    vd.version(m_vdg.version());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the current max descriptor.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor get_max_descriptor() const
  {
    return m_vdg.curr_vd();
  }

#ifdef _STAPL
  void define_type(stapl::typer& t)
  {
    t.member(m_storage);
    t.member(m_vdg);
  }
#endif
};

} // namespace sequential
}

#endif
