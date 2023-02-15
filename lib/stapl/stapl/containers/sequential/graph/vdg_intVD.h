/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_VDG_INT_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_VDG_INT_HPP

#include <map>
#include <boost/type_traits/remove_reference.hpp>
#include <boost/type_traits/remove_const.hpp>

namespace stapl {
namespace sequential {

//////////////////////////////////////////////////////////////////////
/// @brief Base class for vertex descriptor generators.
/// @ingroup graphBaseUtil
///
/// This class will generate unique vertex descriptors, taking into
/// account that the graph may auto generate them, as well as the users
/// may specify them.
/// This is used by @ref vdg_storage_int to generate/delete/find vertices.
//////////////////////////////////////////////////////////////////////
template <class VD>
class vdg_base_int
{
protected:
  size_t m_curr_vd;
public:

  vdg_base_int(void)
      : m_curr_vd(0)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a generator which produces descriptors starting from
  /// the specified initial offset.
  //////////////////////////////////////////////////////////////////////
  vdg_base_int(long start)
      : m_curr_vd(start)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Generates and returns the next vertex descriptor.
  //////////////////////////////////////////////////////////////////////
  VD next(void)
  {
    return VD(m_curr_vd++);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reclaim deleted VDs. Not used in this class.
  //////////////////////////////////////////////////////////////////////
  void free(const VD&)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compares version numbers to see if the iterator inside is valid.
  /// Used in versioning generators. Unused in this class.
  //////////////////////////////////////////////////////////////////////
  bool is_valid(const VD&)
  {
    return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Manually increment the version number, if needed for some storage.
  /// Unused in this class.
  //////////////////////////////////////////////////////////////////////
  void increment(void)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Integrate the current VD into the generated sequence --
  /// future invocations of next() should not produce this VD.
  //////////////////////////////////////////////////////////////////////
  void update(VD const& vd)
  {
    if (vd >= m_curr_vd)
      m_curr_vd = vd + 1;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the current descriptor.
  //////////////////////////////////////////////////////////////////////
  VD curr_vd(void) const
  {
    return m_curr_vd;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reset the generator to start from descriptor zero (0).
  /// @todo Reset to the initial given value instead of zero (0).
  //////////////////////////////////////////////////////////////////////
  void reset(void)
  {
    m_curr_vd = 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the version of the container.
  /// @return Version number of 0
  //////////////////////////////////////////////////////////////////////
  size_t version(void) const
  {
    return 0;
  }

#ifdef _STAPL
  void define_type(typer& t)
  {
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
//////////////////////////////////////////////////////////////////////
template <class Traits>
class vdg_storage_int
{
  typedef vdg_storage_int<Traits> this_type;

public:
  typedef typename Traits::vertex_descriptor vertex_descriptor;
  typedef typename Traits::vertex_property vertex_property;

protected:
  typedef typename Traits::edge_type edge_type;
  typedef adjacency_descriptor_impl<edge_type> edgelist_type;
  typedef typename select_vertex<vertex_descriptor, vertex_property,
      edgelist_type>::type vertex_impl_type;

  /// Type of the vertex descriptor generator.
  typedef vdg_base_int<vertex_descriptor> vdg_type;

  /// The vertices are stored in an std::vector.
  typedef std::vector<vertex_impl_type> vertex_set_type;

  // Data members.
  /// The vertex descriptor generator.
  vdg_type m_vdg;

  /// The container for storing vertices.
  vertex_set_type m_storage;

public:
  // simple iterator type for storage; the graph iterator is decided in
  // the adjacency list class
  typedef typename vertex_set_type::iterator iterator;
  typedef typename vertex_set_type::const_iterator const_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a storage with the given number of vertices starting
  /// from zero (0). Vertices have contiguous descriptors.
  /// @param nv The number of vertices to be initially stored.
  //////////////////////////////////////////////////////////////////////
  vdg_storage_int(size_t nv = 0)
      : m_vdg(0)
  {
    m_storage.reserve(nv);
    //init the vertex descriptors and adj lists
    vertex_property p = vertex_property();  //default constructed property;
    for (size_t i = 0; i < nv; ++i) {
      m_storage.push_back(vertex_impl_type(m_vdg.next(), p));
    }
  }

  vdg_storage_int(this_type const& other)
      : m_vdg(other.m_vdg)
  {
    this->m_storage = other.m_storage;
    // the edge list is a pointer; next clone the edge lists
    iterator storage_it = this->m_storage.begin();
    const_iterator storage_it_other = other.m_storage.begin();
    const_iterator storage_it_other_end = other.m_storage.end();

    for (const_iterator it = storage_it_other; it != storage_it_other_end;
        ++it, ++storage_it) {
      storage_it->edgelist(&(it->edgelist()));
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

  iterator begin(void)
  {
    return iterator(this->m_storage.begin());
  }

  const_iterator begin(void) const
  {
    return const_iterator(this->m_storage.begin());
  }

  iterator end(void)
  {
    return iterator(this->m_storage.end());
  }

  const_iterator end(void) const
  {
    return const_iterator(this->m_storage.end());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of vertices in the storage; for use in
  /// g.num_vertices().
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return this->m_storage.size();
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
  vertex_descriptor add_vertex(vertex_descriptor const& vd,
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
  bool delete_vertex(vertex_descriptor const&, iterator const& vi)
  {
    if (vi == this->end()) {
      return false;
    }
    this->m_storage.erase(vi);
    m_vdg.free(vi->descriptor());
    return true;
    //commented out next is another delete strategy
    //if(vi+1 == this->end()) this->m_vertices.pop_back();
    //else {
    //*(vi.base()) = *((this->end()-1).base());
    //this->m_vertices.pop_back();
    //}
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
  //////////////////////////////////////////////////////////////////////
  iterator find_vertex(vertex_descriptor const& vd)
  {
    iterator vi, startvi;

    // find the spot to start looking, hopefully at the vd (constant time!!)
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
        return vi;
      } else {
        vi--;
      }
    }
    // look forward from v[_vid]
    vi = startvi;
    while (vi != this->end()) {
      if ((*vi).descriptor() == vd) {
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
  /// Unused in this class.
  //////////////////////////////////////////////////////////////////////
  void update_descriptor(vertex_descriptor& vd, iterator const& vi)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the current max descriptor.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor get_max_descriptor(void) const
  {
    return m_vdg.curr_vd();
  }

#ifdef _STAPL
  void define_type(stapl::typer& t) {
    t.member(m_storage);
    t.member(m_vdg);
  }
#endif
};



//////////////////////////////////////////////////////////////////////
/// @brief Specialization of helper to infer the return property
/// pointer (const or non const) for non const value,
/// used by adjacency list using map for vertex storage.
/// @ingroup graphBaseStorage
//////////////////////////////////////////////////////////////////////
template <typename Value>
struct mit_pointer
{
  typedef typename Value::second_type* type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of helper to infer the return property
/// pointer (const or non const) for const value,
/// used by adjacency list using map for vertex storage.
/// @ingroup graphBaseStorage
//////////////////////////////////////////////////////////////////////
template <typename Value>
struct mit_pointer<const Value&>
{
  typedef const typename Value::second_type* type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of helper to infer the return property
/// reference (const or non const) for non const value,
/// used by adjacency list using map for vertex storage.
/// @ingroup graphBaseStorage
//////////////////////////////////////////////////////////////////////
template <typename Value>
struct mit_reference
{
private:
  typedef typename boost::remove_reference<Value>::type value_type;
public:
  typedef typename value_type::second_type& type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of helper to infer the return property
/// reference (const or non const) for const value,
/// used by adjacency list using map for vertex storage.
/// @ingroup graphBaseStorage
//////////////////////////////////////////////////////////////////////
template <typename Value>
struct mit_reference<const Value&>
{
  typedef const typename Value::second_type& type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of helper to construct the type of the const
/// reference used by adjacency list using map for vertex storage.
/// @ingroup graphBaseStorage
//////////////////////////////////////////////////////////////////////
template <typename Value>
struct mit_const_reference
{
private:
  typedef typename boost::remove_const<
    typename boost::remove_reference<Value>::type>::type value_type;
public:
  typedef const typename value_type::second_type& type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Iterator adaptor for using a map for vertex storage.
/// @ingroup graphBaseStorage
//////////////////////////////////////////////////////////////////////
template <typename BaseIterator>
class mit_adaptor
  : public boost::iterator_adaptor<mit_adaptor<BaseIterator>, BaseIterator>
{
private:
  typedef boost::iterator_adaptor<mit_adaptor<BaseIterator>,
                                  BaseIterator> base_type;
  typedef typename std::iterator_traits<BaseIterator>::value_type
                                                internal_value_type;
  typedef typename std::iterator_traits<BaseIterator>::reference
                                                internal_reference_type;
  typedef typename internal_value_type::second_type vertex_type;
  typedef typename mit_pointer<internal_value_type>::type pointer_type;
public:
  typedef vertex_type value_type;
  typedef typename mit_pointer<internal_value_type>::type pointer;
  typedef typename mit_reference<internal_reference_type>::type reference;
  typedef typename mit_const_reference<internal_reference_type>::type
                                                const_reference;
  typedef typename vertex_type::vertex_descriptor vertex_descriptor;
  typedef typename vertex_type::property_type property_type;
  typedef typename vertex_type::edgelist_type edgelist_type;

  mit_adaptor(void) = default;

  mit_adaptor(BaseIterator iterator)
      : base_type(iterator)
  { }

  template <typename Iter>
  inline mit_adaptor(mit_adaptor<Iter> const& other)
      : base_type(other.base())
  {
  }

  pointer operator->(void) const
  {
    return const_cast<vertex_type*>(&(this->base_reference()->second));
  }

  reference operator*(void)
  {
    return this->base_reference()->second;
  }

  const_reference operator*(void) const
  {
    return this->base_reference()->second;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief An adjacency list using an std::map for storing vertices.
/// Used inside the @ref adjacency_list_model to store vertices.
/// @ingroup graphBaseStorage
/// @tparam Traits The traits class for specifying the types of descriptors,
/// storages, edges and vertices.
/// Contains the physical storage for the Graph, and provides functionality
/// to Add/Delete/Find vertices, and Iterators to access the storage from
/// outside.
//////////////////////////////////////////////////////////////////////
template <class Traits>
class vdg_map_int
{
  typedef vdg_map_int<Traits> this_type;

public:
  typedef typename Traits::vertex_descriptor vertex_descriptor;
  typedef typename Traits::vertex_property vertex_property;

  typedef typename Traits::edge_type edge_type;
  typedef adjacency_descriptor_impl<edge_type> edgelist_type;
  typedef typename select_vertex<vertex_descriptor, vertex_property,
      edgelist_type>::type vertex_impl_type;

  /// Type of the vertex descriptor generator.
  typedef vdg_base_int<vertex_descriptor> vdg_type;

  /// The vertices are stored in an std::map.
  typedef std::map<vertex_descriptor, vertex_impl_type> vertex_set_type;
  /// Internal value type of the std::map.
  typedef std::pair<vertex_descriptor, vertex_impl_type> iv_type;

protected:
  // Data members.
  /// The vertex descriptor generator.
  vdg_type m_vdg;

  /// The container for storing vertices.
  vertex_set_type m_storage;

public:
  typedef mit_adaptor<typename vertex_set_type::iterator> iterator;
  typedef mit_adaptor<typename vertex_set_type::const_iterator> const_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a storage with the given number of vertices starting
  /// from zero (0). Vertices have contiguous descriptors.
  /// @param nv The number of vertices to be initially stored.
  //////////////////////////////////////////////////////////////////////
  vdg_map_int(size_t nv = 0)
      : m_vdg(0)
  {
    //init the vertex descriptors and adj lists
    vertex_property p = vertex_property();
    for (size_t i = 0; i < nv; ++i) {
      vertex_descriptor vd = m_vdg.next();
      m_storage.insert(iv_type(vd, vertex_impl_type(vd, p)));
    }
  }

  vdg_map_int(this_type const& other)
      : m_vdg(other.m_vdg)
  {
    this->m_storage = other.m_storage;
    // the edge list is a pointer; next clone the edge lists
    typename vertex_set_type::iterator storage_it = this->m_storage.begin();
    typename vertex_set_type::const_iterator storage_it_other =
        other.m_storage.begin();
    typename vertex_set_type::const_iterator storage_it_other_end =
        other.m_storage.end();

    for (typename vertex_set_type::const_iterator it = storage_it_other;
         it != storage_it_other_end; ++it, ++storage_it) {
      storage_it->second.edgelist(&(it->second.edgelist()));
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Resize the internal storage to accommodate specified number of
  /// vertices.
  //////////////////////////////////////////////////////////////////////
  void resize(size_t nv)
  {
    this->m_storage.clear();
    vertex_property p = vertex_property(); // default constructed property;

    for (size_t i = 0; i < nv; ++i) {
      vertex_descriptor vd = m_vdg.next();
      m_storage.insert(iv_type(vd, vertex_impl_type(vd, p)));
    }
  }

  iterator begin(void)
  {
    return iterator(this->m_storage.begin());
  }
  const_iterator begin(void) const
  {
    return const_iterator(this->m_storage.begin());
  }
  iterator end(void)
  {
    return iterator(this->m_storage.end());
  }
  const_iterator end(void) const
  {
    return const_iterator(this->m_storage.end());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of vertices in the storage; for use in
  /// g.num_vertices().
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return this->m_storage.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds vertex to the storage with the given vertex descriptor.
  /// @param vd The descriptor of the added vertex.
  /// @return The descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_descriptor const& vd)
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
  vertex_descriptor add_vertex(vertex_descriptor const& vd,
                               vertex_property const& vp)
  {
    vertex_impl_type v(vd, vp);
    m_vdg.update(vd);
    m_storage.insert(iv_type(vd, v));
    return vd;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the vertex pointed to by the specified iterator.
  /// @param vi The iterator of the vertex to be deleted.
  /// @return Whether or not the vertex was successfully deleted.
  //////////////////////////////////////////////////////////////////////
  bool delete_vertex(vertex_descriptor const&, iterator const& vi)
  {
    if (vi.base() == m_storage.end()) {
      return false;
    }
    this->m_storage.erase(vi.base());
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
      i->second.clear();
    }
    this->m_storage.clear();
    this->m_vdg.reset();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the specified vertex, or returns
  /// an iterator to @ref this->end() if the vertex does not exist.
  //////////////////////////////////////////////////////////////////////
  iterator find_vertex(vertex_descriptor const& vd)
  {
    return iterator(m_storage.find(vd));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Updates a versioning descriptor with the new iterator.
  /// Unused in this class.
  //////////////////////////////////////////////////////////////////////
  void update_descriptor(vertex_descriptor& vd, iterator const& vi)
  {
  }

  vertex_descriptor get_max_descriptor(void) const
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
// class vdg_map_int

} // namespace sequential
} // namespace stapl

#endif
