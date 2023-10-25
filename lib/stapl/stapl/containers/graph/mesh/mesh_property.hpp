/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_MESH_MESH_PROPERTY_HPP
#define STAPL_CONTAINERS_GRAPH_MESH_MESH_PROPERTY_HPP

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/mesh/geom_vector.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <vector>
#include <stapl/domains/explicit_domain.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default property for a mesh cell.
/// @tparam Dim dimension of the mesh.
/// @tparam Precision type of vertex coordinate.
///
/// A mesh cell property requires get_centroid() and
/// set_centroid(geom_vector const&) methods.
//////////////////////////////////////////////////////////////////////
template<int Dim, typename Precision=double>
class cell_property
{
public:
  typedef geom_vector<Dim, Precision> geom_vector_type;

private:
  geom_vector_type m_centroid;
  size_t m_partition_id;
  double m_value;

public:
  cell_property()
    : m_centroid()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param centroid coordinates of the cell centroid.
  //////////////////////////////////////////////////////////////////////
  cell_property(geom_vector_type const& centroid)
    : m_centroid(centroid)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return centroid coordinates of the cell.
  /// @return centroid coordinates of the cell.
  //////////////////////////////////////////////////////////////////////
  geom_vector_type get_centroid() const
  {
    return m_centroid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the coordinates of the cell centroid.
  /// @param centroid centroid coordinates.
  //////////////////////////////////////////////////////////////////////
  void set_centroid(geom_vector_type const& centroid)
  {
    m_centroid = centroid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return partition identifier of the cell.
  /// @return partition identifier of the cell.
  //////////////////////////////////////////////////////////////////////
  size_t get_partition_id() const
  {
    return m_partition_id;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the value stores in the cell.
  /// @return value stores in the cell.
  //////////////////////////////////////////////////////////////////////
  double value() const
  {
    return m_value;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set value of the cell.
  /// @param val cell value.
  //////////////////////////////////////////////////////////////////////
  void value(double const& val)
  {
    m_value = val;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the cell partition id.
  /// @param pid cell partition id.
  //////////////////////////////////////////////////////////////////////
  void set_partition_id(size_t const& pid)
  {
    m_partition_id = pid;
  }

  void define_type(typer& t)
  {
    t.member(m_centroid);
    t.member(m_partition_id);
    t.member(m_value);
  }
}; //cell_property


//////////////////////////////////////////////////////////////////////
/// @brief Default face property for a cell face.
/// @tparam Dim dimension of the mesh.
/// @tparam Precision type of vertex coordinate.
///
/// A mesh cell face property requires a constructor taking a sequence
/// of face vertex ids and a reference to the pArray storing the mesh
/// vertex coordinates.
//////////////////////////////////////////////////////////////////////
template<int Dim, typename Precision=double>
class face_property
{
public:
  typedef geom_vector<Dim, Precision>             geom_vector_type;
  typedef size_t                                  vertex_id;
  typedef std::vector<vertex_id>                  vid_sequence_type;
  typedef explicit_domain<vertex_id>              vid_domain_type;
  typedef array<geom_vector_type>                 array_type;
  typedef array_view<array_type, vid_domain_type> array_view_type;

private:
  array_view_type m_vertices;
  geom_vector_type m_normal;

public:
  face_property()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param vertex_ids sequence of face vertex identifiers.
  /// @param vertex_array pArray of mesh vertex coordinates.
  //////////////////////////////////////////////////////////////////////
  face_property(vid_sequence_type const& vertex_ids, array_type& vertex_array)
   : m_vertices(vertex_array, vid_domain_type(vertex_ids))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get face normal coordinates.
  /// @return face normal coordinates.
  //////////////////////////////////////////////////////////////////////
  geom_vector_type get_normal() const
  {
    return m_normal;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get pView over the vertices' coordinates of this face.
  /// @return pView over the vertices' coordinates of this face.
  //////////////////////////////////////////////////////////////////////
  array_view_type get_vertices() const
  {
    return m_vertices;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get vertex identifiers of this face.
  /// @return vertex identifiers of this face.
  //////////////////////////////////////////////////////////////////////
  vid_sequence_type get_vertex_ids() const
  {
    return m_vertices.domain().get_sequence();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set normal coordinates of this face.
  /// @return normal coordinates of this face.
  //////////////////////////////////////////////////////////////////////
  void set_normal(geom_vector_type const& norm)
  {
    m_normal = norm;
  }

  void define_type(typer& t)
  {
    t.member(m_vertices);
    t.member(m_normal);
  }
}; //face_property


template <typename Accessor, int Dim, typename Precision>
class proxy<cell_property<Dim, Precision>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef cell_property<Dim, Precision> target_t;
  typedef typename target_t::geom_vector_type geom_vector_type;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
  {
    return Accessor::read();
  }

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

  geom_vector_type get_centroid() const
  {
    return Accessor::const_invoke(&target_t::get_centroid);
  }

  void set_centroid(geom_vector_type const& centroid)
  {
    Accessor::invoke(&target_t::set_centroid, centroid);
  }

  size_t get_partition_id() const
  {
    return Accessor::const_invoke(&target_t::get_partition_id);
  }

  void set_partition_id(size_t const& pid)
  {
    Accessor::invoke(&target_t::set_partition_id, pid);
  }

  double value() const
  {
    return Accessor::const_invoke(&target_t::value);
  }

  void value(double const& val)
  {
    Accessor::invoke(&target_t::value, val);
  }

}; //struct proxy

} //namespace stapl

#endif
