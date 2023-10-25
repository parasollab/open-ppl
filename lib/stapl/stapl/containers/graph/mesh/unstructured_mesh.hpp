/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_MESH_UNSTRUCTURED_MESH_HPP
#define STAPL_CONTAINERS_GRAPH_MESH_UNSTRUCTURED_MESH_HPP

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/containers/graph/mesh/mesh_property.hpp>
#include <stapl/containers/graph/partitioners/graph_partitioner_utils.hpp>
#include <string>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Implementation of n-dimensional unstructured mesh.
/// Inherits from @ref stapl::dynamic_graph and adds functionality to manage
/// cells, faces and mesh vertices.
/// @tparam CellProperty type of property for the mesh cell.
/// Default is cell_property.
/// @tparam FaceProperty type of property for the mesh face.
/// Default is face_property.
//////////////////////////////////////////////////////////////////////
template <int Dim, typename CellProperty = cell_property<Dim>,
                   typename FaceProperty = face_property<Dim> >
class unstructured_mesh
  : public dynamic_graph<DIRECTED, MULTIEDGES, CellProperty, FaceProperty>
{
public:
  typedef CellProperty cell_property;
  typedef FaceProperty face_property;
  typedef dynamic_graph<DIRECTED, MULTIEDGES,
                        cell_property, face_property>  base_type;
  typedef typename base_type::edge_descriptor          edge_descriptor;
  typedef typename face_property::vid_sequence_type    vid_sequence_type;
  typedef typename face_property::geom_vector_type     geom_vector_type;
  typedef array<geom_vector_type>                      vertex_array_type;
  typedef size_t                                       index_type;

  const static size_t dim_value = Dim;
private:
  ///pArray storing the coordinates of the mesh vertices.
  vertex_array_type m_array_vertices;

public:
  unstructured_mesh()
    : base_type(), m_array_vertices()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Contructor taking a mesh generator.
  /// @param gen mesh generator.
  //////////////////////////////////////////////////////////////////////
  template<typename Generator>
  unstructured_mesh(Generator gen)
    : base_type(), m_array_vertices(gen.num_vertices())
  {
    gen(*this);
    build_centroids(*this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor initializing the size of the array of vertices.
  /// @param num_mesh_vertices number of vertices of the mesh.
  /// @note here "vertex" means a geometric vertex of a mesh cell and a graph
  /// vertex represents a mesh cell.
  //////////////////////////////////////////////////////////////////////
  unstructured_mesh(size_t const& num_mesh_vertices)
    : base_type(), m_array_vertices(num_mesh_vertices)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the coordinates of a cell vertex.
  /// @param idx cell vertex identifier.
  /// @param val geometric vector representing the coordinates of
  /// the cell vertex.
  //////////////////////////////////////////////////////////////////////
  void set_mesh_vertex(index_type const& idx, geom_vector_type const& val)
  {
    m_array_vertices.set_element(idx, val);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add a cell face asynchronously.
  /// @param ed face descriptor.
  /// @param vertex_ids sequence of vertices of the cell face.
  //////////////////////////////////////////////////////////////////////
  void add_cell_face_async(edge_descriptor const& ed,
                                  vid_sequence_type const& vertex_ids =
                                                           vid_sequence_type())
  {
    this->add_edge_async(ed, face_property(vertex_ids, m_array_vertices));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a pointer on the pArray storing the mesh vertices.
  /// @return pointer over pArray of vertices.
  //////////////////////////////////////////////////////////////////////
  vertex_array_type* get_vertex_array()
  {
    return &m_array_vertices;
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_array_vertices);
  }
};  //unstructured_mesh

} //namespace stapl

#endif
