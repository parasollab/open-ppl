/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_MESH_IMPORT_MESH_FILE_HPP
#define STAPL_CONTAINERS_GRAPH_MESH_IMPORT_MESH_FILE_HPP

#include <silo.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stapl/containers/array/static_array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <string>
#include <vector>
#include <algorithm>
#include <boost/unordered_map.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to set the coordinates of a geometric vector.
/// @tparam Dim dimemsion of the vector.
//////////////////////////////////////////////////////////////////////
template<int Dim>
struct set_geom_vector
{
  //////////////////////////////////////////////////////////////////////
  /// @param coords array of coordinates.
  /// @param geom_vector geometric vector to be set.
  /// @param i position of the coordinates in the source array.
  //////////////////////////////////////////////////////////////////////
  template<typename Coord, typename GeomVector>
  GeomVector operator()(Coord coords, GeomVector geom_vector, size_t i)
  {
    geom_vector.template set<Dim>(static_cast<double*>(coords[Dim])[i]);
    return set_geom_vector<Dim-1>()(coords, geom_vector, i);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to set the coordinates of a geometric vector.
/// Template specialization to stop the recursion on the coordinates.
//////////////////////////////////////////////////////////////////////
template<>
struct set_geom_vector<-1>
{
  //////////////////////////////////////////////////////////////////////
  /// @param coords array of coordinates.
  /// @param geom_vector geometric vector to be set.
  /// @param i position of the coordinates in the source array.
  template<typename Coord, typename GeomVector>
  GeomVector operator()(Coord coords, GeomVector geom_vector, size_t i)
  {
    return geom_vector;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Return the sister face of a cell face.
/// @param face cell face.
/// @return sister of the cell face.
/// @todo Replace implementation with the use of std::reverse_copy.
//////////////////////////////////////////////////////////////////////
std::vector<size_t> get_sister_face(std::vector<size_t> const& face)
{
  std::vector<size_t> sister_face;
  sister_face.reserve(face.size());
  sister_face.push_back(face.front());
  std::vector<size_t>::const_reverse_iterator rit = face.rbegin(),
                              rend = face.rend()-1;
  for (; rit!=rend; ++rit)
    sister_face.push_back(*rit);
  return sister_face;
}


//////////////////////////////////////////////////////////////////////
/// @brief Store a face in a map and connect it in the mesh  to its
/// sister face if it exists in the map.
/// @param face_map map storing cell faces.
/// @param face cell face to be stored.
/// @param cell_id cell identifier of the face.
/// @param mesh mesh data structure.
//////////////////////////////////////////////////////////////////////
template<typename FaceMap, typename MeshType>
void push_to_face_map(FaceMap & face_map, std::vector<size_t> const& face,
                      size_t const& cell_id, MeshType& mesh)
{
  typedef typename MeshType::edge_descriptor edge_descriptor;

  std::vector<size_t> sister_face(get_sister_face(face));
  typename FaceMap::iterator map_it = face_map.find(sister_face);
  if (map_it != face_map.end())
  {
    mesh.add_cell_face_async(edge_descriptor(cell_id, map_it->second), face);
    mesh.add_cell_face_async(edge_descriptor(map_it->second, cell_id),
                             sister_face);
    face_map.erase(map_it);
  }
  else
  {
    face_map.insert(std::make_pair(face, cell_id));
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Reorder the vertices of a face to have the vertex with the
/// minimum id stored in front.
/// @param face cell face to be reordered.
/// @todo Replace implementation with the use of std::rotate_copy
//////////////////////////////////////////////////////////////////////
void set_face_order(std::vector<size_t>& face)
{
  size_t min_value = *std::min_element(face.begin(), face.end());

  if (face.front() == min_value)
    return;

  std::deque<size_t> face_copy(face.begin(), face.end());
  do
  {
    size_t front_value = face_copy.front();
    face_copy.pop_front();
    face_copy.push_back(front_value);
  } while (face_copy.front() != min_value);

  std::copy(face_copy.begin(), face_copy.end(), face.begin());
}


//////////////////////////////////////////////////////////////////////
/// @brief Return global identifier of a vertex from silo file.
/// @param mesh_file silo file storing the mesh.
/// @param zone_id cell identifier of the vertex.
/// @param index position of the vertex in the cell.
/// @return global identifier of the vertex.
//////////////////////////////////////////////////////////////////////
size_t get_node_id(DBucdmesh *mesh_file, size_t const& zone_id,
                   size_t const& index)
{
  size_t shape_size = mesh_file->zones->shapesize[0];
  size_t local_id = mesh_file->zones->nodelist[zone_id*shape_size+index];
  return static_cast<int*>(mesh_file->gnodeno)[local_id];
}


//////////////////////////////////////////////////////////////////////
/// @brief Extract 2d cells and its vertices from a silo file.
/// @param mesh_file silo mesh file.
/// @param face_map map data structure storing cell faces.
/// @param mesh mesh data structure.
//////////////////////////////////////////////////////////////////////
template<typename FaceMap, typename MeshType>
void extract_2d_zones(DBucdmesh *mesh_file, FaceMap & face_map, MeshType& mesh)
{
  size_t nzones = mesh_file->zones->nzones;
  size_t shape_size = mesh_file->zones->shapesize[0];
  for (size_t i=0; i<nzones; ++i)
  {
    size_t cell_id = static_cast<int*>(mesh_file->zones->gzoneno)[i];
    for (size_t j=0; j<shape_size-1; ++j)
    {
      std::vector<size_t> face;
      face.push_back(get_node_id(mesh_file, i, j));
      face.push_back(get_node_id(mesh_file, i, j+1));
      set_face_order(face);
      push_to_face_map(face_map, face, cell_id, mesh);
    }
    //Special case: last edge
    std::vector<size_t> last_face;
    last_face.push_back(get_node_id(mesh_file, i, shape_size-1));
    last_face.push_back(get_node_id(mesh_file, i, 0));
    set_face_order(last_face);
    push_to_face_map(face_map, last_face, cell_id, mesh);
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Extract tetrahedral cells and its vertices from a silo file.
/// @param mesh_file silo mesh file.
/// @param face_map map data structure storing cell faces.
/// @param mesh mesh data structure.
//////////////////////////////////////////////////////////////////////
template<typename FaceMap, typename MeshType>
void extract_tetrahedron_zones(DBucdmesh *mesh_file, FaceMap & face_map,
                               MeshType& mesh)
{
  size_t nzones = mesh_file->zones->nzones;
  for (size_t i=0; i<nzones; ++i)
  {
    size_t cell_id = static_cast<int*>(mesh_file->zones->gzoneno)[i];
    //Face 1
    std::vector<size_t> face;
    face.push_back(get_node_id(mesh_file, i, 0));
    face.push_back(get_node_id(mesh_file, i, 1));
    face.push_back(get_node_id(mesh_file, i, 2));
    set_face_order(face);
    push_to_face_map(face_map, face, cell_id, mesh);
    face.clear();

    //Face 2
    face.push_back(get_node_id(mesh_file, i, 0));
    face.push_back(get_node_id(mesh_file, i, 3));
    face.push_back(get_node_id(mesh_file, i, 1));
    set_face_order(face);
    push_to_face_map(face_map, face, cell_id, mesh);
    face.clear();

    //Face 3
    face.push_back(get_node_id(mesh_file, i, 0));
    face.push_back(get_node_id(mesh_file, i, 2));
    face.push_back(get_node_id(mesh_file, i, 3));
    set_face_order(face);
    push_to_face_map(face_map, face, cell_id, mesh);
    face.clear();

    //Face 4
    face.push_back(get_node_id(mesh_file, i, 1));
    face.push_back(get_node_id(mesh_file, i, 3));
    face.push_back(get_node_id(mesh_file, i, 2));
    set_face_order(face);
    push_to_face_map(face_map, face, cell_id, mesh);
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Functor importing a mesh from a silo file.
//////////////////////////////////////////////////////////////////////
struct import_silo_file
{
private:
  std::string m_filename;
  size_t m_nnodes;

  //////////////////////////////////////////////////////////////////////
  /// @brief Functor merging maps storing cell faces and adding connections
  /// between sister faces in the mesh data structure.
  /// @tparam MeshType type of the mesh data structure.
  //////////////////////////////////////////////////////////////////////
  template<typename MeshType>
  struct face_map_reduce_wf
  {
    typedef std::vector<size_t>         face_type;
    typedef boost::unordered_map<face_type, size_t> face_map_type;
    typedef face_map_type          result_type;
  private:
    MeshType *const m_mesh;

  public:
    //////////////////////////////////////////////////////////////////////
    /// @param mesh pointer on the mesh data structure.
    //////////////////////////////////////////////////////////////////////
    face_map_reduce_wf(MeshType *const mesh)
      : m_mesh(mesh)
    { }

    //////////////////////////////////////////////////////////////////////
    /// @param map1 map to be merged.
    /// @param map2 map to be merged.
    /// @return merged map of @p map1 and @p map2.
    //////////////////////////////////////////////////////////////////////
    face_map_type operator()(face_map_type map1, face_map_type map2)
    {
      face_map_type::iterator it = map1.begin(),
                              end_it = map1.end();
      for (; it!=end_it; ++it)
      {
        push_to_face_map(map2, it->first, it->second, *m_mesh);
      }
      return map2;
    }

    void define_type(typer &t)
    {
      t.member(m_mesh);
    }
  };

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor extracting the number of vertices of the mesh.
  /// @param fname silo file name storing the mesh.
  //////////////////////////////////////////////////////////////////////
  import_silo_file(std::string fname)
    : m_filename(fname)
  {
    size_t num_locs = get_num_locations();
    size_t myid = get_location_id();
    int max_node_id = 0;
    //Construct location file name
    std::string myfile_name;
    size_t pos = m_filename.find('.');
    std::string mesh_name = m_filename.substr(0, pos);
    myfile_name = m_filename.substr(0, pos+1);
    std::ostringstream myid_str;
    myid_str << myid;
    myfile_name += myid_str.str();
    m_filename = myfile_name;

    DBfile *file = NULL;
    DBucdmesh *mesh = NULL;
    // Open Silo File
    file = DBOpen(m_filename.c_str(), DB_HDF5, DB_READ);
    if (file)
    {
      mesh = DBGetUcdmesh(file, mesh_name.c_str());
      if (!mesh)
      {
        std::cerr << "Error: Mesh \"" << mesh_name << "\" not found in file "
                  << m_filename << std::endl;
        exit(1);
      }

      //Compute total number of cell nodes for this mesh input
      size_t num_nodes = mesh->nnodes;
      if (mesh->gnodeno == NULL)
      {
        std::cerr << "Error: Mesh \"" << m_filename
                  << ":" << mesh_name << "\" has no global node ids"
                  << std::endl;
        exit(1);
      }
      for (size_t i=0; i< num_nodes; ++i)
      {
        if (static_cast<int*>(mesh->gnodeno)[i] > max_node_id)
        {
          max_node_id = static_cast<int*>(mesh->gnodeno)[i];
        }
      }
      DBFreeUcdmesh(mesh);
      DBClose(file);
    }

    typedef static_array<size_t>   array_type;
    typedef array_view<array_type> array_view_type;
    array_type max_node_array(num_locs);
    array_view_type max_node_view(max_node_array);
    max_node_array[myid] = max_node_id;

    m_nnodes = map_reduce(identity<size_t>(), max<size_t>(), max_node_view) + 1;
  }

  size_t get_num_vertices() const
  {
    return m_nnodes;
  }

  //////////////////////////////////////////////////////////////////////
  /// @param mesh mesh data structure.
  //////////////////////////////////////////////////////////////////////
  template<typename MeshType>
  void operator()(MeshType& mesh)
  {
    typedef typename MeshType::vertex_array_type  vertex_array_type;
    typedef typename MeshType::edge_descriptor    edge_descriptor;
    typedef typename MeshType::vertex_property    cell_property_type;
    typedef typename MeshType::geom_vector_type   geom_vector_type;
    typedef boost::unordered_map<std::vector<size_t>, size_t> face_map_type;

    size_t pos = m_filename.find('.');
    std::string mesh_name = m_filename.substr(0, pos);
    size_t num_locs = get_num_locations();
    size_t myid = get_location_id();
    DBfile *file = NULL;
    DBucdmesh *mesh_file = NULL;
    // Open Silo File
    file = DBOpen(m_filename.c_str(), DB_HDF5, DB_READ);
    if (file)
    {
      mesh_file = DBGetUcdmesh(file, mesh_name.c_str());

      //Check if dimension of data structure and input file matches
      if (static_cast<size_t>(mesh_file->ndims) != MeshType::dim_value)
      {
        std::cerr << "Error: Dimension of mesh data structure does not "
                     "match dimension of mesh input file."
                  << m_filename << std::endl;
        exit(1);
      }

      //Fill vertex array with vertex coordinates
      vertex_array_type* vertex_array = mesh.get_vertex_array();
      size_t num_nodes = mesh_file->nnodes;
      for (size_t i=0; i<num_nodes; ++i)
      {
        geom_vector_type gvector = set_geom_vector<MeshType::dim_value-1>()
                                   (mesh_file->coords, geom_vector_type(), i);
        (*vertex_array)[static_cast<int*>(mesh_file->gnodeno)[i]] = gvector;
      }

      //Adding cells to graph
      size_t nzones = mesh_file->zones->nzones;
      for (size_t i=0; i<nzones; ++i)
      {
        mesh.add_vertex(static_cast<int*>(mesh_file->zones->gzoneno)[i],
                        cell_property_type());
      }

      rmi_fence(); //for add_vertex

      //Adding edges in mesh graph and creating map of boundary faces
      face_map_type face_map;
      face_map_type::iterator map_it;
      if (MeshType::dim_value == 3)
        extract_tetrahedron_zones(mesh_file, face_map, mesh);
      else
        extract_2d_zones(mesh_file, face_map, mesh);

      //Adding edges between locations
      typedef static_array<face_map_type>   face_map_array_type;
      typedef array_view<face_map_array_type> face_map_array_view_type;
      face_map_array_type face_map_array(num_locs);
      face_map_array_view_type face_map_array_vw(face_map_array);
      face_map_array_vw[myid] = face_map;

      face_map_type
      boundary_faces = map_reduce(identity<face_map_type>(),
                                  face_map_reduce_wf<MeshType>(&mesh),
                                  face_map_array_vw);

      //Add self-edges on the mesh boundary
      size_t num_faces = boundary_faces.size();
      size_t block_size = num_faces/num_locs;
      size_t rem = num_faces%num_locs;
      size_t first_face = (myid<rem) ? myid*(block_size+1)
                                     : rem*(block_size+1)+(myid-rem)*block_size;
      size_t open_last_face = (myid<rem) ? first_face+block_size+1
                                         : first_face+block_size;

      map_it = boundary_faces.begin();
      std::advance(map_it, first_face);
      face_map_type::iterator map_end_it = boundary_faces.begin();
      std::advance(map_end_it, open_last_face);
      for (;map_it!=map_end_it; ++map_it)
      {
        mesh.add_cell_face_async(
          edge_descriptor(map_it->second, map_it->second), map_it->first);
      }
      rmi_fence(); //for add_cell_face_async
      DBFreeUcdmesh(mesh_file);
      DBClose(file);
    }

  }
};

} // stapl namespace

#endif
