/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_MESH_GENERATE_OUTPUT_MESH_FILE_HPP
#define STAPL_CONTAINERS_GRAPH_MESH_GENERATE_OUTPUT_MESH_FILE_HPP

#include <silo.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <boost/unordered_map.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/native_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor creating a silo multimesh file storing a partition of
/// a mesh.
/// @tparam PropertyMap type of property map storing values associated
/// to mesh cells.
/// @tparam VertexArray type of pArray storing the mesh vertices.
//////////////////////////////////////////////////////////////////////
template<typename PropertyMap, typename VertexArray>
struct create_submesh_file_wf
{
private:
  PropertyMap  m_partition_map;
  VertexArray* m_vertex_array;
  std::string  m_filename;
  bool         m_case_3d;

  //////////////////////////////////////////////////////////////////////
  /// @brief Extract the vertices of a triangle cell from the mesh.
  /// @param edge_begin begin iterator on the cell's faces.
  /// @param edge_end end iterator on the cell's faces.
  /// @return sequence of vertices of the triangle cell.
  //////////////////////////////////////////////////////////////////////
  template <typename IteratorType>
  std::vector<size_t> extract_vertices_triangle_zones(IteratorType edge_begin,
                                                IteratorType edge_end)
  {
    //extract one edge of
    std::vector<size_t> vect = edge_begin->property().get_vertex_ids();
    ++edge_begin;
    std::vector<size_t> vect2 = edge_begin->property().get_vertex_ids();
    std::vector<size_t>::iterator it = vect2.begin(), end_it = vect2.end();
    for (; it != end_it; ++it)
    {
      if (std::find(vect.begin(), vect.end(), *it) == vect.end())
      {
        vect.push_back(*it);
        break;
      }
    }
    return vect;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Extract the vertices of a tetrahedral cell from the mesh.
  /// @param edge_begin begin iterator on the cell's faces.
  /// @param edge_end end iterator on the cell's faces.
  /// @return sequence of vertices of the tetrahedral cell.
  //////////////////////////////////////////////////////////////////////
  template <typename IteratorType>
  std::vector<size_t>
  extract_vertices_tetrahedron_zones(IteratorType edge_begin,
                                     IteratorType edge_end)
  {
    //extract one edge of
    std::vector<size_t> vect = edge_begin->property().get_vertex_ids();
    ++edge_begin;
    std::vector<size_t> vect2 = edge_begin->property().get_vertex_ids();
    std::vector<size_t>::iterator it = vect2.begin(), end_it = vect2.end();
    for (; it != end_it; ++it)
    {
      if (std::find(vect.begin(), vect.end(), *it) == vect.end())
      {
        vect.push_back(*it);
        break;
      }
    }
    return vect;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Convert 2D geometric vector into a std vector.
  /// @param coord 2D geometric vector.
  /// @return std vector storing the coordinated of @p coord.
  //////////////////////////////////////////////////////////////////////
  template <typename ELEMENT>
  std::vector<ELEMENT> convert_geom_vector(geom_vector<2, ELEMENT> const&
                                             coord)
  {
    std::vector<ELEMENT> vect;
    vect.push_back(coord.x());
    vect.push_back(coord.y());
    return vect;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Convert 3D geometric vector into a std vector.
  /// @param coord 3D geometric vector.
  /// @return std vector storing the coordinated of @p coord.
  //////////////////////////////////////////////////////////////////////
  template <typename ELEMENT>
  std::vector<ELEMENT> convert_geom_vector(geom_vector<3, ELEMENT> const&
                                             coord)
  {
    std::vector<ELEMENT> vect;
    vect.push_back(coord.x());
    vect.push_back(coord.y());
    vect.push_back(coord.z());
    return vect;
  }

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param pmap property map storing values associated to mesh cells.
  /// @param varray pArray of mesh vertices.
  /// @param fname output file name.
  /// @param case_3d true if mesh is 3D.
  //////////////////////////////////////////////////////////////////////
  create_submesh_file_wf(PropertyMap pmap, VertexArray* varray,
                         std::string const& fname, bool case_3d)
    : m_partition_map(pmap), m_vertex_array(varray), m_filename(fname),
      m_case_3d(case_3d)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param submesh partition of the mesh.
  /// @param index partition identifier.
  //////////////////////////////////////////////////////////////////////
  template<typename SubMesh>
  void operator()(SubMesh const& submesh, size_t const& index)
  {
    // Open the Silo file
    char name[100];
    sprintf(name, "%s.%lu", m_filename.c_str(), index);
    DBfile *dbfile = NULL;
    dbfile = DBCreate(name, DB_CLOBBER, DB_LOCAL,
        NULL, DB_HDF5);
    if (dbfile == NULL)
    {
      std::cerr << "error: could not create Silo file!" << std::endl;
    }

    // Connectivity
    size_t mesh_size = submesh.size();
    std::vector<double> x_vect;
    std::vector<double> y_vect;
    std::vector<double> z_vect;
    std::vector<int> node_ids;
    boost::unordered_map<size_t, size_t> node_map;
    size_t node_index = 0;
    int *nodelist;
    if (m_case_3d)
      nodelist = new int [mesh_size*4];
    else
      nodelist = new int [mesh_size*3];
    int *gl_zone_ids = new int [mesh_size];
    int *partitionlist = new int [mesh_size];
    size_t nodelist_index = 0;
    size_t zone_index = 0;

    typedef typename VertexArray::value_type geom_vector_type;
    typename SubMesh::iterator it = submesh.begin(),
        end_it = submesh.end();
    for (; it!=end_it; ++it)
    {
      std::vector<size_t> cell_vertices;
      std::vector<size_t>::iterator vect_it;
      typename SubMesh::adj_edge_iterator edge_it;
      if (m_case_3d)
        cell_vertices = extract_vertices_tetrahedron_zones((*it).begin(),
                                                           (*it).end());
      else
      cell_vertices = extract_vertices_triangle_zones((*it).begin(),
                                                      (*it).end());
      for (vect_it=cell_vertices.begin();vect_it!=cell_vertices.end();++vect_it)
      {
        size_t nid = *vect_it;
        if (node_map.find(nid) == node_map.end())
        {
          node_ids.push_back(nid);
          node_map[nid] = node_index;
          geom_vector_type coord = (*m_vertex_array)[nid];
          std::vector<double> coord_copy = convert_geom_vector(coord);
          x_vect.push_back(coord_copy[0]);
          y_vect.push_back(coord_copy[1]);
          if (m_case_3d)
            z_vect.push_back(coord_copy[2]);
          ++node_index;
        }

        nodelist[nodelist_index] = node_map[nid];
        ++nodelist_index;
      }
      gl_zone_ids[zone_index] = (*it).descriptor();
      partitionlist[zone_index] =
      m_partition_map.get(*it);
      ++zone_index;
    }

    int lnodelist;
    if (m_case_3d)
      lnodelist = mesh_size*4;
    else
      lnodelist = mesh_size*3;
    // shape type 1 has 3 nodes (tri)
    int shapesize[1];
    if (m_case_3d)
      shapesize[0] = 4;
    else
      shapesize[0] = 3;
    // We have mesh_size tris
    int shapecounts[] = {static_cast<int>(mesh_size)};
    int nshapetypes = 1;
    int nnodes = x_vect.size();
    int nzones = mesh_size;
    int ndims;
    if (m_case_3d)
      ndims = 3;
    else
      ndims = 2;
    int zshapetype[1];
    if (m_case_3d)
      zshapetype[0] = DB_ZONETYPE_TET;
    else
      zshapetype[0] = DB_ZONETYPE_TRIANGLE;
    double *x_block = new double [nnodes];
    double *y_block = new double [nnodes];
    double *z_block = NULL;
    if (m_case_3d)
      z_block = new double [nnodes];
    int *gl_node_ids = new int [nnodes];
    double **coords_block;
    if (m_case_3d)
      coords_block = new double* [3];
    else
      coords_block = new double* [2];
    coords_block[0] = x_block;
    coords_block[1] = y_block;
    if (m_case_3d)
      coords_block[2] = z_block;
    memcpy( x_block, &x_vect[0], sizeof( double ) * nnodes );
    memcpy( y_block, &y_vect[0], sizeof( double ) * nnodes );
    if (m_case_3d)
      memcpy( z_block, &z_vect[0], sizeof( double ) * nnodes );
    memcpy( gl_node_ids, &node_ids[0], sizeof( int ) * nnodes );

    // Write out connectivity information.
    DBoptlist *zone_opt = DBMakeOptlist(1);
    DBAddOption(zone_opt, DBOPT_ZONENUM, gl_zone_ids);
    DBPutZonelist2(dbfile, "zonelist", nzones, ndims, nodelist, lnodelist,
                   0, 0, 0, zshapetype, shapesize, shapecounts, nshapetypes,
                   zone_opt);

    // Write an unstructured mesh.
    DBoptlist* mesh_opt = DBMakeOptlist(1);
    DBAddOption(mesh_opt, DBOPT_NODENUM, gl_node_ids);
    DBPutUcdmesh(dbfile, m_filename.c_str(), ndims, NULL, coords_block, nnodes,
                 nzones, "zonelist", NULL, DB_DOUBLE, mesh_opt);

    // Write partition information.
    DBPutUcdvar1(dbfile, "partition_info", m_filename.c_str(), partitionlist,
                 nzones, NULL, 0, DB_INT, DB_ZONECENT, NULL);

    // Free data structures.
    DBFreeOptlist(zone_opt);
    DBFreeOptlist(mesh_opt);
    delete[] nodelist;
    delete[] x_block;
    delete[] y_block;
    if (m_case_3d)
      delete[] z_block;
    delete[] coords_block;
    delete[] gl_node_ids;
    delete[] gl_zone_ids;
    delete[] partitionlist;

    // Close the Silo file.
    DBClose(dbfile);
  }

  void define_type(typer& t)
  {
    t.member(m_partition_map);
    t.member(m_vertex_array);
    t.member(m_filename);
    t.member(m_case_3d);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Create silo master multimesh file.
/// @param filename output mesh file name.
/// @param num_submeshes number of mesh partitions.
//////////////////////////////////////////////////////////////////////
void create_master_mesh_file(std::string const& filename,
                             size_t const& num_submeshes)
{
  if (get_location_id() != 0)
    return;

  //Master file
  DBfile *dbfile = NULL;
  char **meshnames = NULL, **varnames;
  int dom, nmesh = num_submeshes, *meshtypes = NULL, *vartypes = NULL;

  // Create the list of mesh names and var names.
  meshnames = (char **)malloc(nmesh * sizeof(char *));
  varnames = (char **)malloc(nmesh * sizeof(char *));
  for (dom = 0; dom < nmesh; ++dom)
  {
    char tmp[100];
    sprintf(tmp, "%s.%d:%s", filename.c_str(), dom, filename.c_str());
    meshnames[dom] = strdup(tmp);
    sprintf(tmp, "%s.%d:partition_info", filename.c_str(), dom);
    varnames[dom] = strdup(tmp);
    //cout << "Master: " << meshnames[dom] << endl;
  }

  // Create the list of mesh types and var types.
  meshtypes = (int *)malloc(nmesh * sizeof(int));
  vartypes = (int *)malloc(nmesh * sizeof(int));
  for (dom = 0; dom < nmesh; ++dom)
  {
    meshtypes[dom] = DB_UCDMESH;
    vartypes[dom] = DB_UCDVAR;
  }

  // Open the Silo file
  std::string name = filename;
  name += ".root";
  dbfile = DBCreate(name.c_str(), DB_CLOBBER, DB_LOCAL,
                    NULL, DB_HDF5);

  // Write the multimesh.
  DBPutMultimesh(dbfile, filename.c_str(), nmesh, meshnames, meshtypes, NULL);

  // Write the multivar.
  DBPutMultivar(dbfile, "partition_info", nmesh, varnames, vartypes, NULL);

  // Close the Silo file.
  DBClose(dbfile);

  // Free the memory.
  for (dom = 0; dom < nmesh; ++dom)
  {
    free(meshnames[dom]);
    free(varnames[dom]);
  }
  free(meshnames);
  free(varnames);
  free(meshtypes);
  free(vartypes);
}


//////////////////////////////////////////////////////////////////////
/// @brief Output a mesh to silo multimesh files.
/// @param mesh mesh data structure.
/// @param partition_weight_map property map storing values associated to
/// mesh cells.
/// @param filename output mesh file name.
//////////////////////////////////////////////////////////////////////
template<typename MeshType, typename PropertyMap>
void generate_silo_file(MeshType& mesh, PropertyMap partition_weight_map,
                        std::string filename)
{
  bool case_3d = false;
  if (MeshType::dim_value == 3)
    case_3d = true;

  //Create native view over the mesh
  typedef typename MeshType::vertex_array_type vertex_array_type;
  typedef graph_view<MeshType> mesh_view_type;
  mesh_view_type mesh_view(mesh);
  typename result_of::native_view<mesh_view_type>::type
    native_vw = native_view(mesh_view);

  size_t num_submeshes = native_vw.size();

  //create subfiles in parallel
  map_func(create_submesh_file_wf<PropertyMap, vertex_array_type>(
             partition_weight_map,mesh.get_vertex_array(), filename, case_3d),
           native_vw,
           counting_view<size_t>(num_submeshes));

  //create master file for multimesh
  create_master_mesh_file(filename, num_submeshes);
}


} // stapl namespace

#endif
