/*
// Copyright (c) 2000-2011, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/mesh/implicit_regular_mesh.hpp>
#include <stapl/containers/graph/mesh/geom_vector.hpp>
#include <stapl/domains/interval.hpp>
#include <iostream>
#include <stapl/utility/tuple.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <map>
#include <test/test_report.hpp>

using namespace stapl;

template <typename Normal>
struct has_normal
{
  typedef bool result_type;

  has_normal(const Normal& norm)
   : normal(norm)
  { }

  template <typename Vertex>
  bool operator()(Vertex v)
  {
    typename Vertex::adj_edge_iterator edge_it;
    for (edge_it=v.begin(); edge_it!=v.end(); ++edge_it)
    {
      if ((*edge_it).property().get_normal() == normal)
        return true;
    }
    return false;
  }

  void define_type(typer& t)
  {
    t.member(normal);
  }

private:
  Normal normal;

};

struct graph_traversal_cell_wf
{
  typedef std::map<size_t, size_t> result_type;

  template<typename Vertex>
  result_type operator()(Vertex v)
  {
    result_type target_map;
    target_map[v.descriptor()] = 1;
    return target_map;
  }
};

struct graph_traversal_edge_source_wf
{
  typedef std::map<size_t, size_t> result_type;

  template<typename Vertex>
  result_type operator()(Vertex v)
  {
    result_type target_map;
    typename Vertex::adj_edge_iterator edge_it;
    for (edge_it = v.begin(); edge_it != v.end(); ++edge_it)
    {
      ++target_map[(*edge_it).source()];
    }
    return target_map;
  }
};

struct graph_traversal_edge_target_wf
{
  typedef std::map<size_t, size_t> result_type;

  template<typename Vertex>
  result_type operator()(Vertex v)
  {
    result_type target_map;
    typename Vertex::adj_edge_iterator edge_it;
    for (edge_it = v.begin(); edge_it != v.end(); ++edge_it)
    {
      ++target_map[(*edge_it).target()];
    }
    return target_map;
  }
};

struct reduce_map_wf
{
  typedef std::map<size_t, size_t> result_type;

  result_type operator()(result_type m1, result_type m2)
  {
    result_type::iterator it;
    for (it = m2.begin(); it != m2.end(); ++it)
    {
      m1[(*it).first] += (*it).second;
    }
    return m1;
  }
};

template<int N>
bool test_implicit_regular_mesh
(const typename implicit_regular_mesh_container<N>::tuple_type& size)
{
  //Constructing implicit regular mesh
  typedef typename implicit_regular_mesh_view_type<N>::type mesh_view_type;
  mesh_view_type mesh_vw = make_implicit_regular_mesh_view<N>(size);

  //Test size()
  size_t size_value = tuple_ops::fold(size, 1, multiplies<size_t>());
  bool bsize = (mesh_vw.size() == size_value);

  typedef std::map<size_t, size_t> map_type;
  map_type::iterator map_it;
  size_t ct = 0;

  //Check cell ids
  bool bcells = true;
  map_type cell_map =
    map_reduce(graph_traversal_cell_wf(), reduce_map_wf(), mesh_vw);
  bcells = bcells && (cell_map.size() == mesh_vw.size());
  ct = 0;

  for (map_it = cell_map.begin(); map_it != cell_map.end(); ++map_it)
  {
    if ((map_it->first != ct) || (map_it->second != 1))
    {
      bcells = false;
      break;
    }

    ++ct;
  }

  //Check edge source ids
  bool bedge_sources = true;
  map_type source_map =
    map_reduce(graph_traversal_edge_source_wf(), reduce_map_wf(), mesh_vw);
  bedge_sources = bedge_sources && (source_map.size() == mesh_vw.size());
  ct = 0;
  for (map_it = source_map.begin(); map_it != source_map.end(); ++map_it)
  {
    if ((map_it->first != ct) || (map_it->second != 2*N))
    {
      bedge_sources = false;
      break;
    }
    ++ct;
  }

  //Check edge target ids
  bool bedge_targets = true;
  map_type target_map =
    map_reduce(graph_traversal_edge_target_wf(), reduce_map_wf(), mesh_vw);
  bedge_targets = bedge_targets && (target_map.size() == mesh_vw.size());
  ct = 0;
  for (map_it = target_map.begin(); map_it != target_map.end(); ++map_it)
  {
    if ((map_it->first != ct) || (map_it->second != 2*N))
    {
      bedge_targets = false;
      break;
    }
    ++ct;
  }

  //Test mesh view with a palgorithm
  typedef geom_vector<N, double> normal_t;
  normal_t normal;
  normal.x() = 1;
  size_t num_normals = count_if(mesh_vw, has_normal<normal_t>(normal));
  bool bpalgo = (num_normals == mesh_vw.size());

  return (bsize && bcells && bedge_sources && bedge_targets && bpalgo);
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc != 4)
  {
    std::cout << "Format: " << argv[0]
              << " num_cells_x num_cells_y num_cells_z" << std::endl;
    return EXIT_FAILURE;
  }

  size_t x = atoi(argv[1]);
  size_t y = atoi(argv[2]);
  size_t z = atoi(argv[3]);

  bool test_2d = test_implicit_regular_mesh<2>(make_tuple(x, y));
  bool test_3d = test_implicit_regular_mesh<3>(make_tuple(x, y, z));

  STAPL_TEST_REPORT((test_2d && test_3d), "Testing implicit regular mesh\t");

  return EXIT_SUCCESS;
}
