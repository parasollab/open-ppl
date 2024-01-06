/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_SWEEP_UTILS_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_SWEEP_UTILS_HPP

#include <benchmarks/sweep/sweep.hpp>
#include <benchmarks/sweep/sweep_property.hpp>
#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/generators/grid.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/utility/do_once.hpp>
#include <test/containers/graph/test_util.h>
#include <sstream>

namespace stapl {

namespace sweep_utils {

template <typename D>
std::string dimension_to_str(D const& d)
{
  std::stringstream ss;
  for (auto const& e : d) {
    ss << e << ".";
  }
  return ss.str();
}

struct print_level_wf
{
  size_t m_size;

  print_level_wf(size_t const& size)
    : m_size(size)
  {}

  typedef void result_type;
  template <typename V>
  void operator() (V&& v)
  {
    auto const& values = v.property().sweep_values();
    std::stringstream ss;
    ss << "vertex: " << v.descriptor() << " values: ";
    for (auto const& e : values) {
      ss << "<" << e.first << ", {";
      for (auto const& d : e.second) {
        if (d.second < m_size)
          ss << "(" << dimension_to_str(d.first)
             << "," << d.second << "), ";
      }
      ss << "}>,  ";
    }
    std::cout << ss.str() << std::endl;
  }

  void define_type(typer& t)
  { t.member(m_size); }
};

struct print_graph_wf
{
  typedef void result_type;
  template <typename V>
  void operator() (V&& v)
  {
    auto coords = v.property().get_coordinates();
    std::stringstream ss;
    ss << "vertex: " << v.descriptor() << " coords {"
       << dimension_to_str(coords) << "}, edges:  ";
    for (auto const& e : v.edges()) {
      ss << e.target() << " {" << dimension_to_str(e.property()) << "}, ";
    }
    std::cout << ss.str() << std::endl;
  }
};

template <typename SweepValues, typename DirectionT>
bool check(SweepValues const& values,
           std::vector<DirectionT> directions,
           DirectionT const& direction,
           size_t const& distance) {
  auto index_of_dir = std::distance(directions.begin(),
                                    std::find(directions.begin(),
                                              directions.end(), direction));

  auto it = values.find(index_of_dir);
  // Check if vertex is reachable from source.
  if (it == values.end() || it->second.empty()) {
    return false;
  }
  auto it2 = it->second.find(direction);
  // Check if vertex is reachable from source in given direction.
  if (it2 == it->second.end()) {
    return false;
  }
  // Check if distance from source in given direction is correct.
  if (it2->second != distance) {
    return false;
  }
  return true;
}

struct check_level_2D_wf
{
  size_t m_nx, m_ny;
  std::vector<std::vector<float>> m_directions;

  check_level_2D_wf(size_t const& nx, size_t const& ny,
                    std::vector<std::vector<float>> const& directions)
    : m_nx(nx), m_ny(ny), m_directions(directions) {}

  typedef bool result_type;
  template <typename V>
  bool operator() (V&& v)
  {
    auto const& values = v.property().sweep_values();

    // all vertices should be reachable in direction (1,1).
    if (!check(values, m_directions, {1,1},
               v.descriptor()/m_nx + v.descriptor()%m_nx))
      return false;

    // all vertices should be reachable in direction (-1,1).
    if (!check(values, m_directions, {-1,1},
               v.descriptor()/m_nx + (m_nx-1)-(v.descriptor()%m_nx)))
      return false;

    // all vertices should be reachable in direction (1,-1).
    if (!check(values, m_directions, {1,-1},
               (m_ny-1)-v.descriptor()/m_nx + (v.descriptor()%m_nx)))
      return false;

    // all vertices should be reachable in direction (-1,-1).
    if (!check(values, m_directions, {-1,-1},
               (m_ny-1)-v.descriptor()/m_nx + (m_nx-1)-(v.descriptor()%m_nx)))
      return false;

    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_nx);
    t.member(m_ny);
  }
};

struct check_level_3D_wf
{
  size_t m_nx, m_ny, m_nz;
  std::vector<std::vector<float>> m_directions;

  check_level_3D_wf(size_t const& nx, size_t const& ny, size_t const& nz,
                    std::vector<std::vector<float>> const& directions)
    : m_nx(nx), m_ny(ny), m_nz(nz), m_directions(directions) {}

  typedef bool result_type;
  template <typename V>
  bool operator() (V&& v)
  {
    auto const& values = v.property().sweep_values();

    auto plane_descriptor = v.descriptor()%(m_nx*m_ny);

    // all vertices should be reachable in direction (1,1,1).
    if (!check(values, m_directions, {1,1,1},
               plane_descriptor/m_nx + plane_descriptor%m_nx
               + v.descriptor()/(m_nx*m_ny))) {
      return false;
    }

    // all vertices should be reachable in direction (-1,1,1).
    if (!check(values, m_directions, {-1,1,1},
               plane_descriptor/m_nx + (m_nx-1)-(plane_descriptor%m_nx)
               + v.descriptor()/(m_nx*m_ny))) {
      return false;
    }

    // all vertices should be reachable in direction (1,-1,1).
    if (!check(values, m_directions, {1,-1,1},
               (m_ny-1)-plane_descriptor/m_nx + (plane_descriptor%m_nx)
               + v.descriptor()/(m_nx*m_ny))) {
      return false;
    }

    // all vertices should be reachable in direction (-1,-1,1).
    if (!check(values, m_directions, {-1,-1,1},
               (m_ny-1)-plane_descriptor/m_nx + (m_nx-1)-(plane_descriptor%m_nx)
               + v.descriptor()/(m_nx*m_ny))) {
      return false;
    }

    // all vertices should be reachable in direction (1,1,-1).
    if (!check(values, m_directions, {1,1,-1},
               plane_descriptor/m_nx + plane_descriptor%m_nx
               + (m_nz-1)-v.descriptor()/(m_nx*m_ny))) {
      return false;
    }

    // all vertices should be reachable in direction (-1,1,-1).
    if (!check(values, m_directions, {-1,1,-1},
               plane_descriptor/m_nx + (m_nx-1)-(plane_descriptor%m_nx)
               + (m_nz-1)-v.descriptor()/(m_nx*m_ny))) {
      return false;
    }

    // all vertices should be reachable in direction (1,-1,-1).
    if (!check(values, m_directions, {1,-1,-1},
               (m_ny-1)-plane_descriptor/m_nx + (plane_descriptor%m_nx)
               + (m_nz-1)-v.descriptor()/(m_nx*m_ny))) {
      return false;
    }

    // all vertices should be reachable in direction (-1,-1,-1).
    if (!check(values, m_directions, {-1,-1,-1},
               (m_ny-1)-plane_descriptor/m_nx + (m_nx-1)-(plane_descriptor%m_nx)
               + (m_nz-1)-v.descriptor()/(m_nx*m_ny))) {
      return false;
    }
    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_nx);
    t.member(m_ny);
  }
};

struct set_coordinates_wf
{
  size_t m_nx, m_ny, m_3d;

  set_coordinates_wf(size_t const& nx, size_t const& ny, bool is_3d)
    : m_nx(nx), m_ny(ny), m_3d(is_3d) {}

  typedef void result_type;
  template <typename V>
  void operator() (V&& v)
  {
    auto z = v.descriptor()/(m_nx*m_ny);
    auto plane_descriptor = v.descriptor()%(m_nx*m_ny);
    auto y = plane_descriptor/m_nx;
    auto x = plane_descriptor%m_nx;
    if (m_3d)
      v.property().set_coordinates({float(x), float(y), float(z)});
    else
      v.property().set_coordinates({float(x), float(y)});
  }

  void define_type(typer& t)
  {
    t.member(m_nx);
    t.member(m_ny);
  }
};

struct set_edges_update_func
{
  size_t m_vd;
  std::vector<float> m_c;

  set_edges_update_func(size_t const& vd = 0,
                        std::vector<float> const& c = {0.0})
    : m_vd(vd), m_c(c) {}

  typedef bool result_type;

  template <class Vertex>
  bool operator()(Vertex&& target) const
  {
    target.property().add_incoming_coordinates(m_vd, m_c);
    return false;
  }

  void define_type(typer& t)
  {
    t.member(m_vd);
    t.member(m_c);
  }
};

struct set_edges_wf
{
  typedef bool result_type;
  template<typename Vertex, typename GraphVisitor>
  bool operator()(Vertex v, GraphVisitor graph_visitor) const
  {
    // If the first step, semd out coordinates to all neighbors.
    if (graph_visitor.level() == 1) {
      set_edges_update_func update_func(v.descriptor(),
                                        v.property().get_coordinates());
      for (typename Vertex::adj_edge_iterator it = v.begin(), it_e = v.end();
           it != it_e; ++it) {
        graph_visitor.visit((*it).target(), update_func);
      }
    } else {
      auto incoming_coordinates = v.property().get_incoming_coordinates();
      v.property().clear_incoming_coordinates();
      for (typename Vertex::adj_edge_iterator it = v.begin(), it_e = v.end();
           it != it_e; ++it) {
        auto mit = incoming_coordinates.find((*it).target());
        if (mit != incoming_coordinates.end()) {
          // Direction of the edge computed between vertex and neighbor.
          auto direction = v.property().get_coordinates();
          for (size_t i=0; i<direction.size(); ++i) {
            direction[i] = mit->second[i] - direction[i];
          }
          (*it).property() = direction;
        }
      }
    }
    return graph_visitor.level() == 1;
  }
};

struct init_edges_2D_wf
{
  size_t m_nx, m_ny;

  init_edges_2D_wf(size_t const& nx, size_t const& ny)
    : m_nx(nx), m_ny(ny) {}

  typedef void result_type;
  template <typename V>
  void operator() (V&& v)
  {
    for (typename V::adj_edge_iterator it = v.begin();
         it != v.end(); ++it) {
      auto source = (*it).source();
      auto target = (*it).target();
      if (target == source+1) {  // +x-neighbor
        (*it).property() = {1, 0};
      } else if (target == source-1) {  // -x-neighbor
        (*it).property() = {-1, 0};
      } else if (target > source+1) {  // +y-neighbor
        (*it).property() = {0, 1};
      } else if (target < source-1) {  // -y-neighbor
        (*it).property() = {0, -1};
      }
    }
  }

  void define_type(typer& t)
  {
    t.member(m_nx);
    t.member(m_ny);
  }
};

struct init_edges_3D_wf
{
  size_t m_nx, m_ny, m_nz;

  init_edges_3D_wf(size_t const& nx, size_t const& ny, size_t const& nz)
    : m_nx(nx), m_ny(ny), m_nz(nz) {}

  typedef void result_type;
  template <typename V>
  void operator() (V&& v)
  {
    for (typename V::adj_edge_iterator it = v.begin();
         it != v.end(); ++it) {
      auto source = (*it).source();
      auto target = (*it).target();
      if (target == source+1) {  // +x-neighbor
        (*it).property() = {1, 0, 0};
      } else if (target == source-1) {  // -x-neighbor
        (*it).property() = {-1, 0, 0};
      } else if (target == source+m_nx) {  // +y-neighbor
        (*it).property() = {0, 1, 0};
      } else if (target == source-m_nx) {  // -y-neighbor
        (*it).property() = {0, -1, 0};
      } else if (target == source+m_nx*m_ny) {  // +z-neighbor
        (*it).property() = {0, 0, 1};
      } else if (target == source-m_nx*m_ny) {  // -z-neighbor
        (*it).property() = {0, 0, -1};
      }
    }
  }

  void define_type(typer& t)
  {
    t.member(m_nx);
    t.member(m_ny);
    t.member(m_nz);
  }
};


struct degree_wf
{
  typedef size_t result_type;
  template <typename V>
  size_t operator() (V&& v)
  { return v.size(); }
};

struct id_wf
{
  size_t m_d;

  id_wf(size_t const& d)
    : m_d(d)
  {}

  typedef std::vector<size_t> result_type;
  template <typename V>
  std::vector<size_t> operator() (V&& v)
  {
    std::vector<size_t> result;
    if (v.size() == m_d)
      result.push_back(v.descriptor());
    return result;
  }

  void define_type(typer& t)
  { t.member(m_d); }
};

struct join
{
  typedef std::vector<size_t> result_type;
  template<typename Reference1, typename Reference2>
  result_type operator()(Reference1 x, Reference2 y) const
  {
    std::vector<size_t> value, xv(x), yv(y);
    for (auto const& e : xv)
      value.push_back(e);
    for (auto const& e : yv)
      value.push_back(e);
    return value;
  }
};

template <typename GView>
std::vector<size_t> find_corners(GView const& vw)
{
  size_t min_degree = map_reduce(degree_wf(), min<size_t>(), vw);
  std::vector<size_t> corners = map_reduce(id_wf(min_degree), join(), vw);
  return corners;
}

  ////////////////////////// READER ////////////////////////

struct set_property_wf
{
  std::vector<float> m_prop;
  typedef void result_type;

  set_property_wf(std::vector<float> const& prop)
    : m_prop(prop)
  { }


  template<typename P>
  void operator() (P& p) const
  { p.set_coordinates(m_prop); }

  void define_type(stapl::typer& t)
  { t.member(m_prop); }
};

struct read_sweep_input_line
{
  /// Lines have properties for edges
  using has_edge_property = std::true_type;

  template<typename Aggr, typename Graph>
  size_t operator() (std::stringstream& ss, Aggr& aggr, Graph* g) const
  {
    size_t vd;
    float vx, vy, vz;
    size_t dest;
    float ex, ey, ez;
    char junk; std::string demarc;

    // read the vertex and its property.
    ss >> vd >> junk >> junk >> vx >> junk >> vy >> junk >> vz >> junk >> junk
       >> demarc;

    g->vp_apply_async(vd, set_property_wf({vx, vy, vz}));
    while (!ss.eof()) {
      ss >> dest >> junk >> junk >> ex >> junk >> ey >> junk >> ez
         >> junk >> junk;

      aggr.add({typename Graph::edge_descriptor(vd, dest),
                std::vector<float>{ex, ey, ez}});
    }
    return 0;
  }
};

template<typename Graph>
graph_view<Graph> sweep_graph_reader(std::string filename)
{
  return sharded_graph_reader<Graph, read_sweep_input_line>(
      filename, read_sweep_input_line());
}


} //namespace sweep_utils
} //namespace stapl

#endif
