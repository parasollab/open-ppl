/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//STAPL includes:
#include "test_util.h"
#include <stapl/containers/graph/digraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>
#include <stapl/containers/graph/algorithms/create_level.hpp>
#include <stapl/containers/graph/algorithms/create_hierarchy.hpp>
#include <stapl/containers/graph/generators/torus.hpp>

using namespace stapl;

struct lvl_vertex_property
{
  size_t               m_x;

  lvl_vertex_property(void)
    : m_x()
  { }

  lvl_vertex_property(size_t i)
    : m_x(i)
  { }

  void put(size_t x)
  { m_x = x; }

  size_t get(void) const
  { return m_x; }

  lvl_vertex_property operator+(lvl_vertex_property const& rhs) const
  { return rhs.m_x + this->m_x; }

  void define_type(typer& t)
  { t.member(m_x); }
};


namespace stapl {

template <class Accessor>
class proxy<lvl_vertex_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef lvl_vertex_property target_t;

public:

  inline explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  inline operator target_t(void) const
  { return Accessor::read(); }

  inline proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  inline proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  inline size_t get(void) const
  { return Accessor::const_invoke(&target_t::get); }

  inline void put(size_t x)
  { Accessor::invoke(&target_t::put, x); }
};

} // namespace stapl


//////////////////////////////////////////////////////////////////////
/// @brief Workfunction to assign leader-mappings to each vertex for level-1
///
/// Creates 10 groups.
//////////////////////////////////////////////////////////////////////
struct init_wf1
{
  typedef void result_type;
  template <class Vertex, class E>
  void operator()(Vertex v, E e)
  {
    e = v.descriptor()%10;
    v.property().property = 1;
    for (typename Vertex::adj_edge_iterator aei = v.begin();
         aei != v.end(); ++aei)
      (*aei).property().property.put(1);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Workfunction to assign leader-mappings to each vertex for level-2
///
/// Creates p groups, one for each location.
//////////////////////////////////////////////////////////////////////
struct init_wf2
{
  size_t m_num_locations;

  init_wf2(size_t num_locs)
    : m_num_locations(num_locs)
  { }

  typedef void result_type;
  template <class Vertex, class E>
  void operator()(Vertex v, E e)
  { e = v.descriptor()%m_num_locations; }

  void define_type(typer& t)
  {
    t.member(m_num_locations);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Workfunction to check level-1 hierarchy
//////////////////////////////////////////////////////////////////////
struct check1_wf
{
  size_t m_size;
  size_t m_num_locations;

  check1_wf(size_t size, size_t num_locs)
    : m_size(size), m_num_locations(num_locs)
  { }

  typedef size_t result_type;

  template<typename Vertex>
  result_type operator()(Vertex v)
  {
    bool passed = true;
    size_t subedges = 0;
    size_t nparts = 10;

    for (size_t i=0; i<v.property().children.size(); ++i)
      if (v.property().children[i] %10 != v.descriptor())
        passed = false;
    if (!(v.property().property.get() == m_size/nparts ||
          v.property().property.get() == m_size/nparts + 1) )
      passed = false;

    if (v.property().supervertex() != v.descriptor()%m_num_locations)
      passed = false;

    for (typename Vertex::adj_edge_iterator aei = v.begin();
         aei != v.end(); ++aei) {
      if ((*aei).property().property.get() != (*aei).property().children.size())
        passed = false;
      subedges += (*aei).property().children.size();
    }

    if (passed)
      return subedges;
    else return 0;
  }

  void define_type(typer& t)
  {
    t.member(m_size);
    t.member(m_num_locations);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Workfunction to check level-2 hierarchy
//////////////////////////////////////////////////////////////////////
struct check2_wf
{
  size_t m_size;
  size_t m_num_locations;

  check2_wf(size_t size, size_t num_locs)
    : m_size(size), m_num_locations(num_locs)
  { }

  typedef bool result_type;

  template<typename Vertex>
  result_type operator()(Vertex v)
  {
    bool passed = true;
    size_t nparts = m_num_locations;

    for (size_t i=0; i<v.property().children.size(); ++i)
      if (v.property().children[i] %nparts != v.descriptor())
        passed = false;

    if (v.property().supervertex() != std::numeric_limits<size_t>::max())
      passed = false;

    return passed;
  }

  void define_type(typer& t)
  {
    t.member(m_size);
    t.member(m_num_locations);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Workfunction to check hierarchy-flattening
//////////////////////////////////////////////////////////////////////
struct flat_comp_wf
{
  size_t m_start;

  flat_comp_wf(size_t start = 0)
    : m_start(start)
  { }

  typedef std::pair<bool, size_t> result_type;

  template<typename Vertex1, typename Vertex2>
  result_type operator()(Vertex1 v1, Vertex2 v2)
  {
    bool passed = v1.property().property.get() == v2.property().get();
    // the flat vertex should have at least as many edges as the hierarchical.
    if (v2.size() < v1.property().children.size())
      return std::make_pair(false, 1);

    // each of the parent-child ("true") edges in the flat vertex should be
    // present in the hierarchical vertex.
    std::vector<size_t> children(v1.property().children);
    for (typename Vertex2::adj_edge_iterator aei = v2.begin();
         aei != v2.end(); ++aei) {
      if ((*aei).property()) {
        if (std::find(children.begin(), children.end(),
                      (*aei).target() - m_start) == children.end())
        passed = false;
      }
    }
    return std::make_pair(passed, 1);
  }

  void define_type(typer& t)
  { t.member(m_start); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Reduce workfunction to check hierarchy-flattening
//////////////////////////////////////////////////////////////////////
struct total_and_wf
{
  typedef std::pair<bool, size_t> result_type;

  template<typename T>
  result_type operator()(T t1, T t2)
  {
    return std::make_pair(t1.first && t2.first, t1.second + t2.second);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Coarsener for creating hierarchical levels.
//////////////////////////////////////////////////////////////////////
struct coarsener
{
  typedef static_array<size_t> array_t;
  typedef array_view<array_t>  array_view_t;

  template<typename GraphView>
  std::pair<bool,
            graph_external_property_map<GraphView, size_t, array_view_t> >
    operator()(GraphView vgraph, size_t level) const
  {
    /// This array is destroyed by the destructor of the final copy of
    /// the array_view returned in the pair.
    array_t* group_array = new array_t(vgraph.size());
    if (level == 0)
      map_func(init_wf1(), vgraph, array_view_t(*group_array));
    else if (level == 1)
      map_func(init_wf2(get_num_locations()),
               vgraph, array_view_t(*group_array));

    return
      std::make_pair(
        true,
        graph_external_property_map<GraphView, size_t, array_view_t>(
          vgraph, array_view_t(group_array)));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Workfunction to indicate termination when creating
/// hierarchical levels.
//////////////////////////////////////////////////////////////////////
struct done
{
  size_t m_max_levels;

  done(size_t max_levels)
    : m_max_levels(max_levels)
  { }

  template<typename GraphView>
  bool operator() (GraphView g, size_t level) const
  { return level >= m_max_levels; }
};


template<typename Hierarchy, typename Graph>
void test_flatten_hierarchy(Hierarchy const& hierarchy, Graph const& vgraph)
{
  one_print("Testing flatten_hierarchy\t\t");

  typedef graph<DIRECTED, NONMULTIEDGES,
                lvl_vertex_property, bool>                  flat_graph_t;
  typedef graph_view<flat_graph_t>                          flat_graph_view_t;
  typedef flat_graph_view_t::domain_type                    dom_t;

  flat_graph_view_t flat_graph = flatten_hierarchy(hierarchy);

  bool passed = flat_graph.size() == vgraph.size() + 10 + get_num_locations();

  size_t total = 0;
  for (int i=hierarchy.size()-1; i>=0; --i) {
    std::pair<bool, size_t> x
      = map_reduce(flat_comp_wf(total+hierarchy[i].size()), total_and_wf(),
                   hierarchy[i],
                   flat_graph_view_t(flat_graph.container(),
                                     dom_t(total,
                                           total+hierarchy[i].size()-1)));
    total+=hierarchy[i].size();
    if (!(x.first && x.second == hierarchy[i].size()))
      passed = false;
  }

  one_print(passed);

  stapl::rmi_fence();
}

void test_create_level(size_t x, size_t y)
{
  typedef digraph<super_vertex_property<lvl_vertex_property>,
                  super_edge_property<lvl_vertex_property> > graph_t;

  one_print("Testing create_level (coarsener)\t");

  typedef graph_view<graph_t> graph_view_t;
  std::vector<graph_view_t> h;
  graph_view_t vgraph = generators::make_torus<graph_view_t>(x, y);
  h.push_back(vgraph);

  vgraph = create_level(h[0], coarsener()(h[0], 0),
                        stapl::plus<lvl_vertex_property>(),
                        stapl::plus<lvl_vertex_property>(),
                        512, true);
  h.push_back(vgraph);

  vgraph = create_level(h[1], coarsener()(h[1], 1),
                        stapl::plus<lvl_vertex_property>(),
                        stapl::plus<lvl_vertex_property>(),
                        512, true);
  h.push_back(vgraph);


  bool passed2 = map_reduce(check2_wf(h[0].size(), get_num_locations()),
                            stapl::logical_and<bool>(), h[2]);

  size_t total_edges = map_reduce(check1_wf(h[0].size(), get_num_locations()),
                                  stapl::plus<size_t>(), h[1]);

  one_print(passed2 &&
            total_edges==h[0].num_edges() &&
            h[1].size() == 10 && h[2].size() == get_num_locations());

  test_flatten_hierarchy(h, h[0]);

  stapl::rmi_fence();
}


void test_create_hierarchy(size_t x, size_t y)
{
  typedef digraph<super_vertex_property<lvl_vertex_property>,
                  super_edge_property<lvl_vertex_property> > graph_t;

  typedef graph_view<graph_t> graph_view_t;
  graph_view_t vgraph = generators::make_torus<graph_view_t>(x, y);

  one_print("Testing create_hierarchy\t\t");

  std::vector<graph_view_t> hierarchy
    = create_hierarchy(vgraph, coarsener(),
                       stapl::plus<lvl_vertex_property>(),
                       stapl::plus<lvl_vertex_property>(),
                       done(2), 512, true);

  bool passed = hierarchy.size() == 3;
  bool passed2 = false;
  size_t total_edges = 0;
  if (passed) {
    passed2 = map_reduce(check2_wf(vgraph.size(), get_num_locations()),
                         stapl::logical_and<bool>(), hierarchy[2]);

    total_edges = map_reduce(check1_wf(vgraph.size(), get_num_locations()),
                             stapl::plus<size_t>(), hierarchy[1]);
  }

  one_print(passed && passed2 && total_edges==vgraph.num_edges() &&
            hierarchy[1].size() == 10 &&
            hierarchy[2].size() == get_num_locations());

  test_flatten_hierarchy(hierarchy, vgraph);

  stapl::rmi_fence();
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t x,y;
  if (argc > 1) {
    x = atol(argv[1]);
    y = atol(argv[2]);
  } else {
    std::cout << "usage: exe x y\n";
    return EXIT_FAILURE;
  }

  test_create_level(x, y);
  test_create_hierarchy(x, y);

  return EXIT_SUCCESS;
}
