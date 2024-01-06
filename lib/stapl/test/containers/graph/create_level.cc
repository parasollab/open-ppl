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
#include <stapl/containers/graph/generators/torus.hpp>

using namespace stapl;

struct lvl_property
{
  size_t               m_x;

  lvl_property()
    : m_x()
  { }

  lvl_property(size_t i)
    : m_x(i)
  { }

  void put (size_t x)
  { m_x = x; }

  size_t get () const
  { return m_x; }

  lvl_property operator+(lvl_property const& rhs) const
  { return rhs.m_x + this->m_x; }

  void define_type(typer& t)
  { t.member(m_x); }
};


namespace stapl {

template <class Accessor>
class proxy<lvl_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef lvl_property target_t;

public:

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t() const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  size_t get() const
  { return Accessor::const_invoke(&target_t::get); }

  void put(size_t x)
  { Accessor::invoke(&target_t::put, x); }
};

} // namespace stapl


class init_wf1
{
private:
  size_t m_num_locations;

public:
  init_wf1(size_t num_locations)
    : m_num_locations(num_locations)
  { }

  typedef void result_type;

  template <class Vertex, class E>
  void operator()(Vertex v, E e) const
  {
    e = v.descriptor() % m_num_locations;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_num_locations);
  }
};


struct init_wf2
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


struct check1_wf
{
  size_t m_size;
  size_t m_nlocs;

  check1_wf(size_t size, size_t nlocs)
    : m_size(size), m_nlocs(nlocs)
  { }

  typedef size_t result_type;

  template<typename Vertex>
  result_type operator()(Vertex v) const
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

    if (v.property().supervertex() != v.descriptor() % m_nlocs)
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
    t.member(m_nlocs);
  }
};


struct check2_wf
{
  size_t m_size;
  size_t m_nlocs;

  check2_wf(size_t size, size_t nlocs)
    : m_size(size), m_nlocs(nlocs)
  { }

  typedef bool result_type;

  template<typename Vertex>
  result_type operator()(Vertex v) const
  {
    bool passed = true;

    for (size_t i=0; i<v.property().children.size(); ++i)
      if (v.property().children[i] % m_nlocs != v.descriptor())
        passed = false;

    if (v.property().supervertex() != std::numeric_limits<size_t>::max())
      passed = false;

    return passed;
  }

  void define_type(typer& t)
  {
    t.member(m_size);
    t.member(m_nlocs);
  }
};


struct print_wf
{
  typedef void result_type;

  template<typename Vertex>
  void operator()(Vertex v)
  {
    std::cout << "  vertex-" << v.descriptor()
              << "\tchildren: ";
    for (size_t i=0; i<v.property().children.size(); ++i)
      std::cout << v.property().children[i] << ", ";
    std::cout << "\twt= " << v.property().property.get();

    if (v.property().supervertex() != std::numeric_limits<size_t>::max())
      std::cout << "\tsupervertex: " << v.property().supervertex();
    std::cout << "\tEdges: ";
    for (typename Vertex::adj_edge_iterator aei = v.begin();
         aei != v.end(); ++aei) {
      std::cout << (*aei).source() << "-" << (*aei).target()
                << " (" << (*aei).property().property.get() << ": ";
      for (size_t i=0; i<(*aei).property().children.size(); ++i) {
        stapl::edge_descriptor_impl<size_t> x = (*aei).property().children[i];
        std::cout << x.source() << "-"
                  << x.target() << ",";
      }
      std::cout << "),   ";
    }
    std::cout << std::endl;
  }
};


template <class GVW>
void test_core_graph(GVW vgraph, std::string s)
{

  typedef static_array<size_t> array_t;
  array_t group_array(vgraph.size());
  typedef array_view<array_t> array_view_t;
  array_view_t group_array_vw(group_array);


  map_func(init_wf2(), vgraph, group_array_vw);

  graph_external_property_map<typename GVW::view_container_type, size_t,
                              array_view_t>
    group_id_prop_map(vgraph.container(), group_array_vw);

  GVW vgraph1 = create_level(vgraph, group_id_prop_map,
                             std::plus<lvl_property>(),
                             std::plus<lvl_property>(), 512, true);

  array_t group_array2(vgraph1.size());
  array_view_t group_array2_vw(group_array2);

  map_func(init_wf1(stapl::get_num_locations()), vgraph1, group_array2_vw);

  graph_external_property_map<typename GVW::view_container_type, size_t,
                              array_view_t>
    group_id_prop_map2(vgraph1.container(), group_array2_vw);

  GVW vgraph2 = create_level(vgraph1, group_id_prop_map2,
                             std::plus<lvl_property>(),
                             std::plus<lvl_property>(), 512, true);


  one_print("Testing create_level\t\t\t");
  bool passed2 = map_reduce(
    check2_wf(vgraph.size(), stapl::get_num_locations()),
    stapl::logical_and<bool>(), vgraph2
  );

  size_t total_edges = map_reduce(
    check1_wf(vgraph.size(), stapl::get_num_locations()),
    stapl::plus<size_t>(), vgraph1
  );

  one_print(passed2 && total_edges==vgraph.num_edges() &&
            vgraph1.size() == 10 && vgraph2.size() == get_num_locations());

  stapl::rmi_fence();
}


void graph_test_static(size_t x, size_t y)
{
  typedef digraph<super_vertex_property<lvl_property>,
                  super_edge_property<lvl_property> > graph_t;

  typedef graph_view<graph_t> graph_view_t;
  graph_view_t vw = generators::make_torus<graph_view_t>(x, y);

  test_core_graph(vw, "static");
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

  graph_test_static(x, y);

  return EXIT_SUCCESS;
}
