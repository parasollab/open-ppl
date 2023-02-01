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
#include <stapl/containers/graph/algorithms/create_level_partial_info.hpp>
#include <stapl/containers/graph/generators/list.hpp>

using namespace stapl;

struct lvl_vertex_property
{
  size_t               m_x;

  lvl_vertex_property()
    : m_x()
  { }

  lvl_vertex_property(size_t i)
    : m_x(i)
  { }

  void put (size_t x)
  { m_x = x; }

  size_t get () const
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

  inline operator target_t() const
  { return Accessor::read(); }

  inline proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  inline proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  inline size_t get() const
  { return Accessor::const_invoke(&target_t::get); }

  inline void put(size_t x)
  { Accessor::invoke(&target_t::put, x); }
};

} // namespace stapl


struct init_wf
{
  size_t m_size;

  typedef void result_type;

  init_wf(size_t const& size)
    : m_size(size)
  { }

  template <class Vertex, class E>
  void operator()(Vertex v, E e)
  {
    e = v.descriptor();
    v.property().property = 1;
    for (typename Vertex::adj_edge_iterator aei = v.begin();
         aei != v.end(); ++aei) {
      (*aei).property().property.put(1);
    }
    if (v.descriptor() !=  m_size-1)
      e = v.descriptor()+1;
  }

  void define_type(typer& t)
  {t.member(m_size); }
};


struct check_wf
{
  size_t m_size;

  check_wf(size_t size)
    : m_size(size)
  { }

  typedef bool result_type;

  template<typename Vertex>
  result_type operator()(Vertex v)
  {
    bool passed = true;
    size_t subedges = 0;

    if (v.property().children.size() != m_size)
        passed = false;
    if (!(v.property().property.get() == m_size))
      passed = false;

    if (v.property().supervertex() != std::numeric_limits<size_t>::max())
      passed = false;

    if (v.size() != 1)
      passed = false;

    for (typename Vertex::adj_edge_iterator aei = v.begin();
         aei != v.end(); ++aei) {
      if ((*aei).property().property.get() != (*aei).property().children.size())
        passed = false;
      subedges += (*aei).property().children.size();
    }

    if (subedges != 2*(m_size-1))
      passed = false;
    return passed;
  }

  void define_type(typer& t)
  { t.member(m_size); }
};


struct print_wf
{
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
      for (size_t i=0; i<(*aei).property().children.size(); ++i)
        std::cout << (*aei).property().children[i].source() << "-"
                  << (*aei).property().children[i].target() << ",";
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

  map_func(init_wf(vgraph.size()), vgraph, group_array_vw);

  graph_external_property_map<
    typename GVW::view_container_type, size_t, array_view_t>
    group_id_prop_map(vgraph.container(), group_array_vw);

  GVW vgraph1 = create_level_partial_info(vgraph, group_id_prop_map,
                                          std::plus<lvl_vertex_property>(),
                                          std::plus<lvl_vertex_property>(),
                                          512, true);


  one_print("Testing create_level (partial mapping)\t");

  bool passed = ((vgraph1.num_edges() == 1) && (vgraph1.size() == 1));

  if (get_location_id() == 0)
    passed = passed && check_wf(vgraph.size())(vgraph1[0]);

  one_print(passed);

  stapl::rmi_fence();
}


void graph_test_static(size_t n)
{
  typedef digraph<super_vertex_property<lvl_vertex_property>,
                  super_edge_property<lvl_vertex_property> > graph_t;

  typedef graph_view<graph_t> graph_view_t;
  graph_view_t vw = generators::make_list<graph_view_t>(n);

  test_core_graph(vw, "static");
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t n;
  if (argc > 1) {
    n = atol(argv[1]);
  } else {
    std::cout << "usage: exe n\n";
    return EXIT_FAILURE;
  }

  graph_test_static(n);

  return EXIT_SUCCESS;
}
