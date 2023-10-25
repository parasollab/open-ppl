/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>
#include <stapl/containers/graph/algorithms/graph_coloring.hpp>
#include <stapl/containers/graph/generators/torus.hpp>
#include <test/test_report.hpp>
#include <iostream>

using namespace stapl;

class access_color
{
public:
  typedef size_t value_type;

  template <typename VProperty>
  value_type get(VProperty const& vp)
  {
    return vp.get_color();
  }

  template <typename VProperty>
  void put(VProperty vp, value_type const& color)
  {
    return vp.set_color(color);
  }
};


class my_vertex_property
{
 public:
  typedef size_t color_value_type;

 private:
  color_value_type   m_color;

 public:
  my_vertex_property(size_t i = 0)
    : m_color(i)
  { }

  my_vertex_property(my_vertex_property const& other) = default;

  color_value_type get_color() const
  { return m_color; }

  void set_color(color_value_type const& c)
  { m_color = c; }

  void define_type(typer& t)
  { t.member(m_color); }
};


namespace stapl {
template <typename Accessor>
class proxy<my_vertex_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef my_vertex_property target_t;

public:
  typedef size_t value_type;

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

  size_t get_color() const
  {
    return Accessor::const_invoke(&target_t::get_color);
  }

  void set_color(size_t const& c)
  {
    Accessor::invoke(&target_t::set_color, c);
  }
}; //struct proxy
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc!=3)
  {
    std::cout << "Format: " << argv[0]
              << " num_x num_y" << std::endl;
    exit(1);
  }

  typedef dynamic_graph<DIRECTED, MULTIEDGES, my_vertex_property> graph_type;
  typedef graph_view<graph_type> view_type;

  size_t num_x = atol(argv[1]);
  size_t num_y = atol(argv[2]);
  view_type gview = generators::make_torus<view_type>(num_x, num_y, false);

  typedef graph_internal_property_map<view_type,access_color> pmap_type;
  pmap_type pmap(gview);

  view_type gview1(gview.container());

  //Color graph
  color_graph(gview1, pmap);

  bool is_valid = is_valid_graph_coloring(gview, pmap);
  STAPL_TEST_REPORT(is_valid,"Testing graph coloring\t");

  return EXIT_SUCCESS;
}
