/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "test_util.h"

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/algorithms/pscc.hpp>
#include <stapl/containers/graph/algorithms/pscc_single.hpp>
#include <stapl/containers/graph/algorithms/pscc_schudy.hpp>
#include <stapl/containers/map/map.hpp>
#include <stapl/containers/map/map.hpp>
#include <stapl/views/map_view.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/containers/graph/generators/cycles.hpp>

template <typename VertexGIDType>
class test_pscc_vertex_property
{
 public:
  typedef VertexGIDType                         color_type;

 private:
  color_type      m_cc;

 public:
  test_pscc_vertex_property()
    : m_cc(stapl::index_bounds<color_type>::invalid()) {}

  color_type get_cc() const { return m_cc; }
  void set_cc(color_type c) { m_cc = c; }

  void define_type(stapl::typer& t)
  {
    t.member(m_cc);
  }
};

namespace stapl {
template <typename VertGID, typename Accessor>
class proxy<test_pscc_vertex_property<VertGID>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef test_pscc_vertex_property<VertGID>      target_t;
  typedef typename target_t::color_type           color_type;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc) { }

  operator target_t() const
  { return Accessor::read(); }
  proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }
  proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this; }

  void set_cc(color_type c)
  { Accessor::invoke(&target_t::set_cc, c); }
  color_type get_cc() const
  { return Accessor::const_invoke(&target_t::get_cc); }
}; //struct proxy
} //namespace stapl

namespace stapl {
//updates the map of used colors, which is a {K=size_t, V=(size_t, bool)}
// the boolean represents whether this color is unique over the SCC
class uniqueness_inserter
{
 private:
  size_t m_new_color;

 public:
  typedef void    result_type;

  uniqueness_inserter(size_t cc) : m_new_color(cc) {}

  template<typename MapPair, typename T>
  result_type operator()(MapPair& where, const T&) const
  {
    if (where.second.second)
      where.second.second = (where.second.first == m_new_color);
  }

  void define_type(typer& t)
  {
    t.member(m_new_color);
  }
};


//returns false if the cc is not valid for this node
// AND updates the map of used colors
struct in_correct_scc
{
 public:
  typedef bool    result_type;

 private:
  size_t m_size;

 public:
  in_correct_scc(size_t s)  : m_size(s) {}

  template<typename Vertex, typename MView>
  result_type operator()(Vertex v, MView& mv)
  {
    mv.insert(
      v.descriptor() / m_size,
       std::make_pair(v.property().get_cc(),true),
      uniqueness_inserter(v.property().get_cc())
    );

    return (v.descriptor()/m_size == v.property().get_cc()/m_size);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_size);
  }
};

} //namespace stapl


//checks the map for conflicts
struct unique_label
{
  typedef bool result_type;

  template <typename MapPair>
  result_type operator()(MapPair const& mp) const
  {
    return mp.second.second;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using namespace std;
  using namespace stapl;

  one_print("Testing strongly connected components...");

  typedef test_pscc_vertex_property<size_t>                 vertex_property;
  typedef stapl::multidigraph<vertex_property>              graph_type;
  typedef stapl::graph_view<graph_type>                     gview_type;

  if (argc == 1)
  {
    if (get_location_id() == 0)
      cout << "usage: " << argv[0]
        << " [0/1/2 dcscm/dcsc/schudy] [cycle count] [cycle size]" << endl;
    return EXIT_FAILURE;
  }
  size_t alg = (argc > 1) ? atoi(argv[1]) : 0;
  size_t ccs = (argc > 2) ? atoi(argv[2]) : 256;
  size_t sz  = (argc > 3) ? atoi(argv[3]) : 3;

  gview_type vw = generators::make_cycle_chain<gview_type>(ccs, sz, false);

  switch(alg)
  {
    case 0:   pscc(vw);           break;
    case 1:   pscc_single(vw);    break;
    case 2:   pscc_schudy(vw);    break;
    default:
      if (get_location_id() == 0)
        cerr << "No valid algorithm selected." << endl;
      return EXIT_FAILURE;
  }

  typedef size_t                                        map_key_type;
  typedef std::pair<size_t, bool>                       map_value_type;
  typedef stapl::map<map_key_type, map_value_type>      correctness_map;
  typedef stapl::indexed_domain<map_key_type>           correctness_domain;
  typedef stapl::map_view<correctness_map>              correctness_map_view;

  correctness_domain dom(0, get_num_locations()*ccs-1);
  correctness_map cmap(dom);
  correctness_map_view cmv(cmap);

  bool passed = stapl::map_reduce(
    in_correct_scc(sz), stapl::logical_and<bool>(), vw, make_repeat_view(cmv)
  );

  correctness_map_view cmv2(cmap);
  passed &= stapl::map_reduce(unique_label(), stapl::logical_and<bool>(), cmv2);

  one_print(passed);

  return EXIT_SUCCESS;
}

