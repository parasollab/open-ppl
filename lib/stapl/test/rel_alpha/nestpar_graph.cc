/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <map>
#include <set>
#include <cstdio>
#include <string>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>
#include "../test_report.hpp"
#include <stapl/utility/tuple.hpp>
#include <stapl/views/map_view.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/containers/map/map.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/skeletons/serial.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>

using namespace std;
#include "testutil.hpp"

//#define OUTER_MAP

typedef void (* test_ptr) (int, int, stapl::stream<ifstream>&, int);


// Different combinations of types between Graphs, Vectors, Arrays, and Maps
// used to test nested parallelism in STAPL.

typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES,
                     int, stapl::properties::no_property>           d_graph_tp;
typedef stapl::array<size_t>                                            arr_tp;
typedef stapl::vector<int>                                              vec_tp;

typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES, d_graph_tp,
                      stapl::properties::no_property>         d_graph_graph_tp;
typedef stapl::array<arr_tp>                                        arr_arr_tp;
typedef stapl::vector<vec_tp>                                       vec_vec_tp;

typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES,
                     arr_tp, stapl::properties::no_property>    d_graph_arr_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES,
                     vec_tp, stapl::properties::no_property>    d_graph_vec_tp;

typedef stapl::array<d_graph_tp>                                d_arr_graph_tp;
typedef stapl::vector<d_graph_tp>                               d_vec_graph_tp;
typedef stapl::array<vec_tp>                                        arr_vec_tp;
typedef stapl::vector<arr_tp>                                       vec_arr_tp;

typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES, d_graph_graph_tp,
                        stapl::properties::no_property> d_graph_graph_graph_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES, arr_arr_tp,
                  stapl::properties::no_property>           d_graph_arr_arr_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES, vec_vec_tp,
                  stapl::properties::no_property>           d_graph_vec_vec_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES, d_graph_arr_tp,
                      stapl::properties::no_property>     d_graph_graph_arr_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES, d_graph_vec_tp,
                      stapl::properties::no_property>     d_graph_graph_vec_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES, d_arr_graph_tp,
                      stapl::properties::no_property>     d_graph_arr_graph_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES, d_vec_graph_tp,
                      stapl::properties::no_property>     d_graph_vec_graph_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES, vec_arr_tp,
                  stapl::properties::no_property>           d_graph_vec_arr_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES, arr_vec_tp,
                  stapl::properties::no_property>           d_graph_arr_vec_tp;

typedef stapl::array<d_graph_graph_tp>                    d_arr_graph_graph_tp;
typedef stapl::array<d_graph_arr_tp>                        d_arr_graph_arr_tp;
typedef stapl::array<d_graph_vec_tp>                        d_arr_graph_vec_tp;
typedef stapl::array<d_arr_graph_tp>                        d_arr_arr_graph_tp;
typedef stapl::array<d_vec_graph_tp>                        d_arr_vec_graph_tp;

typedef stapl::vector<d_graph_graph_tp>                   d_vec_graph_graph_tp;
typedef stapl::vector<d_graph_vec_tp>                       d_vec_graph_vec_tp;
typedef stapl::vector<d_graph_arr_tp>                       d_vec_graph_arr_tp;
typedef stapl::vector<d_vec_graph_tp>                       d_vec_vec_graph_tp;
typedef stapl::vector<d_vec_graph_tp>                       d_vec_arr_graph_tp;

// View Types
typedef stapl::graph_view<d_graph_tp>                            d_graph_vw_tp;
typedef stapl::array_view<arr_tp>                                    arr_vw_tp;
typedef stapl::vector_view<vec_tp>                                   vec_vw_tp;

typedef stapl::graph_view<d_graph_graph_tp>                d_graph_graph_vw_tp;
typedef stapl::array_view<arr_arr_tp>                            arr_arr_vw_tp;
typedef stapl::vector_view<vec_vec_tp>                           vec_vec_vw_tp;

typedef stapl::graph_view<d_graph_arr_tp>                    d_graph_arr_vw_tp;
typedef stapl::graph_view<d_graph_vec_tp>                    d_graph_vec_vw_tp;

typedef stapl::array_view<d_arr_graph_tp>                    d_arr_graph_vw_tp;
typedef stapl::vector_view<d_vec_graph_tp>                   d_vec_graph_vw_tp;

typedef stapl::array_view<arr_vec_tp>                            arr_vec_vw_tp;
typedef stapl::vector_view<vec_arr_tp>                           vec_arr_vw_tp;

typedef stapl::graph_view<d_graph_graph_graph_tp>    d_graph_graph_graph_vw_tp;
typedef stapl::graph_view<d_graph_arr_arr_tp>            d_graph_arr_arr_vw_tp;
typedef stapl::graph_view<d_graph_vec_vec_tp>            d_graph_vec_vec_vw_tp;
typedef stapl::graph_view<d_graph_graph_arr_tp>        d_graph_graph_arr_vw_tp;
typedef stapl::graph_view<d_graph_graph_vec_tp>        d_graph_graph_vec_vw_tp;
typedef stapl::graph_view<d_graph_arr_graph_tp>        d_graph_arr_graph_vw_tp;
typedef stapl::graph_view<d_graph_vec_graph_tp>        d_graph_vec_graph_vw_tp;
typedef stapl::graph_view<d_graph_vec_arr_tp>            d_graph_vec_arr_vw_tp;
typedef stapl::graph_view<d_graph_arr_vec_tp>            d_graph_arr_vec_vw_tp;

typedef stapl::array_view<d_arr_graph_graph_tp>        d_arr_graph_graph_vw_tp;
typedef stapl::array_view<d_arr_graph_arr_tp>            d_arr_graph_arr_vw_tp;
typedef stapl::array_view<d_arr_graph_vec_tp>            d_arr_graph_vec_vw_tp;
typedef stapl::array_view<d_arr_arr_graph_tp>            d_arr_arr_graph_vw_tp;
typedef stapl::array_view<d_arr_vec_graph_tp>            d_arr_vec_graph_vw_tp;

typedef stapl::vector_view<d_vec_graph_graph_tp>       d_vec_graph_graph_vw_tp;
typedef stapl::vector_view<d_vec_graph_vec_tp>           d_vec_graph_vec_vw_tp;
typedef stapl::vector_view<d_vec_graph_arr_tp>           d_vec_graph_arr_vw_tp;
typedef stapl::vector_view<d_vec_vec_graph_tp>           d_vec_vec_graph_vw_tp;
typedef stapl::vector_view<d_vec_vec_graph_tp>           d_vec_arr_graph_vw_tp;

// Map
typedef stapl::map<int,int>                                             map_tp;
typedef stapl::map< int, stapl::map<int,int> >                      map_map_tp;
typedef stapl::map_view<map_tp>                                      map_vw_tp;
typedef stapl::map_view<map_map_tp>                              map_map_vw_tp;
typedef stapl::indexed_domain<int>                                  ndx_dom_tp;

typedef stapl::dynamic_graph<stapl::DIRECTED, stapl::MULTIEDGES,
                     map_tp, stapl::properties::no_property>     dd_graph_m_tp;
typedef stapl::graph_view<dd_graph_m_tp>                      dd_graph_m_vw_tp;
typedef stapl::array<dd_graph_m_tp>                          dd_arr_graph_m_tp;
typedef stapl::vector<dd_graph_m_tp>                         dd_vec_graph_m_tp;
typedef stapl::vector_view<dd_vec_graph_m_tp>             dd_vec_graph_m_vw_tp;
typedef stapl::array_view<dd_arr_graph_m_tp>              dd_arr_graph_m_vw_tp;

typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES,
                     map_tp, stapl::properties::no_property>      d_graph_m_tp;
typedef stapl::graph_view<d_graph_m_tp>                        d_graph_m_vw_tp;
typedef stapl::array<d_graph_m_tp>                            d_arr_graph_m_tp;
typedef stapl::vector<d_graph_m_tp>                           d_vec_graph_m_tp;
typedef stapl::vector_view<d_vec_graph_m_tp>               d_vec_graph_m_vw_tp;
typedef stapl::array_view<d_arr_graph_m_tp>                d_arr_graph_m_vw_tp;


// Used to print test results, called by each test function
void print_results(stapl::tuple<bool, double> fin_res,
  int num_executions)
{
  bool passed = stapl::get<0>(fin_res);
  double exec_time = stapl::get<1>(fin_res) / num_executions;
  stapl::do_once([&](){
    if (passed == true)
      std::cerr << "PASSED";
    else
      std::cerr << "FAILED";
  });

  stapl::do_once([&](){
    std::cerr << "\t\t exec time: ";
  });

  stapl::do_once([&](){
    std::cerr << std::to_string(exec_time).c_str();
  });

  std::cerr << "\n";
}

// Populates the arrays with values used for the sizes of the inner most
// containers. Called by gen_sz_out and all two-level tests.
struct gen_sz_in
{
private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  typedef void result_type;
  gen_sz_in(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  template <typename View1>
  result_type operator()(View1 length)
  {
    m_zin >> length;
    length = length*m_procs;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_procs);
  }
};

// Calls gen_sz_in on each sizes array for the outer containers in cases
// where the nesting level of containers is more than two.
struct gen_sz_out
{
private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  gen_sz_out(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 length)
  {
    stapl::serial_io(gen_sz_in(m_zin, m_procs), length);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_procs);
  }
};

// Used to populate a map in the innermost level of a nested container.
struct fill_m
{
  private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  fill_m(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef int result_type;
  template <typename Pair>
  result_type operator()(Pair p)
  {
    int num;
    m_zin >> num;
    p.second = num;
    return num;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_procs);
  }
};

// Used to test proper assignment of an innermost map in a nested container.
struct test_m
{
  typedef int result_type;
  template <typename Pair>
  result_type operator()(Pair p)
  {
    return p.second;
  }
};

// Used to populate a graph in the innermost level of a nested container.
struct fill_g
{
private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  fill_g(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef int result_type;
  template <typename Vertex>
  result_type operator()(Vertex v)
  {
    int num;
    m_zin >> num;
    v.property() = num*m_procs;
    return num*m_procs;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_procs);
  }
};

// Used to test proper assignment of an innermost graph in a nested container.
struct test_g
{
  typedef int result_type;
  template <typename Vertex>
  result_type operator()(Vertex v)
  {
    return v.property();
  }
};

// Used to populate a vector or array in the innermost level of a nested
// container.
struct fill_x
{
private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  fill_x(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef int result_type;
  template <typename Element>
  result_type operator()(Element e)
  {
    int num;
    m_zin >> num;
    e = num*m_procs;
    return num*m_procs;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_procs);
  }
};

// Used to test proper assignment of an innermost vector or array
// in a nested container.
struct test_x
{
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element e)
  {
    return e;
  }
};

// Used to populate a graph(graph) in the second level of a nested
// container or as a nested container by itself.
struct fill_gg
{
private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  fill_gg(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef int result_type;
  template <typename Vertex>
  result_type operator()(Vertex v)
  {
    return stapl::map_reduce(fill_g(m_zin, m_procs),
      stapl::plus<int>(), v.property());
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_procs);
  }
};

// Used to test proper assignment of a graph(graph) in a nested container
// or as a nested container by itself.
struct test_gg
{
  typedef int result_type;
  template<typename Vertex>
  result_type operator()(Vertex v)
  {
    return stapl::map_reduce(test_g(), stapl::plus<int>(), v.property());
  }
};

// Used to populate a vector(array), array(vector), array(array) or
// vector(vector) in the second level of a nested container or as a nested
// container by itself.
struct fill_xx
{
private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  fill_xx(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef int result_type;
  template <typename View>
  result_type operator()(View vw)
  {
    return stapl::map_reduce(fill_x(m_zin, m_procs),
      stapl::plus<int>(), vw);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_procs);
  }
};

// Used to test proper assignment of a vector(array), array(vector),
// array(array) or vector(vector) in the second level of a nested container
// or as a nested container by itself.
struct test_xx
{
  typedef int result_type;
  template<typename View>
  result_type operator()(View vw)
  {
    return stapl::map_reduce(test_x(), stapl::plus<int>(), vw);
  }
};

// Used to populate a graph(array) or graph(vector) in the second level of a
// nested container or as a nested container by itself.
struct fill_gx
{
private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  fill_gx(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef int result_type;
  template <typename Vertex>
  result_type operator()(Vertex v)
  {
    return stapl::map_reduce(fill_x(m_zin, m_procs),
      stapl::plus<int>(), v.property());
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_procs);
  }
};

// Used to test proper assignment of a graph(array) or graph(vector)
// in the second level of a nested container or as a nested container by
// itself.
struct test_gx
{
  typedef int result_type;
  template<typename Vertex>
  result_type operator()(Vertex v)
  {
    return stapl::map_reduce(test_x(), stapl::plus<int>(), v.property());
  }
};

// Used to populate an array(graph) or vector(graph) in the second level of a
// nested container or as a nested container by itself.
struct fill_xg
{
  private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  fill_xg(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef int result_type;
  template <typename view>
  result_type operator()(view vw)
  {
    return stapl::map_reduce(fill_g(m_zin, m_procs),
      stapl::plus<int>(), vw);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);

    t.member(m_procs);
  }
};

// Used to test proper assignment of an array(graph) or vector(graph)
// in the second level of a nested container or as a nested container
// by itself.
struct test_xg
{
  typedef int result_type;
  template<typename View>
  result_type operator()(View vw)
  {
    return stapl::map_reduce(test_g(), stapl::plus<int>(), vw);
  }
};

// Used to populate an graph(map) in the second level of a
// nested container or as a nested container.
struct fill_gm
{
  private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  fill_gm(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef int result_type;
  template <typename Vertex>
  result_type operator()(Vertex v, int size)
  {
    int total = 0;
    int num;
    for (int i = 0; i < size; i++) {
      m_zin >> num;
      v.property()[i] = num*m_procs;
      total += num*m_procs;

    }

    return total;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);

    t.member(m_procs);
  }
};

// Used to test proper assignment of a graph(map)
// in the second level of a nested container or as a nested container.
struct test_gm
{
  typedef int result_type;
  template<typename Vertex>
  result_type operator()(Vertex v) {
    return stapl::map_reduce(test_m(), stapl::plus<int>(), v.property());
  }
};

// Used to populate an graph(graph(graph)) as a nested container.
struct fill_ggg
{
private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  fill_ggg(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef int result_type;
  template <typename Vertex>
  result_type operator()(Vertex v) const
  {
    return stapl::map_reduce(fill_gg(m_zin, m_procs),
      stapl::plus<int>(), v.property());
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_procs);
  }
};

// Used to test proper assignment of a graph(graph(graph))
struct test_ggg
{
   typedef int result_type;
   template <typename Vertex>
   result_type operator()(Vertex v)
   {
     return stapl::map_reduce(test_gg(), stapl::plus<int>(), v.property());
   }
};

// Used to populate an graph(array(vector)), graph(vector(array))
// graph(vector(vector)), or graph(array(array)) as a nested container.
struct fill_gxx
{
private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  fill_gxx(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef int result_type;
  template <typename Vertex>
  result_type operator()(Vertex v) const
  {
    return stapl::map_reduce(fill_xx(m_zin, m_procs),
      stapl::plus<int>(), v.property());
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_procs);
  }
};

// Used to test proper assignment of a graph(array(vector)),
// graph(vector(array)) graph(vector(vector)), or graph(array(array))
// as a nested container.
struct test_gxx
{
   typedef int result_type;
   template <typename Vertex>
   result_type operator()(Vertex v)
   {
     return stapl::map_reduce(test_xx(), stapl::plus<int>(), v.property());
   }
};

// Used to populate an graph(graph(vector)) or  graph(graph(array))
// as a nested container.
struct fill_ggx
{
private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  fill_ggx(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef int result_type;
  template <typename Vertex>
  result_type operator()(Vertex v) const
  {
    return stapl::map_reduce(fill_gx(m_zin, m_procs),
      stapl::plus<int>(), v.property());
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_procs);
  }
};

// Used to test proper assignment of a graph(graph(vector)) or
// graph(graph(array)) as a nested container.
struct test_ggx
{
   typedef int result_type;
   template <typename Vertex>
   result_type operator()(Vertex v)
   {
     return stapl::map_reduce(test_gx(), stapl::plus<int>(), v.property());
   }
};

// Used to populate an array(graph(vector)), vector(graph(array))
// array(graph(array)), or vector(graph(vector)) as a nested container.
struct fill_xgx
{
private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  fill_xgx(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef int result_type;
  template <typename View>
  result_type operator()(View vw) const
  {
    return stapl::map_reduce(fill_gx(m_zin, m_procs),
      stapl::plus<int>(), vw);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_procs);
  }
};

// Used to test proper assignment of an array(graph(vector)),
// vector(graph(array)), array(graph(array)), or vector(graph(vector))
//as a nested container.
struct test_xgx
{
   typedef int result_type;
   template <typename View>
   result_type operator()(View vw)
   {
     return stapl::map_reduce(test_gx(), stapl::plus<int>(), vw);
   }
};

// Used to populate an array(graph(graph)), vector(graph(graph))
// as a nested container.
struct fill_xgg
{
private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  fill_xgg(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef int result_type;
  template <typename View>
  result_type operator()(View vw) const
  {
    return stapl::map_reduce(fill_gg(m_zin, m_procs),
      stapl::plus<int>(), vw);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_procs);
  }
};

// Used to test proper assignment of a array(graph(graph)),
// vector(graph(graph))
// as a nested container.
struct test_xgg
{
   typedef int result_type;
   template <typename View>
   result_type operator()(View vw)
   {
     return stapl::map_reduce(test_gg(), stapl::plus<int>(), vw);
   }
};

// Used to populate an array(vector(graph)), vector(vector(graph)),
// array(array(graph)), vector(array(graph)) as a nested container.
struct fill_xxg
{
private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  fill_xxg(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef int result_type;
  template <typename View>
  result_type operator()(View vw) const
  {
    return stapl::map_reduce(fill_xg(m_zin, m_procs),
      stapl::plus<int>(), vw);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_procs);
  }
};

// Used to test proper assignment of an array(vector(graph)),
// vector(vector(graph)), array(array(graph)), vector(array(graph).
struct test_xxg
{
   typedef int result_type;
   template <typename View>
   result_type operator()(View vw)
   {
     return stapl::map_reduce(test_xg(), stapl::plus<int>(), vw);
   }
};

// Used to populate an array(graph(map)), vector(graph(map)),
// as a nested container.
struct fill_xgm
{
private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  fill_xgm(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef int result_type;
  template<typename View1>
  result_type operator()(View1 vw, int data_size) {
    return stapl::map_reduce(fill_gm(m_zin, m_procs),
      stapl::plus<int>(), vw, stapl::make_repeat_view(data_size));
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_procs);
  }
};

// Used to test proper assignment of array(graph(map)) or vector(graph(map))
struct test_xgm
{
  typedef int result_type;
  template<typename View1>
  result_type operator()(View1 vw) {
    return stapl::map_reduce(test_gm(), stapl::plus<int>(), vw);
  }
};

// Used to populate an (graph(array(graph)) or graph(vector(graph)
// as a nested container.
struct fill_gxg
{
private:
  stapl::stream<ifstream> m_zin;
  int m_procs;

public:
  fill_gxg(stapl::stream<ifstream> const& zin, int procs)
    : m_zin(zin), m_procs(procs)
  { }

  typedef int result_type;
  template <typename Vertex>
  result_type operator()(Vertex v) const
  {
    return stapl::map_reduce(fill_xg(m_zin, m_procs),
      stapl::plus<int>(), v.property());
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
    t.member(m_procs);
  }
};

// Used to test proper assignment of a (graph(array(graph))
// or graph(vector(graph) as a nested container.
struct test_gxg
{
  typedef int result_type;
  template<typename Vertex>
  result_type operator()(Vertex v) {
    return stapl::map_reduce(test_xg(), stapl::plus<int>(), v.property());
  }
};

// Test case: graph(graph)
void gg(int data_size, int num_executions, stapl::stream<ifstream>& zin,
  int procs)
{
  stapl::do_once([&](){
    std::cerr << "graph(graph(int))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++) {
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    d_graph_graph_tp                                           gg(init_arr_vw);
    d_graph_graph_vw_tp                                              gg_vw(gg);

    auto real_result = stapl::map_reduce(fill_gg(zin, procs),
        stapl::bit_xor<int>(), gg_vw);
    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_gg(), stapl::bit_xor<int>(),
      gg_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

// Test case: graph(array)
void ga(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
    std::cerr << "graph(array(int))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++) {
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);
    d_graph_arr_tp                                            dga(init_arr_vw);
    d_graph_arr_vw_tp                                              dga_vw(dga);

    auto real_result = stapl::map_reduce(fill_gx(zin, procs),
      stapl::min<int>(), dga_vw);
    stapl::counter<stapl::default_timer> ctr;

    ctr.start();
    auto test_result = stapl::map_reduce(test_gx(),
      stapl::min<int>(), dga_vw);
    stapl::get<1>(fin_res) += ctr.stop();

    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

void gv(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
    std::cerr << "graph(vector(int))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++) {
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);
    d_graph_vec_tp                                            dgv(init_arr_vw);
    d_graph_vec_vw_tp                                              dgv_vw(dgv);

    auto real_result = stapl::map_reduce(fill_gx(zin, procs),
   stapl::max<int>(), dgv_vw);
    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_gx(),
      stapl::max<int>(), dgv_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

// Test case: array(graph)
void ag(int data_size, int num_executions, stapl::stream<ifstream>& zin,
  int procs)
{
  stapl::do_once([&](){
    std::cerr << "array(graph(int))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++) {
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);
    d_arr_graph_tp                                           dag(init_arr_vw);
    d_arr_graph_vw_tp                                             dag_vw(dag);

    auto real_result = stapl::map_reduce(fill_xg(zin, procs),
      stapl::plus<int>(), dag_vw);
    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_xg(),
      stapl::plus<int>(), dag_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

// Test case: vector(graph)
void vg(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
    std::cerr << "vector(graph(int))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++) {
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);
    d_vec_graph_tp                                            dvg(init_arr_vw);
    d_vec_graph_vw_tp                                             dvg_vw(dvg);

    auto real_result = stapl::map_reduce(fill_xg(zin, procs),
      stapl::bit_xor<int>(), dvg_vw);
    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_xg(),
      stapl::bit_xor<int>(), dvg_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

void gm(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
    std::cerr << "graph(map(int,int))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++) {
    d_graph_m_tp                                                 gm(data_size);
    d_graph_m_vw_tp                                                  gm_vw(gm);
    auto real_result = stapl::map_reduce(fill_gm(zin, procs),
      stapl::min<int>(), gm_vw, stapl::make_repeat_view(data_size));

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_gm(),
      stapl::min<int>(), gm_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

// Test case: graph(graph(graph)
void ggg(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "graph(graph(graph(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_graph_graph_graph_tp                                         dggg(sz_vw);
    d_graph_graph_graph_vw_tp                                    dggg_vw(dggg);

    auto real_result = stapl::map_reduce(fill_ggg(zin, procs),
      stapl::max<int>(), dggg_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_ggg(), stapl::max<int>(),
      dggg_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
    print_results(fin_res, num_executions);
  }
}

void gaa(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "graph(array(array(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_graph_arr_arr_tp                                             dgaa(sz_vw);
    d_graph_arr_arr_vw_tp                                        dgaa_vw(dgaa);

    auto real_result = stapl::map_reduce(fill_gxx(zin, procs),
      stapl::plus<int>(), dgaa_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_gxx(), stapl::plus<int>(),
      dgaa_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: graph(vector(vector(int)))
void gvv(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "graph(vector(vector(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_graph_vec_vec_tp                                             dgvv(sz_vw);
    d_graph_vec_vec_vw_tp                                        dgvv_vw(dgvv);

    auto real_result = stapl::map_reduce(fill_gxx(zin, procs),
      stapl::bit_xor<int>(), dgvv_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_gxx(), stapl::bit_xor<int>(),
      dgvv_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

// Test case graph(vector(array))
void gva(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "graph(vector(array(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++) {
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_graph_vec_arr_tp                                             dgva(sz_vw);
    d_graph_vec_arr_vw_tp                                        dgva_vw(dgva);

    auto real_result = stapl::map_reduce(fill_gxx(zin, procs),
      stapl::min<int>(), dgva_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_gxx(), stapl::min<int>(),
      dgva_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: graph(array(vector(int)))
void gav(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "graph(array(vector(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_graph_arr_vec_tp                                             dgav(sz_vw);
    d_graph_arr_vec_vw_tp                                        dgav_vw(dgav);

    auto real_result = stapl::map_reduce(fill_gxx(zin, procs),
      stapl::max<int>(), dgav_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_gxx(), stapl::max<int>(),
      dgav_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: graph(graph(array(int)))
void gga(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "graph(graph(array(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_graph_graph_arr_tp                                           dgga(sz_vw);
    d_graph_graph_arr_vw_tp                                      dgga_vw(dgga);

    auto real_result = stapl::map_reduce(fill_ggx(zin, procs),
      stapl::plus<int>(), dgga_vw);
    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_ggx(), stapl::plus<int>(),
      dgga_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: graph(graph(vector(int)))
void ggv(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "graph(graph(vector(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_graph_graph_vec_tp                                           dggv(sz_vw);
    d_graph_graph_vec_vw_tp                                      dggv_vw(dggv);

    auto real_result = stapl::map_reduce(fill_ggx(zin, procs),
      stapl::bit_xor<int>(), dggv_vw);
    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_ggx(), stapl::bit_xor<int>(),
      dggv_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: graph(array(graph(int)))
void gag(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "graph(array(graph(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_graph_arr_graph_tp                                           dgag(sz_vw);
    d_graph_arr_graph_vw_tp                                      dgag_vw(dgag);

    auto real_result = stapl::map_reduce(fill_gxg(zin, procs),
      stapl::min<int>(), dgag_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_gxg(), stapl::min<int>(),
      dgag_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: graph(vector(graph(int)))
void gvg(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "graph(vector(graph(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_graph_vec_graph_tp                                           dgvg(sz_vw);
    d_graph_vec_graph_vw_tp                                      dgvg_vw(dgvg);

    auto real_result = stapl::map_reduce(fill_gxg(zin, procs),
      stapl::max<int>(), dgvg_vw);
    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_gxg(), stapl::max<int>(),
      dgvg_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: array(graph(graph(int)))
void agg(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "array(graph(graph(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_arr_graph_graph_tp                                           dagg(sz_vw);
    d_arr_graph_graph_vw_tp                                      dagg_vw(dagg);

    auto real_result = stapl::map_reduce(fill_xgg(zin, procs),
      stapl::plus<int>(), dagg_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_xgg(), stapl::plus<int>(),
      dagg_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: vector(graph(graph(int)))
void vgg(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "vector(graph(graph(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_vec_graph_graph_tp                                           dvgg(sz_vw);
    d_vec_graph_graph_vw_tp                                      dvgg_vw(dvgg);

    auto real_result = stapl::map_reduce(fill_xgg(zin, procs),
      stapl::bit_xor<int>(), dvgg_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_xgg(), stapl::bit_xor<int>(),
      dvgg_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: vector(graph(vector(int)))
void vgv(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "vector(graph(vector(int)))...\t\t";
  });


  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_vec_graph_vec_tp                                             dvgv(sz_vw);
    d_vec_graph_vec_vw_tp                                        dvgv_vw(dvgv);

    auto real_result = stapl::map_reduce(fill_xgx(zin, procs),
      stapl::min<int>(), dvgv_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_xgx(), stapl::min<int>(),
      dvgv_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: array(graph(array(int)))
void aga(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "array(graph(array(int)))...\t\t";
  });


  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_arr_graph_arr_tp                                             daga(sz_vw);
    d_arr_graph_arr_vw_tp                                        daga_vw(daga);

    auto real_result = stapl::map_reduce(fill_xgx(zin, procs),
      stapl::max<int>(), daga_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_xgx(), stapl::max<int>(),
      daga_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: array(graph(vector(int)))
void agv(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "array(graph(vector(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_arr_graph_vec_tp                                             dagv(sz_vw);
    d_arr_graph_vec_vw_tp                                        dagv_vw(dagv);

    auto real_result = stapl::map_reduce(fill_xgx(zin, procs),
      stapl::plus<int>(), dagv_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_xgx(), stapl::plus<int>(),
      dagv_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: vector(graph(array(int)))
void vga(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "vector(array(graph(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_vec_graph_arr_tp                                             dvga(sz_vw);
    d_vec_graph_arr_vw_tp                                        dvga_vw(dvga);

    auto real_result = stapl::map_reduce(fill_xgx(zin, procs),
      stapl::bit_xor<int>(), dvga_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_xgx(), stapl::bit_xor<int>(),
      dvga_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: array(array(graph(int)))
void aag(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "array(array(graph(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_arr_arr_graph_tp                                             daag(sz_vw);
    d_arr_arr_graph_vw_tp                                        daag_vw(daag);

    auto real_result = stapl::map_reduce(fill_xxg(zin, procs),
      stapl::min<int>(), daag_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_xxg(), stapl::min<int>(),
      daag_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: vector(vector(graph(int)))
void vvg(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "vector(vector(graph(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_vec_vec_graph_tp                                             dvvg(sz_vw);
    d_vec_vec_graph_vw_tp                                        dvvg_vw(dvvg);

    auto real_result = stapl::map_reduce(fill_xxg(zin, procs),
      stapl::max<int>(), dvvg_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_xxg(), stapl::max<int>(),
      dvvg_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: arr(vector(graph(int)))
void avg(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "arr(vector(graph(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_arr_vec_graph_tp                                             davg(sz_vw);
    d_arr_vec_graph_vw_tp                                        davg_vw(davg);

    auto real_result = stapl::map_reduce(fill_xxg(zin, procs),
      stapl::plus<int>(), davg_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_xxg(), stapl::plus<int>(),
      davg_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: vector(arr(graph(int)))
void vag(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "vector(array(graph(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    arr_arr_tp                                                 sz(init_arr_vw);
    arr_arr_vw_tp                                                    sz_vw(sz);
    stapl::map_func(gen_sz_out(zin, procs), sz_vw);

    d_vec_arr_graph_tp                                             dvag(sz_vw);
    d_vec_arr_graph_vw_tp                                        dvag_vw(dvag);

    auto real_result = stapl::map_reduce(fill_xxg(zin, procs),
      stapl::bit_xor<int>(), dvag_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_xxg(), stapl::bit_xor<int>(),
      dvag_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}


//Test case: vector(graph(map(int,int))).
void vgm(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "vector(graph(map(int,int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    d_vec_graph_m_tp                                          vgm(init_arr_vw);
    d_vec_graph_m_vw_tp                                            vgm_vw(vgm);

    auto real_result = stapl::map_reduce(fill_xgm(zin, procs),
      stapl::min<int>(), vgm_vw, stapl::make_repeat_view(data_size));

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_xgm(), stapl::min<int>(),
      vgm_vw);

    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: array(graph(map(int,int)))
void agm(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "array(graph(map(int,int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++){
    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    d_arr_graph_m_tp                                          agm(init_arr_vw);
    d_arr_graph_m_vw_tp                                            agm_vw(agm);

    auto real_result = stapl::map_reduce(fill_xgm(zin, procs),
      stapl::max<int>(), agm_vw, stapl::make_repeat_view(data_size));

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_xgm(), stapl::max<int>(),
      agm_vw);

    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

#if OUTER_MAP

typedef stapl::map<int, d_graph_tp>                               d_m_graph_tp;
typedef stapl::map_view<d_m_graph_tp>                          d_m_graph_vw_tp;
typedef stapl::map<int, d_arr_graph_tp>                      dd_m_arr_graph_tp;
typedef stapl::map_view<dd_m_arr_graph_tp>                dd_m_arr_graph_vw_tp;
typedef stapl::map<int, d_vec_graph_tp>                      dd_m_vec_graph_tp;
typedef stapl::map_view<dd_m_vec_graph_tp>                dd_m_vec_graph_vw_tp;
typedef stapl::map<int, d_graph_arr_tp>                      dd_m_graph_arr_tp;
typedef stapl::map_view<dd_m_graph_arr_tp>                dd_m_graph_arr_vw_tp;
typedef stapl::map<int, d_graph_vec_tp>                      dd_m_graph_vec_tp;
typedef stapl::map_view<dd_m_graph_vec_tp>                dd_m_graph_vec_vw_tp;

// Used to populate a map(int, graph) as a nested container.
struct fill_mg
{
  private:
  stapl::stream<ifstream> m_zin;

public:
  fill_mg(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef int result_type;
  template <typename MapView>
  result_type operator()(MapView mvw)
  {
    return stapl::map_reduce(fill_g(m_zin), stapl::plus<int>(),
      mvw.second);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);

  }
};

// Used to test proper assignment of a map(int, graph)
struct test_mg
{
  typedef int result_type;
  template<typename MapView1>
  result_type operator()(MapView1 mvw)
  {
    return stapl::map_reduce(test_g(), stapl::plus<int>(), mvw.second);
  }
};

// Used to populate a map(int, array(graph)) or map(int, vector(graph))
// as a nested container.
struct fill_mxg
{
private:
  stapl::stream<ifstream> m_zin;

public:
  fill_mxg(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef int result_type;
  template<typename Pair>
  result_type operator()(Pair p) {
    return stapl::map_reduce(fill_xg(m_zin), stapl::plus<int>(),
      p.second);
  }

  void define_type(stapl::typer& t) {
    t.member(m_zin);
  }
};

// Used to test proper assignment of a map(int, array(graph)) or
// map(int, vector(graph))
struct test_mxg
{
  typedef int result_type;
  template<typename Pair>
  result_type operator()(Pair p) {
    return stapl::map_reduce(test_xg(), stapl::plus<int>(), p.second);
  }
};

// Used to populate a map(int, graph(array)) or map(int, graph(vector))
// as a nested container.
struct fill_mgx
{
private:
  stapl::stream<ifstream> m_zin;

public:
  fill_mgx(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef int result_type;
  template<typename Pair>
  result_type operator()(Pair p) {
    return stapl::map_reduce(fill_gx(m_zin), stapl::plus<int>(),
      p.second);
  }

  void define_type(stapl::typer& t) {
    t.member(m_zin);
  }
};

// Used to test proper assignment of a map(int, graph(array)) or
// map(int, graph(vector))
struct test_mgx
{
  typedef int result_type;
  template<typename Pair>
  result_type operator()(Pair p) {
    return stapl::map_reduce(test_gx(), stapl::plus<int>(), p.second);
  }
};

//Test case: map(graph(int))
void mg(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
    std::cerr << "map(graph(int))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++) {
    ndx_dom_tp                                           map_dom(0, data_size);
    d_m_graph_tp                                                  mg(map_dom);
    d_m_graph_vw_tp                                                 mg_vw(mg);
    d_graph_tp                                                mg_g(data_size);

    for (int j = 0; j < data_size; j++)
      mg.insert(make_pair(j, mg_g));

    auto real_result = stapl::map_reduce(fill_mg(zin, procs),
        stapl::plus<int>(), mg_vw);
    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_mg(),stapl::plus<int>(),
      mg_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = stapl::get<0>(real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
    print_results(fin_res, num_executions);
}

// Test case: map(int, array(graph(int)))
void mag(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "map(int, array(graph(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++) {
    ndx_dom_tp                                           map_dom(0, data_size);
    dd_m_arr_graph_tp                                             mag(map_dom);
    dd_m_arr_graph_vw_tp                                           mag_vw(mag);

    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    dd_arr_graph_tp                                        mag_ag(init_arr_vw);
    dd_arr_graph_vw_tp                                       mag_ag_vw(mag_ag);

    for (int j = 0; j < data_size; j++)
      mag.insert(make_pair(j, mag_ag));

    auto real_result = stapl::map_reduce(fill_mxg(zin, procs),
      stapl::bit_xor<int>(), mag_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_mxg(), stapl::bit_xor<int>(),
      mag_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: map(int, vector(graph(int)))
void mvg(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "map(int, vector(graph(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++) {
    ndx_dom_tp                                           map_dom(0, data_size);
    dd_m_vec_graph_tp                                             mvg(map_dom);
    dd_m_vec_graph_vw_tp                                           mvg_vw(mvg);

    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    dd_vec_graph_tp                                        mvg_vg(init_arr_vw);
    dd_vec_graph_vw_tp                                       mvg_vg_vw(mvg_vg);

  for (int j = 0; j < data_size; j++)
      mvg.insert(make_pair(j, mvg_vg));

    auto real_result = stapl::map_reduce(fill_mxg(zin, procs),
      stapl::min<int>(), mvg_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_mxg(), stapl::min<int>(),
      mvg_vw);

    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                     && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: map(int, graph(array(int))).
void mga(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "map(int, graph(array(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++) {
    ndx_dom_tp                                           map_dom(0, data_size);
    dd_m_graph_arr_tp                                             mga(map_dom);
    dd_m_graph_arr_vw_tp                                           mga_vw(mga);

    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    dd_graph_arr_tp                                        mga_ga(init_arr_vw);
    dd_graph_arr_vw_tp                                       mga_ga_vw(mga_ga);

    for (int j = 0; j < data_size; j++)
      mga.insert(make_pair(j, mga_ga));

    auto real_result = stapl::map_reduce(fill_mgx(zin, procs),
      stapl::max<int>(), mga_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_mgx(), stapl::max<int>(),
      mga_vw);

    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                    && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}

//Test case: map(int, graph(vector(int)))
void mgv(int data_size, int num_executions, stapl::stream<ifstream>& zin,
                    int procs)
{
  stapl::do_once([&](){
  std::cerr << "map(int, graph(vector(int)))...\t\t";
  });

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_executions; i++) {
    ndx_dom_tp                                           map_dom(0, data_size);
    dd_m_graph_vec_tp                                             mgv(map_dom);
    dd_m_graph_vec_vw_tp                                           mgv_vw(mgv);

    arr_tp                                                 init_arr(data_size);
    arr_vw_tp                                            init_arr_vw(init_arr);
    stapl::serial_io(gen_sz_in(zin, procs), init_arr_vw);

    dd_graph_vec_tp                                           mgv(init_arr_vw);
    dd_graph_vec_vw_tp                                       mgv_gv_vw(mgv_gv);

    for (int j = 0; j < data_size; j++)
      mgv.insert(make_pair(j, mgv_gv));

    auto real_result = stapl::map_reduce(fill_mgx(zin, procs),
      stapl::plus<int>(), mgv_vw);

    stapl::counter<stapl::default_timer> ctr;
    ctr.start();
    auto test_result = stapl::map_reduce(test_mgx(), stapl::plus<int>(),
      mgv_vw);

    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (real_result == test_result)
                                                    && stapl::get<0>(fin_res);
  }
  print_results(fin_res, num_executions);
}
#endif


///////////////////////////////////////////////////////////////////////////////
// Prints help options for the user. Function is invoked if user provides
// -h or --help as command line arguments.
///////////////////////////////////////////////////////////////////////////////

void show_help()
{
  stapl::do_once([&]() {
    std::cerr << "Allowed options: \n";
    std::cerr << "\t-h [ --help ] \t\t Print this help message \n";
    std::cerr << "\t-d [ --data ] arg \t Data set \n";
    std::cerr << "\t-l [ --list ] \t\t Print tests provided \n";
    std::cerr << "\t-t [ --test ] arg \t Test to run \n";
    std::cerr << "\t-i [ --iterations ]\t arg Number of times test is repeated";
    std::cerr << " in timed section\n";
    std::cerr << "\n";
    std::cerr << "\nNOTE: With no options the program will run all tests in";
    std::cerr << "a medium data set 32 times.\n";
  });
}


///////////////////////////////////////////////////////////////////////////////
// This struct can be used to specify the tests that we provide, and also the
// tests that we want to run. It does so by keeping a map of function pointers
// and function abbreviations. Each function abbreviation represents a name for
// a function that executes a test case provided here. If the user
// does not select any tests, the program runs all tests, in a medium data
// size, otherwise only runs the tests specified by the user. This is invoked
// by main
// m_data is the length of the sizes array.
// m_num_executions how many iterations of the test to run
// m_filename what file should I use for input values.
// m_test_map a map of functions (which are test cases) and their
// abbreviations.
// m_zin stream object to read input from fill
///////////////////////////////////////////////////////////////////////////////
struct test_executor
{
   int m_data;
   int m_num_executions;
   std::string m_filename;
   std::map<std::string, test_ptr> m_test_map;
   stapl::stream<std::ifstream> m_zin;

   int m_procs;

protected:
  void all()
  {
    for (typename std::map<std::string, test_ptr>::iterator
        it = m_test_map.begin(); it != m_test_map.end(); ++it) {
        ((test_ptr) (it->second))(m_data, m_num_executions, m_zin,
          m_procs);
    }
  }

public:
  test_executor(int data_size, int num_exec, std::string filename, int procs)
    : m_data(data_size), m_num_executions(num_exec), m_filename(filename),
      m_procs(procs)
  {
    m_test_map["gg"] = &gg;
    m_test_map["ag"] = &ag;
    m_test_map["ga"] = &ga;
    m_test_map["gv"] = &gv;
    m_test_map["vg"] = &vg;
    m_test_map["gm"] = &gm;

    m_test_map["ggg"] = &ggg;
    m_test_map["gaa"] = &gaa;
    m_test_map["gvv"] = &gvv;
    m_test_map["gva"] = &gva;
    m_test_map["gav"] = &gav;
    m_test_map["gga"] = &gga;
    m_test_map["ggv"] = &ggv;
    m_test_map["gag"] = &gag;
    m_test_map["gvg"] = &gvg;
    m_test_map["agg"] = &agg;
    m_test_map["vgg"] = &vgg;
    m_test_map["vgv"] = &vgv;
    m_test_map["aga"] = &aga;
    m_test_map["agv"] = &agv;
    m_test_map["vga"] = &vga;
    m_test_map["aag"] = &aag;
    m_test_map["vvg"] = &vvg;
    m_test_map["avg"] = &avg;
    m_test_map["vag"] = &vag;
    m_test_map["vgm"] = &vgm;
    m_test_map["agm"] = &agm;


#ifdef OUTER_MAP
    m_test_map["mg"] = &mg;
    m_test_map["mag"] = &mag;
    m_test_map["mav"] = &mav;
    m_test_map["mga"] = &mga;
    m_test_map["mgv"] = &mgv;
#endif

    m_zin.open(m_filename.c_str());
  }

  typedef void result_type;
  template <typename View>
  result_type operator()(const View& m_selected_tests)
  {
    if (m_selected_tests.size() != 0)
    {
      for (typename stapl::vector<std::string>::iterator
        it = m_selected_tests.begin(); it != m_selected_tests.end(); ++it)
      {
        if (m_test_map[(*it)] != NULL)
        {
          ((test_ptr) m_test_map[(*it)])
                                      (m_data, m_num_executions, m_zin,
                                        m_procs);
        }
        else
          std::cerr << "Invalid test name: " << (*it) << ".\n";
      }
    }
    else
      all();

    m_zin.close();
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_data);
    t.member(m_num_executions);
    t.member(m_filename);
    t.member(m_test_map);
    t.member(m_zin);

    t.member(m_procs);
  }
};

///////////////////////////////////////////////////////////////////////////////
// Main pre-selects values for data_size, number of iterations of a test, and
// the file where to read input from; in case the user does not specify any of
// those command line arguments.
// Then reads input to see whether the user has modified default arguments.
// If the user has specified something for any of the program options described
// in the show_help function than those pre-selected values are updated.
// It also does input validation, and finally invokes the test_executor struct
// which does the rest of the work in the program.
///////////////////////////////////////////////////////////////////////////////
stapl::exit_code stapl_main(int argc, char** argv)
{
  int data_size = 10000;
  int num_iterations = 32;
  stapl::vector<std::string> selected_tests;
  std::string input_filename = "data/medium_factors.zin";
  std::set<std::string> test_set;

  test_set.insert("gg");
  test_set.insert("ag");
  test_set.insert("gv");
  test_set.insert("ga");
  test_set.insert("vg");
  test_set.insert("gm");
  test_set.insert("gav");
  test_set.insert("gga");
  test_set.insert("ggv");
  test_set.insert("gag");
  test_set.insert("ggg");
  test_set.insert("gaa");
  test_set.insert("gvv");
  test_set.insert("gvg");
  test_set.insert("gva");
  test_set.insert("agg");
  test_set.insert("vgg");
  test_set.insert("vgv");
  test_set.insert("aga");
  test_set.insert("agv");
  test_set.insert("vga");
  test_set.insert("aag");
  test_set.insert("vvg");
  test_set.insert("avg");
  test_set.insert("vag");
  test_set.insert("vgm");
  test_set.insert("agm");

#ifdef OUTER_MAP
  test_set.insert("mg");
  test_set.insert("mag");
  test_set.insert("mav");
  test_set.insert("mga");
  test_set.insert("mgv");
#endif

  if (argc > 1)
  {
    for (int i = 0; i < argc; i++)
    {
      if (strcmp("-h", argv[i]) == 0 || strcmp("--help", argv[i]) == 0)
      {
        show_help();
        exit(1); // exit
      }

      if (strcmp("-d", argv[i]) == 0 || strcmp("--data", argv[i]) == 0)
      {
        if (i+1 <= argc-1)
        {
          i++;

          char * opt = argv[i];
          switch ( opt[0] )
          {
            case 't':
              data_size = 1;
              input_filename = "data/tiny_factors.zin";
              break;
            case 's':
              data_size = 100;
              input_filename = "data/small_factors.zin";
              break;
            case 'm':
              data_size = 10000;
              input_filename = "data/medium_factors.zin";
              break;
          #ifdef BIG_HUGE_DATA
            case 'b':
              data_size = 10000000;
              input_filename = "data/big_factors.zin";
              break;
            case 'h':
              data_size = 100000000;
              input_filename = "data/huge_factors.zin";
              break;
          #endif
            default:
              std::cerr << "usage: exe --data tiny/small/medium/big/huge\n";
              exit(1); // exit
          }
        }
        else
        {
          break;
        }
      }

      if (strcmp("-i", argv[i]) == 0 || strcmp("--iterations", argv[i]) == 0)
      {
        if (i+1 <= argc-1)
        {
          i++;
          num_iterations = atoi(argv[i]);
          if (num_iterations < 1)
          {
            std::cerr << "Number of iterations cannot be less than 1!\n";
            exit(1);
          }
        }
      }

      if (strcmp("-l", argv[i]) == 0 || strcmp("--list", argv[i]) == 0)
      {
        std::cerr << "Tests Provided ~ Abbreviations \n";
        int i = 1;
        for (auto it = test_set.begin(); it != test_set.end(); ++it)
        {
          std::cerr << i << ". " << (*it) << "\n";
          ++i;
        }
        exit(1); // must exit program
      }

      if (strcmp("-t", argv[i]) == 0 || strcmp("--test", argv[i]) == 0)
      {
        if (i+1 <= argc-1)
        {
          i++;
          if (test_set.find(argv[i]) != test_set.end())
          {
            selected_tests.push_back(argv[i]);
          }
          else {
            std::cerr << "\nInvalid test name provided!\n";
            exit(1);
          }
        }
        else
          break;
      }
    }
  }

  int procs = stapl::get_num_locations();
  test_executor run(data_size*procs, num_iterations, input_filename, procs);
  stapl::vector_view<stapl::vector<std::string> >
                                                 sel_test_view(selected_tests);
  run(sel_test_view);
  return EXIT_SUCCESS;
}
