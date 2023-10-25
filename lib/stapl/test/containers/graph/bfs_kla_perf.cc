/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/algorithms/breadth_first_search.hpp>

#include "test_util.h"

using namespace stapl;


class bfs_property
{
  typedef size_t VD;
  typedef size_t Level;

  VD    m_vd;
  Level m_level;

public:
  typedef VD    parent_type;
  typedef Level level_type;

  inline VD parent() const
  { return m_vd; }

  inline Level level() const
  { return m_level; }

  inline void parent(VD const& vd)
  { m_vd = vd; }

  inline void level(Level const& c)
  { m_level = c; }

  void define_type(stapl::typer& t)
  {
    t.member(m_vd);
    t.member(m_level);
  }
};


namespace stapl {

template <class Accessor>
class proxy<bfs_property, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef bfs_property target_t;

public:
  typedef size_t vd_type;
  typedef size_t VD;
  typedef size_t Level;
  typedef size_t value_type;

  inline explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  inline operator target_t() const
  { return Accessor::read(); }

  inline proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  inline proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  inline VD parent() const
  { return Accessor::const_invoke(&target_t::parent); }

  inline Level level() const
  {
    typedef Level (target_t::* mem_fun_t)(void) const;
    constexpr mem_fun_t mem_fun = &target_t::level;

    return Accessor::const_invoke(mem_fun);
  }

  inline void  parent(const VD& vd)
  { Accessor::invoke(&target_t::parent, vd); }

  inline void  level(Level const& c)
  { Accessor::invoke(&target_t::level, c); }
}; //struct proxy

}


class init_wf
{
  size_t    m_source;

public:
  typedef void result_type;

  init_wf(size_t source)
    : m_source(source)
  { }

  template <class Vertex>
  void operator()(Vertex v)
  {
    if (v.descriptor() != m_source) {
      v.property().level(0);
    } else {
      v.property().level(1);
    }
    v.property().parent(v.descriptor());
  }

  void define_type(stapl::typer& t)
  { t.member(m_source); }
};


template<typename Graph>
void bfs_test(Graph graph, int k, size_t niter, size_t src)
{
  counter<default_timer> t;
  double time;
  std::vector<double> times;

  size_t num_iterations;
  std::stringstream ss;
  ss << "k= " << k << " > KLA BFS: ";

  auto exec_policy = stapl::sgl::make_execution_policy("kla", graph, k);

  for (size_t i=0; i<niter; ++i) {
    std::stringstream ss2;
    stapl::map_func(init_wf(src), graph);

    t.reset(); t.start();
      num_iterations = stapl::breadth_first_search(exec_policy, graph, src);
    time = t.stop();
    ss2 << "  iterations = " << num_iterations << ", time = " << time;
    times.push_back(time);

    if (get_location_id() == 0)
      std::cout << ss2.str() << std::endl;

    rmi_fence();
  }

  compute_stats(ss.str(), times);
}


template<stapl::graph_attributes Directedness>
void test_graph(size_t niter, std::string filename)
{
  typedef graph<Directedness, MULTIEDGES, bfs_property, size_t> graph_type;
  typedef graph_view<graph_type> graph_view_t;

  if (stapl::get_location_id()==0)
    printf("Reading graph from file: %s\n", filename.c_str());

  counter<default_timer> t;
  t.start();
  graph_view_t v1 = read_adj_list<graph_type>(filename);
  double read_time = t.stop();
  graph_type* g = v1.get_container();
  graph_view_t v(*g);

  if (stapl::get_location_id()==0) {
    std::cout << "Num vertices: " << v.num_vertices()
              << "\nNum edges: " << v.num_edges()
              << "\nRead Time: " << read_time << std::endl;
  }

  v.sort_edges();

  srand(0);

  size_t src = 0;
  while (v[src].size() < 2)
    ++src;

  if (stapl::get_location_id()==0)
    std::cout << "Source: " << src << std::endl;
  rmi_fence();

  size_t k = 0;
  bfs_test(v, k, niter, src);

  k = 1;
  bfs_test(v, k, niter, src);

  for (size_t i=0; i<2; ++i) {
    k *= 2;
    bfs_test(v, k, niter, src);
  }
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe filename  [--undirected] [--miniterations nexp]"
              << std::endl;
    return EXIT_SUCCESS;
  }
  std::string filename  = argv[1];
  bool directed = true;
  size_t niter  = 10;

  for (int i = 1; i < argc; i++) {
    if (!strcmp("--miniterations", argv[i]))
      niter = atoi(argv[i+1]);
    if (!strcmp("--undirected", argv[i]))
      directed = false;
  }

  if (directed)
    test_graph<stapl::DIRECTED>(niter, filename);
  else
    test_graph<stapl::UNDIRECTED>(niter, filename);

  return EXIT_SUCCESS;
}
