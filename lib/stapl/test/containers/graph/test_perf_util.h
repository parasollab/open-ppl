#ifndef STAPL_TEST_CONTAINERS_GRAPH_TEST_PERF_UTIL_H
#define STAPL_TEST_CONTAINERS_GRAPH_TEST_PERF_UTIL_H

#include <stapl/containers/graph/generators/random_dag.hpp>
#include <stapl/containers/graph/generators/cycles.hpp>
#include <stapl/containers/graph/generators/torus.hpp>

using namespace stapl::generators;

struct no_generator{

  template<typename GraphView>
  GraphView generate() const
  {
    stapl::abort("No generator for this algorithm");
    return GraphView();
  }

};

struct dag_generator{
private:
  size_t m_size;
public:
  dag_generator(size_t size)
    : m_size(size)
  { }

  template<typename GraphView>
  GraphView generate() const
  {
    return make_random_dag<GraphView>(m_size);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_size);
  }
};



struct cycle_chain_generator{
private:
  size_t m_cyclesize;
  size_t m_cyclecount;
public:
  cycle_chain_generator(size_t size = 3, size_t ccs = 256)
    : m_cyclesize(size), m_cyclecount(ccs)
  { }

  template<typename GraphView>
  GraphView generate() const
  {
    return make_cycle_chain<GraphView>(m_cyclecount, m_cyclesize, false);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_cyclesize);
    t.member(m_cyclecount);
  }
};

struct torus_generator{
private:
  size_t m_num_x;
  size_t m_num_y;
public:
  torus_generator(size_t num_x, size_t num_y)
    : m_num_x(num_x), m_num_y(num_y)
  { }

  template<typename GraphView>
  GraphView generate() const
  {
    return make_torus<GraphView>(m_num_x, m_num_y, false);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_num_x);
    t.member(m_num_y);
  }
};


#endif
