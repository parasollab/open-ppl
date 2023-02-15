#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/paradigms/graph_paradigm.hpp>
#include <stapl/containers/graph/algorithms/boruvka.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/complete.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/utility/do_once.hpp>

#include "test_util.h"

#include <cstdlib>
#include <limits>
#include <vector>

using namespace std;
using namespace stapl;


template<typename G>
void print_graph(G& g)
{
  do_once([&]() {
    std::cout << "-----------------" << std::endl;
    for (auto v : g) {
      std::cout << v.descriptor() << "  :  ";
      for (auto e : v) {
        std::cout << e.target() << " ("
                  << e.property().property.weight() << "),  ";
      }
      std::cout << std::endl;
    }
    std::cout << "-----------------" << std::endl;
    });
}


struct set_weights
  : generators::rand_gen
{
  typedef void result_type;

  template<typename Elt>
  void operator()(Elt e)
  { e = this->rand()%10; }
};


/////////////////////////////////////////////////////////////////////
/// @brief Sets the weight of each edge in the graph to a random value.
///
/// @note Each edge has its source and target values initialized to the
/// current source and target vertices (the initial values) and assigns
/// weights to the edges based on the input weights array.
////////////////////////////////////////////////////////////////////
struct set_edge_weights
{
  typedef void result_type;

  template<typename Vertex, typename Weights>
  void operator()(Vertex v, Weights w) const
  {
    for (auto it = v.begin(); it != v.end(); ++it) {
      (*it).property().property.source((*it).source());
      (*it).property().property.target((*it).target());
      if ((*it).source() < (*it).target())
        (*it).property().property.weight(w[(*it).source()]);
      else
        (*it).property().property.weight(w[(*it).target()]);
    }
  }
};


struct incr_preds
{
  size_t m_source;

  incr_preds(size_t const& source = 0)
    : m_source(source)
  { }

  template<typename Vertex>
  bool operator()(Vertex target) const
  {
    if (target.descriptor() == m_source) {
      target.property().property = 1;
      return true;
    } else {
      return false;
    }
  }

  void define_type(typer& t)
  { t.member(m_source); }
};


struct set_preds
{
  size_t m_source;

  set_preds(size_t const& source = 0)
    : m_source(source)
  { }

  struct gt_target
  {
    size_t m_source;

    gt_target()
      : m_source()
    { }

    gt_target(size_t const& source)
      : m_source(source)
    { }

    template<typename E>
    bool operator()(E e) const
    { return e.target() > m_source; }
  };

  template<typename Vertex, typename GraphVisitor>
  bool operator()(Vertex v, GraphVisitor graph_visitor) const
  {
    if (graph_visitor.level() == 0) {
      v.property().property = 0;
      return true;
    } else if (v.descriptor() == m_source &&
               v.property().property != size_t(0)) {
      return false;
    } else {
      graph_visitor.visit_all_edges_if(v, incr_preds(m_source), gt_target());
      return false;
    }
  }

  void define_type(typer& t)
  { t.member(m_source); }
};


struct num_preds
{
  typedef size_t result_type;

  template<typename Vertex>
  result_type operator()(Vertex v) const
  { return v.property().property; }
};


template<typename G>
bool check_cycle(G& g)
{
  graph_paradigm(set_preds(), incr_preds(), g);
  return map_reduce(num_preds(), stapl::plus<size_t>(), g) == 0;
}


///////////////////////////////////////////////////////////////////////
/// @brief This is the main function. Here the graph and tree are initialized
/// and boruvka's algorithm is called.
//////////////////////////////////////////////////////////////////////
stapl::exit_code stapl_main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "usage: exe n" << std::endl;
    return EXIT_FAILURE;
  }

  int graph_size = atoi(argv[1]);

  typedef graph<UNDIRECTED, NONMULTIEDGES, int>::vertex_descriptor vertex_desc_t;
  typedef properties::boruvka_edge_property<vertex_desc_t, int> edge_prop_type;
  typedef graph<UNDIRECTED, NONMULTIEDGES,
                super_vertex_property<vertex_desc_t>,
                super_edge_property<edge_prop_type> > graph_type;
  typedef graph_view<graph_type> view_type;

  one_print("Testing Minimum-Spanning Tree...\t");

  view_type input_vw = generators::make_mesh<view_type>(graph_size, graph_size);

  stapl::array<int> weights(input_vw.size());
  stapl::array_view<stapl::array<int> > weights_vw(weights);
  map_func(set_weights(), weights_vw);

  map_func(set_edge_weights(), input_vw, make_repeat_view(weights_vw));

#if 0
  print_graph(input_vw);
#endif

  view_type mst = boruvka(input_vw);

#if 0
  print_graph(mst);
#endif

  bool passed = check_cycle(mst);

  one_print(passed);

  return EXIT_SUCCESS;
}
