/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "test_util.h"
#include <stapl/containers/sequential/graph/bgl_undirected_graph_adapter.hpp>
#include <stapl/containers/sequential/graph/algorithms/graph_algo.h>
#include <stapl/containers/sequential/graph/algorithms/find_cycle.h>
#include <stapl/containers/sequential/graph/algorithms/connected_components.h>
#include "test_mst.cc"

using namespace boost;
using namespace stapl;
using namespace std;

size_t NE;

class weight_dijkstra{
public:
  size_t source;
  size_t target;
  size_t id;
  double d;
  weight_dijkstra(){source=9999;target=9999;id=999;d=1;}
  weight_dijkstra(size_t s, size_t t):source(s),target(t){id=999;d=1;}

  bool operator==(const weight& _other) const {
    return source==_other.source &&
      target==_other.target &&
      id==_other.id;
  }

  bool operator<(const weight_dijkstra& _other) const {
    return this->d < _other.d;
  }
  bool operator>(const weight_dijkstra& _other) const {
    return this->d > _other.d;
  }

  double& Weight(){return d;}
  const double& Weight() const {return d;}
  double MaxWeight() const {return 99999;}
};


class my_vertex_property{
  typedef graph_color<size_t>::value_type color_value_type;
  color_value_type m_color;
  double m_flux;
public:
  my_vertex_property(size_t i=999): m_color(), m_flux(i){}
  void set_color(color_value_type _c){ m_color = _c; }
  color_value_type get_color() const { return m_color; }
  double& flux(){ return m_flux;}
};


template<class Graph>
struct adapt_edge_to_distance
{
  typedef typename Graph::adj_edge_iterator argument_type;
  typedef double&                           result_type;

  double& operator()(typename Graph::adj_edge_iterator it)
  {
    return (*it).property().Weight();
  }
};

template <class G>
void build_graph(G& g){
  // size_t NE = 2*N;

  typename G::vertex_iterator vi;
  typename G::const_vertex_iterator cvi;

  typedef typename G::vertex_descriptor VD;
  std::vector<VD> vertices(N);
  vector<pair<VD,VD> >     edges(NE);
  vector<pair<VD,size_t> > edgeids(NE);

  // cout << "add_vertices..." << endl; cout.flush();
  for (unsigned int i=0; i<N-1; ++i)
    g.add_vertex(i);

  stapl::vector_property_map<graph_color<size_t> > cmap;
  std::vector< std::pair<size_t,typename G::vertex_descriptor> > ccstt;
  size_t num_cc = 0;
  double w;
  // cout << "add_edges..." << endl; cout.flush();
  for (unsigned int i=0; i<NE; ++i) {
    // add a random edge with a random weight.
    w = rand() % N;
    int v1 = rand() % N;
    int v2 = rand() % N;
    g.add_edge(v1, v2, w);
  }
  // cout << "check_cycle..." << endl; cout.flush();
  while (!is_cycle(g, cmap)) {
    w = rand() % N;
    int v1 = rand() % N;
    int v2 = rand() % N;
    g.add_edge(v1, v2, w);
  }
  // cout << "get_cc_stats..." << endl; cout.flush();
  num_cc = get_cc_stats (g,cmap,ccstt);
  while (num_cc > 1) {
    for (unsigned int i=0; i<ccstt.size()-1; ++i) {
      w = rand() % N;
      int v1 = ccstt[i].second;
      int v2 = ccstt[i+1].second;
      g.add_edge(v1, v2, w);
    }
    num_cc = get_cc_stats (g,cmap,ccstt);
  }
  if(N < 15) display1(g);

  cout << "\n" << g.get_num_vertices() << ", " << g.get_num_edges() << "\t"; //  << endl;
}

template <class G, class BG, class WM>
void generate_boost_graph_from_stapl(G& g, BG& bg, WM& weightmap) {
  using namespace boost;

  for (typename G::vertex_iterator vi = g.begin(); vi != g.end(); ++vi) {
    add_vertex(bg);
  }
  weightmap = get(edge_weight, bg);
  for (typename G::edge_iterator ei = g.edges_begin(); ei != g.edges_end(); ++ei) {
    typename boost::graph_traits<BG>::edge_descriptor e; bool inserted;
    tie(e, inserted) = add_edge((*ei).descriptor().source(), (*ei).descriptor().target(), bg);
    weightmap[e] = (*ei).property();
  }
}

template <class ed>
struct lted {
  bool operator()(ed e1,ed e2) const {
    return e1.id() < e2.id();
  }
};

template <class Graph>
double total_weight(Graph& g) {
  typedef typename Graph::edge_iterator EI;
  double total_wt = 0;
  for (EI ei = g.edges_begin(); ei != g.edges_end(); ++ei) {
    total_wt += (*ei).property();
  }
  return total_wt;
}


template <class G, class ColorMap>
void test_core_bgl_graph(const G& g, ColorMap& cmap){
  typename G::const_vertex_iterator cvi = g.begin();
}

template <class G>
void delete_v(G& g, int n) {
  for (int i = 0; i < n; ++i) {
    int v1 = rand() % g.get_num_vertices();
    g.delete_vertex(v1);
  }
}

template <class G>
void add_v(G& g, int n){
  for (int i = 0; i < n; ++i) {
    // int N = g.get_num_vertices();
    g.add_vertex(N + rand()%N);
  }
  for (int i = 0; i < n; ++i) {
    int N = g.get_num_vertices()-1;
    double w = rand() % N;
    int v1 = rand() % N;
    int v2 = rand() % N;
    g.add_edge(v1, v2, w);
  }
}


int main(int argc, char** argv){
  // cout<<"New Graph test concepts\n";
  if(argc < 2) {
    // cout<<"by default N is 10\n";
    N=10;
    NE = 2*N;
  }
  else {
    if (argc < 3) {
      // cout<<"by default NE is O(2*N) \n";
      N = atoi(argv[1]);
      NE = 2*N;
    } else {
      N = atoi(argv[1]);
      NE = atoi(argv[2])*N;
    }
  }
  srand(time(NULL));

  // cout<<"test graph<UNDIRECTED, MULTIEDGES,int,weight_dijkstra>...\n";
  stapl::graph<stapl::UNDIRECTED,stapl::MULTIEDGES,int,double> g1;
  stapl::vector_property_map<stapl::graph_color<size_t> > cmap;
  stapl::static_graph_view<stapl::graph<stapl::UNDIRECTED,stapl::MULTIEDGES,int,double> > sv1(g1);

  cout << "calling buildgraph..." << endl; cout.flush();
  build_graph(g1);
  display_sizes(g1);

  cout << "calling bfs initially..." << endl; cout.flush();
  // bfs_test::bfs_test_core_graph(g1,cmap);
  mst_test::mst_test_core_graph(g1,cmap);

  double ND = .4 * (double) g1.get_num_vertices();
  cout << "ND = " << ND << endl;
  delete_v(g1, ND);
  add_v(g1, 2*ND);
  cout << "done modifying the graph, continue to algorithms..." << endl;
  mst_test::mst_test_core_graph(g1,cmap);

  // dijkstra_test::dijkstra_test_core_graph(g1,cmap);

  // cout << "calling bfs..." << endl; cout.flush();
  // bfs_test::bfs_test_core_graph(g1,cmap);

  // cout << "calling bfs a second time..." << endl; cout.flush();
  // bfs_test::bfs_test_core_graph(g1,cmap);

  /*
    cout<<"\ntest static_graph_view<graph<UNDIRECTED, MULTIEDGES,int,weight_dijkstra> >...\n";
    test_core_graph(sv1,cmap);

    cout<<"\ntest graph<UNDIRECTED, MULTIEDGES,my_vertex_property,weight_dijkstra>...\n";
    graph<UNDIRECTED,MULTIEDGES,my_vertex_property,weight_dijkstra> g2;
    internal_property_map<graph<UNDIRECTED,MULTIEDGES,my_vertex_property,weight_dijkstra>,graph_color<size_t> > cmap2(g2);
    build_graph(g2);
    test_core_graph(g2,cmap2);

    std::cout<<"\ntest directed_preds_graph<MULTIEDGES, int, weight_dijkstra>\n";
    directed_preds_graph<MULTIEDGES,int,weight_dijkstra> dpg;
    vector_property_map<graph_color<size_t> > cmap3;
    build_graph(dpg);
    test_core_graph(dpg,cmap3);

    cout<<"\ntest graph<DIRECTED,MULTIEDGES, int, weight_dijkstra, bgl_traits>...\n";
    typedef bgl_adaptor_traits<UNDIRECTED,MULTIEDGES,int, weight_dijkstra>  bgl_traits;
    graph<UNDIRECTED,MULTIEDGES, int, weight_dijkstra, bgl_traits> g_bgl;
    vector_property_map<graph_color<size_t> > cmap1;
    build_graph(g_bgl);
    test_core_graph(g_bgl,cmap1);
  */
  return 0;
}
