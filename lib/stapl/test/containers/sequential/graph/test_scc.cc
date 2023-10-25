/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include "test_util.h"

#include <stapl/containers/sequential/graph/algorithms/scc.h>
#include <stapl/containers/sequential/graph/algorithms/scc_fhp.h>
#include <deque>

using namespace std;
using namespace stapl;
using namespace stapl::sequential;

class weight_dijkstra{
public:
  size_t source;
  size_t target;
  size_t id;
  double d;

  weight_dijkstra(void)
  { source=9999;target=9999;id=999;d=1; }

  weight_dijkstra(size_t s, size_t t)
    :source(s),target(t)
  { id=999;d=1; }

  bool operator==(weight const& other) const {
    return source==other.source &&
      target==other.target &&
      id==other.id;
  }
  double& Weight(void)
  { return d; }

  double MaxWeight(void)
  { return 99999; }
};


template <class G, class VD>
void scc_add(G& g,
  vector<VD>& vertices,
  vector<pair<VD,VD> >&,
  vector<pair<VD,size_t> >&,
  size_t&) {
  //this builda tree and a separate vertex (two components)
  //used for testing
  size_t nv = vertices.size();
  size_t  sccs = (nv<20)?10:nv/5;
  for (size_t i=0; i!=sccs; ++i) {
    VD v1 = rand() % (nv-3);
    g.add_edge(v1,v1+1);
    g.add_edge(v1+1,v1+2);
    g.add_edge(v1+2,v1);
  }
}

template <class T>
struct comp_vs : public std::binary_function<T, T, bool> {
  bool operator()(T const& x, T const& y) {
    return x[0] > y[0];
  }
};

template <class G>
void build_graph(G& g) {
  size_t NE = N;

  typename G::vertex_iterator vi;
  typename G::const_vertex_iterator cvi;

  typedef typename G::vertex_descriptor VD;
  std::vector<VD> vertices(N);
  vector<pair<VD,VD> >     edges(NE);
  vector<pair<VD,size_t> > edgeids(NE);
  AddVerts<G,typename G::vertex_property> AV;
  AddEdges<G,typename G::edge_property>   AE;

  AV.add(g,vertices,N);
  AE.add(g, vertices, edges, edgeids,NE);

  scc_add(g, vertices, edges, edgeids,NE);
  //or read from file
  //read_vertices_edges_graph(g,"vertices_edges.graph");
  if (N < 10) {
    display(g);
  }
}

template <class G, class ColorMap>
void test_core_graph(G& g, ColorMap& cmap) {
  typedef typename G::vertex_descriptor VD;
  bool err;


  cout<<"test scc algos .....";
  err = false;
  std::vector<std::vector<VD> > sccs;
  cmap.reset();
  scc_pinar(g,cmap,sccs);
  comp_vs<std::vector<VD> > cv;

  for (size_t i=0;i<sccs.size();++i) {
    std::sort(sccs[i].begin(),sccs[i].end());
  }
  std::sort(sccs.begin(), sccs.end(), cv);

  std::vector<std::vector<VD> > sccs1;
  cmap.reset();
  scc(g,cmap,sccs1);
  for (size_t i=0; i<sccs1.size(); ++i) {
    std::sort(sccs1[i].begin(),sccs1[i].end());
  }
  std::sort(sccs1.begin(), sccs1.end(), cv);
  for (size_t i=0; i<sccs1.size(); ++i) {
    std::sort(sccs1[i].begin(),sccs1[i].end());
    for (size_t j=0; j<sccs1[i].size(); ++j) {
      if (sccs[i][j] != sccs1[i][j]) {
        err = true;
      }
    }
  }
  if (err) {
    cout<<"Failed\n";
  } else {
    cout<<"Passed\n";
  }

}

int main(int argc, char** argv) {
  cout<<"\n>>Testing SCC graph algorithms\n";
  if (argc < 2) {
    cout<<"by default N is 10\n";
    N=10;
  }
  else N = atoi(argv[1]);
  srand(N);

  cout<<"test graph<DIRECTED, MULTIEDGES,int,weight_dijkstra>...\n";
  typedef graph<DIRECTED,MULTIEDGES,int,weight_dijkstra>  graph_t;
  typedef std::vector<size_t>                      color_map_cont_t;
  typedef ident_prop_func<size_t>                  color_map_func_t;
  typedef graph_external_property_map<graph_t,
                                      color_map_cont_t,
                                      color_map_func_t>   color_map_t;
  graph_t g1;
  static_graph_view<graph<DIRECTED,MULTIEDGES,int,weight_dijkstra> > sv1(g1);
  build_graph(g1);
  color_map_cont_t color_map_cont(g1.get_max_descriptor());
  color_map_t cmap(color_map_cont, color_map_func_t());
  test_core_graph(g1,cmap);

  std::cout << "\n>>> Testing "
            << "directed_preds_graph<MULTIEDGES, int, weight_dijkstra>\n";
  typedef directed_preds_graph<MULTIEDGES,int,weight_dijkstra>  graph_t2;
  typedef graph_external_property_map<graph_t2,
                                      color_map_cont_t,
                                      color_map_func_t>   color_map_t2;
  graph_t2 dpg;
  build_graph(dpg);
  color_map_cont = color_map_cont_t(dpg.get_max_descriptor());
  color_map_t2 cmap3(color_map_cont, color_map_func_t());
  test_core_graph(dpg,cmap3);

  cout << "\ntest "
       << "graph<DIRECTED,MULTIEDGES, int, weight_dijkstra, bgl_traits>...\n";
  typedef bgl_adaptor_traits<DIRECTED, MULTIEDGES,
                             int, weight_dijkstra>  bgl_traits;
  typedef graph<DIRECTED, MULTIEDGES,
                int, weight_dijkstra, bgl_traits>   graph_t3;
  typedef graph_external_property_map<graph_t3,
                                      color_map_cont_t,
                                      color_map_func_t>   color_map_t3;
  graph_t3 g_bgl;
  build_graph(g_bgl);
  color_map_cont = color_map_cont_t(g_bgl.get_max_descriptor());
  color_map_t3 cmap1(color_map_cont, color_map_func_t());
  test_core_graph(g_bgl,cmap1);

  return 0;
}
