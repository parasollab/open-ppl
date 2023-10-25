/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "test_util.h"
#include <limits>

#include <stapl/containers/sequential/graph/algorithms/breadth_first_search.h>
#include <stapl/containers/sequential/graph/algorithms/graph_generators.h>
#include <stapl/containers/sequential/graph/algorithms/diameter.h>
#include <stapl/containers/sequential/graph/algorithms/count_hop_pairs.h>
#include <stapl/containers/sequential/graph/algorithms/count_triangles.h>
#include <stapl/containers/sequential/graph/algorithms/graph_algo.h>
#include <stapl/containers/sequential/graph/algorithms/find_cycle.h>
#include <stapl/containers/sequential/graph/algorithms/connected_components.h>
#include <stapl/containers/sequential/graph/algorithms/dijkstra.h>
#include <stapl/containers/sequential/graph/algorithms/mst_graph.h>

using namespace stapl;
using namespace stapl::sequential;
using namespace std;

class weight_dijkstra
{
public:
  size_t source;
  size_t target;
  size_t id;
  double d;
  weight_dijkstra()
    : source(9999), target(9999), id(999), d(1)
  {}
  weight_dijkstra(double d_in)
    : source(9999), target(9999), id(999), d(d_in)
  {}
  weight_dijkstra(size_t s, size_t t)
    : source(s), target(t), id(999), d(1)
  {}

  bool operator==(const weight_dijkstra& _other) const
  {
    return source==_other.source &&
      target==_other.target &&
      id==_other.id && d == _other.d;
  }
  weight_dijkstra operator+(const weight_dijkstra& _other) const
  { return weight_dijkstra(d+_other.d); }
  bool operator<(const weight_dijkstra& _other) const
  { return d < _other.d; }
  double& Weight()
  { return d; }
  static double MaxWeight()
  {return 99999;}
};

class my_vertex_property
{
  typedef size_t color_value_type;
  color_value_type m_color;
  double m_flux;
public:
  my_vertex_property(size_t i=999)
    : m_color(0), m_flux(i)
  {}
  void set_color(color_value_type _c)
  { m_color = _c; }
  color_value_type get_color() const
  { return m_color; }
  double& flux()
  { return m_flux;}
};

class my_color_map_func{
public:
  typedef size_t value_type;

  value_type get(my_vertex_property& mvp)
  {return mvp.get_color();}
  void put(my_vertex_property& mvp, value_type _v)
  {mvp.set_color(_v);}

  template <class Functor>
  void apply(my_vertex_property& mvp, Functor _f)
  {}
};

template< class GRAPH>
class visitor_display
{
  typedef typename GRAPH::vertex_iterator   vertex_iterator;
  typedef typename GRAPH::adj_edge_iterator adj_edge_iterator;
  typedef typename GRAPH::vertex_descriptor VD;
  typedef typename GRAPH::edge_descriptor ED;

 public:
  vector<VD> m_vd;
  vector<ED> m_ed;
  void reset()
  {
    m_vd.clear();
    m_ed.clear();
  }

  visitor_display() = default;
  visitor_display(GRAPH& _g)
  {}

  visitor_return discover_vertex(vertex_iterator _vi)
  {
    m_vd.push_back((*_vi).descriptor());
#ifdef _DISPLAY
    cout<<"vertex discovered:"<<(*_vi).descriptor()<<"\n";
#endif
    return CONTINUE;
  }

  visitor_return examine_vertex(vertex_iterator /*_vi*/)
  {
    return CONTINUE;
  }

  visitor_return examine_edge(vertex_iterator /*_vi*/,
                              adj_edge_iterator /*_ei*/)
  {
#ifdef _DISPLAY
    cout<<"examine edge: ("<<(*_ei).source()<<"-->"<<(*_ei).target()<<")\n";
#endif
    return CONTINUE;
  }
  visitor_return tree_edge(vertex_iterator /*_vi*/, adj_edge_iterator _ei)
  {
    m_ed.push_back( (*_ei).descriptor() );
#ifdef _DISPLAY
    cout<<"tree edge: ("<<(*_ei).source()<<"-->"<<(*_ei).target()<<")\n";
#endif
    return CONTINUE;
  }

  visitor_return non_tree_edge(vertex_iterator /*_vi*/,
                               adj_edge_iterator /*_ei*/)
  {
#ifdef _DISPLAY
    cout<<"non tree edge: ("<<(*_ei).source()<<"-->"<<(*_ei).target()<<")\n";
#endif
    return CONTINUE;
  }

  visitor_return gray_target(vertex_iterator /*_vi*/, adj_edge_iterator _ei)
  {
    m_ed.push_back( (*_ei).descriptor() );
#ifdef _DISPLAY
    cout<<"gray target edge: ("<<(*_ei).source()<<"-->"<<(*_ei).target()<<")\n";
#endif
    return CONTINUE;
  }

  visitor_return black_target(vertex_iterator /*_vi*/, adj_edge_iterator _ei)
  {
    m_ed.push_back( (*_ei).descriptor() );
#ifdef _DISPLAY
    cout << "black target edge: (" << (*_ei).source() << "-->"
         << (*_ei).target() << ")\n";
#endif
    return CONTINUE;
  }

  visitor_return finish_vertex(vertex_iterator /*_vi*/, int =-1)
  {
#ifdef _DISPLAY
    cout<<"vertex finished:"<<(*_vi).descriptor()<<"\n";
#endif
    return CONTINUE;
  }
};

template <class G, class VD>
void tree_add(G& g,
              vector<VD>& vertices,
              vector<pair<VD,VD> >&,
              vector<pair<VD,size_t> >&,
              size_t& NE)
{
  //this builda tree and a separate vertex (two components)
  //used for testing
  size_t nv = vertices.size();
  NE = 0;

  g.add_edge(0,1);
  g.add_edge(1,0);

  for(size_t i=1;i < nv; ++i) {
    if (2*i < nv-1)
    {
      g.add_edge(i+1,2*i+1);NE++;
    }
    if (2*i+1 < nv-1)
    {
      g.add_edge(i+1,2*i+1+1);NE++;
    }
  }
}


template<class Graph>
struct adapt_edge_to_distance
{
  typedef typename Graph::adj_edge_iterator argument_type;
  typedef double&                           result_type;

  double& operator()(typename Graph::adj_edge_iterator it)
  { return it.property().Weight(); }
};

template <class G>
void build_graph(G& g)
{
  size_t NE = 2*N;

  typename G::vertex_iterator vi;
  typename G::const_vertex_iterator cvi;

  typedef typename G::vertex_descriptor VD;
  std::vector<VD> vertices(N);
  vector<pair<VD,VD> >     edges(NE);
  vector<pair<VD,size_t> > edgeids(NE);
  AddVerts<G,typename G::vertex_property> AV;
  //AddEdges<G,typename G::edge_property>   AE;

  AV.add(g,vertices,N);
  tree_add(g, vertices, edges, edgeids,NE);
  if (N < 10)
    display(g);
}

/**
 * single-source bfs
 * full bfs
 * single-source bfs-early quit
 * full bfs-early quit
 * single-source dfs
 * full dfs
 * single-source dfs-early quit
 * full dfs-early quit
 * is_cycle
 * get_back_edges
 * is_same_cc
 * get_cc_stats
 * get_cc_edges
 * is_clique
 * find_path_dijkstra
 * topological_traversal
 */
template <class G, class ColorMap>
void test_core_graph(G& g, ColorMap& cmap)
{
  typedef typename G::vertex_descriptor VD;
  bool err;
  size_t NE = g.get_num_edges();
  visitor_display<G> vis;

  cout<<"test bfs(source).....";
  err = false;
  cmap.reset();
  breadth_first_search(g,0,vis,cmap);
  if (vis.m_vd.size() != 2)
    err = true;
  if (vis.m_ed.size() != 2)
    err = true;
  vis.reset();
  cmap.reset();
  breadth_first_search(g,2,vis,cmap);
  if (vis.m_vd.size() != N-2)
    err = true;
  if (vis.m_ed.size() != NE-2)
    err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";

  cout<<"test bfs(all).....";
  err = false;
  cmap.reset();
  vis.reset();
  breadth_first_search(g,vis,cmap);
  if (vis.m_vd.size() != N)
    err = true;
  if (vis.m_ed.size() != NE)
    err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";

  cout<<"test bfs_early_quit(source).....";
  err = false;
  vis.reset();
  cmap.reset();
  breadth_first_search(g,0,vis,cmap);
  if (vis.m_vd.size() != 2)
    err = true;
  if (vis.m_ed.size() != 2)
    err = true;
  vis.reset();
  cmap.reset();
  breadth_first_search_early_quit(g,2,vis,cmap);
  if (vis.m_vd.size() != N-2)
    err = true;
  if (vis.m_ed.size() != NE-2)
    err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";

  cout<<"test bfs_early_quit(all).....";
  err = false;
  cmap.reset();
  vis.reset();
  breadth_first_search_early_quit(g, vis, cmap);
  if (vis.m_vd.size() != N)
    err = true;
  if (vis.m_ed.size() != NE)
    err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";

  ///////////////////////////////////////////////////// start dfs traversals
  cout<<"test dfs(source).....";
  err = false;
  cmap.reset();
  vis.reset();
  depth_first_search(g,0,vis,cmap);
  if (vis.m_vd.size() != 2)
    err = true;
  if (vis.m_ed.size() != 2)
    err = true;
  vis.reset();
  cmap.reset();
  depth_first_search(g,2,vis,cmap);
  if (vis.m_vd.size() != N-2)
    err = true;
  if (vis.m_ed.size() != NE-2)
    err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";

  cout<<"test dfs(all).....";
  err = false;
  cmap.reset();
  vis.reset();
  depth_first_search(g,vis,cmap);
  if (vis.m_vd.size() != N)
    err = true;
  if (vis.m_ed.size() != NE)
    err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";

  cout<<"test dfs_early_quit(source).....";
  err = false;
  vis.reset();
  cmap.reset();
  depth_first_search(g,0,vis,cmap);
  if (vis.m_vd.size() != 2)
    err = true;
  if (vis.m_ed.size() != 2)
    err = true;
  vis.reset();
  cmap.reset();
  depth_first_search_early_quit(g,2,vis,cmap);
  if (vis.m_vd.size() != N-2)
    err = true;
  if (vis.m_ed.size() != NE-2)
    err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";

  cout<<"test dfs_early_quit(all).....";
  err = false;
  cmap.reset();
  vis.reset();
  depth_first_search_early_quit(g, vis, cmap);
  if (vis.m_vd.size() != N)
    err = true;
  if (vis.m_ed.size() != NE)
    err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";
  //////////////////////////// end dfs

  cout<<"test is_cycle.....";
  err = false;
  cmap.reset();
  if (is_cycle(g,cmap) != true)
    err = true;
  else
    err = false;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";

  cout<<"test back_edges.....";
  vector<pair<VD,VD> > be;
  cmap.reset();
  get_back_edges(g, cmap, be);
#ifdef _DISPLAY
  for (size_t i=0; i<be.size(); ++i) {
    std::cout<<be[i].first<<"-->"<<be[i].second<<"\n";
  }
#endif
  if (be.size() != 1)
    err = true;
  if (!err)
    if (be[0].first != 0 || be[0].second != 1)
      err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";

  cout<<"test same_cc.....";
  err = false;
  cmap.reset();
  if (is_same_cc(g,cmap, 0, 1) != true)
    err = true;
  cmap.reset();
  if (is_same_cc(g,cmap, 0, 2) == true)
    err = true;
  cmap.reset();
  if (is_same_cc(g,cmap, 2, 3) != true)
    err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";

  cout<<"test get_cc.....";
  err = false;
  vector<VD> cc;
  cmap.reset();
  get_cc(g,cmap,0,cc);
#ifdef _DISPLAY
  for(size_t i=0;i<cc.size();++i){
    std::cout<<cc[i]<<"--\n";
  }
#endif
  if (cc.size() != 2)
    err = true;
  cmap.reset();
  cc.clear();
  get_cc(g,cmap,2,cc);
  if (cc.size() != N-2)
    err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";

  cout<<"test get_cc_stats.....";
  err = false;
  vector<pair<size_t,VD> > ccs;
  cmap.reset();
  get_cc_stats(g,cmap,ccs);
#ifdef _DISPLAY
  for (size_t i=0; i<ccs.size(); ++i) {
    std::cout<<ccs[i].first<<"-->"<<ccs[i].second<<"\n";
  }
#endif
  if (ccs.size() != 2)
    err = true;
  if (!err)
    if (ccs[0].first != 2 && ccs[1].first != N-2)
      err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";

  cout<<"test get_cc_edges.....";
  vector<pair<VD,VD> > cc_e;
  cmap.reset();
  get_cc_edges(g,cmap,cc_e,0);
#ifdef _DISPLAY
  for (size_t i=0; i<cc_e.size(); ++i) {
    std::cout<<cc_e[i].first<<"-->"<<cc_e[i].second<<"\n";
  }
#endif
  if (cc_e.size() != 2)
    err = true;
  cmap.reset();
  cc_e.clear();
  get_cc_edges(g,cmap,cc_e,2);
  if (cc_e.size() != NE-2)
    err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";

  cout<<"test is_clique().....";
  err = false;
  if (!is_clique(g,0))
    err = true;
  if (is_clique(g,2))
    err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";

  cout<<"test find_path_dijkstra.....";
  err = false;
  vector<pair<typename G::vertex_property,typename G::edge_property> > vpath;
  vector<VD> dpath;
  find_path_dijkstra(g,0,1,vpath, weight_dijkstra::MaxWeight());
  //nbe = find_path_dijkstra(g,0,1,dpath);
#ifdef _DISPLAY
  for (size_t i=0; i<vpath.size(); ++i) {
    std::cout<<vpath[i].first.flux()<<"-->"<<vpath[i].second.Weight()<<"\n";
  }
#endif
  if (vpath.size() != 2)
    err = true;
  vpath.clear();
  find_path_dijkstra(g,2,5,vpath, weight_dijkstra::MaxWeight());
  if (vpath.size() != 3)
    err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";

  /*
  cout<<"test dijkstra_ssp.....";
  err=false;
  graph<DIRECTED,NONMULTIEDGES,
    typename G::vertex_property,
    typename G::edge_property> ssp_tree;

  std::vector <size_t> p_map(g.get_num_vertices());
  std::vector<typename G::edge_property> w_map(g.get_num_edges());
  dijkstra_sssp(g, p_map, w_map, 2);
  convert_to_graph(p_map, w_map, ssp_tree);
#ifdef _DISPLAY
  display(ssp_tree);
#endif
  if (ssp_tree.get_num_vertices() != N-2)
    err = true;
  if (ssp_tree.get_num_edges() != NE-2)
    err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";
  */
  cout<<"test topological traversal .....";
  err=false;
  typedef std::vector<size_t>      t_map_cont_type;
  typedef ident_prop_func<size_t>  t_map_func_type;
  typedef graph_external_property_map<G,
                                      t_map_cont_type,
                                      t_map_func_type> t_map_type;
  t_map_cont_type t_map_cont(g.get_max_descriptor());
  t_map_type t_map(t_map_cont, t_map_func_type());
  vis.reset();
  topological_traversal(g,vis,t_map);
  if (vis.m_vd.size() != N-2)
    err = true;
  if (vis.m_ed.size() != NE-2)
    err = true;
  if (err)
    cout<<"Failed\n";
  else
    cout<<"Passed\n";
}


template <class G, class ColorMap>
void test_core_bgl_graph(const G& g, ColorMap& cmap)
{
  typename G::const_vertex_iterator cvi = g.begin();
}

/**
 * diameter
 * count_local_triangles
 * count_hop_pairs
 */
template <typename Graph>
void test_metrics(Graph& g, size_t num_verts)
{
  cout<<"Building a star graph with "<<num_verts<<" vertices.\n";
  build_star_graph(g, num_verts);

  bool err = false;
  cout<<"Test count_local_triangles.....";
  size_t num_triangles = count_local_triangles(g, (*g.begin()).descriptor());
  if (num_triangles != 0)
    cout<<"Failed.\n";
  else
    cout<<"Passed.\n";

  err = false;
  cout<<"Test count_hop_pairs.....";
  if (count_hop_pairs(g,1) != g.get_num_edges())
    err = true;
  if (count_hop_pairs(g,2) != (num_verts*(num_verts-1)/2))
    err = true;
  if (err)
    cout<<"Failed.\n";
  else
    cout<<"Passed.\n";

  err = false;
  cout<<"Test diameter.....";
  if (diameter(g,(*g.begin()).descriptor()) != 2)
    err = true;
  if (err)
    cout<<"Failed.\n";
  else
    cout<<"Passed.\n";

  cout<<"\nConnecting the graph's outer ring...\n";
  for(size_t i=1; i<num_verts; ++i)
    g.add_edge(i, (i+1)%num_verts != 0 ? i+1 : 1);

  err = false;
  cout<<"Test count_local_triangles.....";
  num_triangles = count_local_triangles(g, (*g.begin()).descriptor());
  if (num_triangles != num_verts-1)
    cout<<"Failed.\n";
  else
    cout<<"Passed.\n";

  err = false;
  cout<<"Test count_hop_pairs.....";
  if (count_hop_pairs(g,1) != g.get_num_edges())
    err = true;
  if (count_hop_pairs(g,2) != (num_verts*(num_verts-1)/2))
    err = true;
  if (err)
    cout<<"Failed.\n";
  else
    cout<<"Passed.\n";

  err = false;
  cout<<"Test diameter.....";
  if (diameter(g,(*g.begin()).descriptor()) != 2)
    err = true;
  if (err)
    cout<<"Failed.\n";
  else
    cout<<"Passed.\n";
//====================================================================
//COMPLETE GRAPH
//====================================================================
  cout<<"\nClearing and rebuilding the graph as a complete graph.\n";
  g.clear();
  build_complete_graph(g, num_verts);

  err = false;
  cout<<"Test count_local_triangles.....";
  num_triangles = count_local_triangles(g, (*g.begin()).descriptor());
  if (num_triangles != ((num_verts-1)*(num_verts-2)/2))
    cout<<"Failed.\n";
  else
    cout<<"Passed.\n";

  err = false;
  cout<<"Test count_hop_pairs.....";
  if (count_hop_pairs(g,1) != (num_verts*(num_verts-1)/2))
    cout<<"Failed.\n";
  else
    cout<<"Passed.\n";

  err = false;
  cout<<"Test diameter.....";
  if (diameter(g,(*g.begin()).descriptor()) != 1)
    err = true;
  if (err)
    cout<<"Failed.\n";
  else
    cout<<"Passed.\n";
}

int main(int argc, char** argv){
  cout<<"New Graph test concepts\n";
  if (argc < 2) {
    cout<<"by default N is 10\n";
    N=10;
  }
  else N = atoi(argv[1]);
  srand(N);

  typedef graph<DIRECTED,MULTIEDGES,int,weight_dijkstra>                graph_t;
  typedef graph<DIRECTED,MULTIEDGES,my_vertex_property,weight_dijkstra> graph_t2;
  typedef directed_preds_graph<MULTIEDGES,int,weight_dijkstra>          graph_t3;
  typedef bgl_adaptor_traits<DIRECTED,MULTIEDGES,int, weight_dijkstra>  bgl_traits;
  typedef graph<DIRECTED,MULTIEDGES, int, weight_dijkstra, bgl_traits>  graph_t4;
  typedef graph<UNDIRECTED,MULTIEDGES>                                  graph_t5;

  typedef vector_property_map<graph_t, size_t>             color_map_t;
  typedef vertex_property_map<graph_t2, my_color_map_func> color_map_t2;
  typedef vector_property_map<graph_t3, size_t>            color_map_t3;

{
  cout<<"test graph<DIRECTED, MULTIEDGES,int,weight_dijkstra>...\n";
  graph_t g1;
  static_graph_view<graph_t>  sv1(g1);
  build_graph(g1);
  color_map_t cmap;
  test_core_graph(g1,cmap);

  cout<<"\ntest static_graph_view<graph<DIRECTED, MULTIEDGES,int,weight_dijkstra> >...\n";
  test_core_graph(sv1,cmap);
}
{
  cout<<"\ntest graph<DIRECTED, MULTIEDGES,my_vertex_property,weight_dijkstra>...\n";
  graph<DIRECTED,MULTIEDGES,my_vertex_property,weight_dijkstra> g2;
  build_graph(g2);
  color_map_t2 cmap2(g2);
  test_core_graph(g2,cmap2);
}
{
  std::cout<<"\ntest directed_preds_graph<MULTIEDGES, int, weight_dijkstra>\n";
  graph_t3 dpg;
  build_graph(dpg);
  color_map_t cmap3;
  test_core_graph(dpg,cmap3);
}
{
  cout<<"\ntest graph<DIRECTED,MULTIEDGES, int, weight_dijkstra, bgl_traits>...\n";
  graph_t4 g_bgl;
  build_graph(g_bgl);
  color_map_t3 cmap1;
  test_core_graph(g_bgl,cmap1);
}
{
  cout<<"\ntest_metrics graph<UNDIRECTED,MULTIEDGES>...\n";
  graph_t5 g5;
  test_metrics(g5, N);
}
  return 0;
}
