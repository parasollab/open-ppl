/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "test_util.h"

using namespace stapl;

template <class G>
void test_core_graph(G& g){
  bool err;
  size_t NE = 2*N;

  typename G::vertex_iterator vi;
  typename G::const_vertex_iterator cvi;

  typedef typename G::vertex_descriptor VD;
  std::vector<VD> vertices(N);
  vector<pair<VD,VD> >     edges(NE);
  vector<pair<VD,size_t> > edgeids(NE);
  typedef AddVerts<G,typename G::vertex_property> AVT;
  typedef AddEdges<G,typename G::edge_property>   AET;
  AVT AV;
  AET AE;

  AV.add(g,vertices,N);
  AE.add(g, vertices, edges, edgeids,NE);
  //check correctness add_vertex/edge
  //display(g);
  //g.display();

  cout<<"Test add_vertex..........";
  err = false;
  size_t nv = g.get_num_vertices();
  if (nv != N) err = true;
  vi=g.begin();
  for (size_t i=0;i<N;++i){
    if ((*vi).descriptor() != vertices[i]) err = true;
    if (!AVT::test_property(vi)) err = true;
    ++vi;
  }
  if (err) cout<<"Failed\n";
  else cout<<"Passed\n";

  cout<<"Test add_edge..........";
  if (g.get_num_edges() == NE) cout<<"Passed\n";
  else cout <<"Failed\n";

  cout<<"Test find_vertex..........";
  err = false;
  for (size_t i=0;i < N; ++i) {
    vi = g.find_vertex(vertices[i]);
    if ( vi==g.end() ) err = true;
    if ((*vi).descriptor() != vertices[i]) err= true;
    //this doesn't work for now; you have to be in a const method to call it
    //cvi = g.find_vertex(vertices[i]);
  }
  if (err) cout<<"Failed\n";
  else cout<<"Passed\n";

  cout<<"Test find_vertex..........";
  err = false;
  for (size_t i=0;i < N; ++i) {
    vi = g.find_vertex(vertices[i]);
    if ( vi==g.end() ) err = true;
    if ((*vi).descriptor() != vertices[i]) err= true;
    //this doesn't work for now; you have to be in a const method to call it
    //cvi = g.find_vertex(vertices[i]);
  }
  if (err) cout<<"Failed\n";
  else cout<<"Passed\n";

  cout<<"Test find_edge...........";
  err=false;
  typename G::edge_iterator gei, gei_end;
  typename G::adj_edge_iterator aei = (*g.begin()).begin();
  gei_end = g.edges_end();
  for (gei=g.edges_begin();gei != gei_end; ++gei){
    typename G::edge_descriptor ed = (*gei).descriptor();
    bool b = g.find_edge(ed, vi, aei);
    if (!b) err = true;
    else {
      if (g.is_directed()){
        if (ed.source() != (*vi).descriptor()) err = true;
        if (ed.source() != (*aei).source()) err = true;
        if (ed.target() != (*aei).target()) err = true;
      }
      else{
        //for an undirected graph we return the edge with the source
        //smaller or equal than target
        if (ed.source() <= ed.target()) {
          if (ed.source() != (*vi).descriptor()) err = true;
          if (ed.source() != (*aei).source()) err = true;
          if (ed.target() != (*aei).target()) err = true;
        }
        else{//they are reversed
          if (ed.target() != (*vi).descriptor()) err = true;
          if (ed.target() != (*aei).source()) err = true;
          if (ed.source() != (*aei).target()) err = true;
        }
      }
      if (ed.id() != (*aei).id()) err = true;
      if ( !AET::test_property(ed,aei) ) err = true;
    }
  }
  if (err) cout<<"Failed\n";
  else cout<<"Passed\n";

  cout<<"Test delete_edge...........";
  err=false;
  vector<typename G::edge_descriptor> v_ed(g.get_num_edges());
  typename vector<typename G::edge_descriptor>::iterator edi = v_ed.begin();
  gei_end = g.edges_end();
  for (gei=g.edges_begin();gei != gei_end; ++gei){
    *edi = (*gei).descriptor();
    ++edi;
  }
  size_t ne = g.get_num_edges();
  for (edi=v_ed.begin(); edi != v_ed.end();++edi){
    typename G::edge_descriptor ed(*edi);
    g.delete_edge(ed);
    if (g.get_num_edges() != ne-1) err = true;
    ne--;
#if 0
    // test incorrect; ed is keeping a reference to the deleted edge and any
    // memory checking tool will rightfully complain about this. See bug #1247
    bool b = g.find_edge(ed, vi, aei);
    if (b) err = true;
#endif
  }
  if (err) cout<<"Failed\n";
  else cout<<"Passed\n";

  cout<<"Test delete_vertex..........";
  AE.add(g, vertices, edges, edgeids,NE);
  err =false;
  for (size_t i=0;i<N;++i){
    if (!g.delete_vertex(vertices[i])) err = true;
    //cout<<"----------------------------------------"
    //<<g.get_num_vertices()<<"\n";
    //display(g);
    if (g.get_num_vertices() != N-i-1) err = true;
    vi = g.find_vertex(vertices[i]);
    if (vi != g.end()) err = true;
  }
  if (err) cout<<"Failed\n";
  else cout<<"Passed\n";

  cout<<"Test clear() graph..........";
  AV.add(g,vertices,N);
  AE.add(g, vertices, edges, edgeids,NE);
  //display(g);
  //cout<<"----------------------------------------"
  //<<g.get_num_vertices()<<"\n";
  err =false;
  g.clear();
  if (g.get_num_edges() != 0) err = true;
  if (g.get_num_vertices() != 0) err = true;
  if (err) cout<<"Failed\n";
  else cout<<"Passed\n";

  cout<<"Test graph constructor(size) ..........";
  err = false;
  G new_g(N) ; //constructor with size
  vertices.clear();
  for (typename G::vertex_iterator nvi = new_g.begin();
       nvi != new_g.end();++nvi){
    vertices.push_back((*nvi).descriptor());
  }
  if (new_g.get_num_vertices() != N) err = true;
  AE.add(new_g, vertices, edges, edgeids,NE);
  gei_end = new_g.edges_end();
  for (gei=new_g.edges_begin();gei != gei_end; ++gei){
    typename G::edge_descriptor ed = (*gei).descriptor();
    bool b = new_g.find_edge(ed, vi, aei);
    if (!b) err = true;
    else {
      if (new_g.is_directed()){
        if (ed.source() != (*vi).descriptor()) err = true;
        if (ed.source() != (*aei).source()) err = true;
        if (ed.target() != (*aei).target()) err = true;
      }
      else{
        //for an undirected graph we return the edge with the source
        //smaller or equal than target
        if (ed.source() <= ed.target()) {
          if (ed.source() != (*vi).descriptor()) err = true;
          if (ed.source() != (*aei).source()) err = true;
          if (ed.target() != (*aei).target()) err = true;
        }
        else{//they are reversed
          if (ed.target() != (*vi).descriptor()) err = true;
          if (ed.target() != (*aei).source()) err = true;
          if (ed.source() != (*aei).target()) err = true;
        }
      }
      if (ed.id() != (*aei).id()) err = true;
    }
  }
  if (err) cout<<"Failed\n";
  else cout<<"Passed\n";

  cout<<"Test graph copy constructor() ..........";
  err = false;
  g.clear();
  AV.add(g,vertices,N);
  AE.add(g, vertices, edges, edgeids,NE);
  G new_ccg(g) ; //constructor with size
  {
    if (g.get_num_vertices() != new_ccg.get_num_vertices()) err = true;
    if (g.get_num_edges() != new_ccg.get_num_edges())       err = true;
    vi = g.begin();
    for (typename G::vertex_iterator nvi = new_ccg.begin();
         nvi != new_ccg.end();++nvi,++vi){
      if ((*nvi).descriptor() != (*vi).descriptor())           err = true;
      if ((*nvi).size() != (*vi).size()) err = true;
    }
    if (g.is_directed()){
      //undirected will have edges in a different order in the new graph
      typename G::edge_iterator ei = g.edges_begin();
      for (typename G::edge_iterator nei=new_ccg.edges_begin();
           nei != new_ccg.edges_end();++nei,++ei){
        if ((*ei).source() != (*nei).source()) err = true;
        if ((*ei).target() != (*nei).target()) err = true;
        if ((*ei).id() != (*nei).id()) err = true;
      }
    }
  }
  if (err) cout<<"Failed\n";
  else cout<<"Passed\n";

  cout<<"Test invalid add_edge()..........";
  err = false;
  g.clear();
  typename G::edge_descriptor ed = g.add_edge(0,1);
  if (is_source_valid(ed)) err = true;
  //if (is_target_valid(ed)) err = true;
  if (is_id_valid(ed))     err = true;
  if (is_valid(ed))        err = true;
  if (err) cout<<"Failed\n";
  else cout<<"Passed\n";


  cout<<"Test const_edge_iterator...........";

  cvi = vi; //testing assignment of vi to const vi
  typename G::const_edge_iterator gcei;
  gcei = gei;//assign ei to const_ei

  err=false;  ne=0;
  g.clear();
  AV.add(g,vertices,N);
  AE.add(g, vertices, edges, edgeids,NE);
  const G& cgref = g;
  for (gcei=cgref.edges_begin();gcei != cgref.edges_end(); ++gcei){
    ne++;
  }
  if (ne != g.get_num_edges()) err = true;
  if (err) {
    cout<<"Failed\n";
    //display(g);
    //std::cout<<ne<<"----->"<<g.get_num_edges()<<"\n";
  }
  else cout<<"Passed\n";
}


//======================================================== IO
template <class G>
void test_io(G& g){
  typedef typename G::vertex_descriptor VD;
  std::vector<VD> vertices(N);
  size_t NE = 2*N;
  vector<pair<VD,VD> >     edges(NE);
  vector<pair<VD,size_t> > edgeids(NE);
  AddVerts<G,typename G::vertex_property> AV;
  AddEdges<G,typename G::edge_property>   AE;

  AV.add(g,vertices,N);
  AE.add(g, vertices, edges, edgeids,NE);
  //display(g);
  write_vertices_edges_graph(g,"vertices_edges.graph");
  read_vertices_edges_graph(g,"vertices_edges.graph");
  //display(g);
  write_graph(g,"mp.graph");
  read_graph(g,"mp.graph");
}

//predecessors specific methods
template <class G>
bool test_pred_info(G& g){
  bool err = false;
  //check that for all edges there is a pred
  for (typename G::vertex_iterator vi =g.begin(); vi != g.end();++vi){
    typename G::adj_edge_iterator ei;
    ei = (*vi).begin();
    while (ei != (*vi).end()){
      typename G::vertex_iterator vi2 = g.find_vertex((*ei).target());
      typename G::preds_iterator pi =
        std::find((*vi2).predecessors().begin(), (*vi2).predecessors().end(),
                  (*ei).source());
      if (pi == (*vi2).predecessors().end()) err = true;
      ++ei;
    }
  }
  //and that for all preds there is an edge
  for (typename G::vertex_iterator vi =g.begin(); vi != g.end();++vi){
    typename G::preds_iterator pi;
    pi = (*vi).predecessors().begin();
    while (pi != (*vi).predecessors().end()){
      typename G::vertex_iterator vi2 = g.find_vertex(*pi);
      typename G::adj_edge_iterator ei;
      ei =
        graph_find((*vi2).begin(), (*vi2).end(),
                  eq_target<typename G::vertex_descriptor>((*vi).descriptor()));
      if (ei == (*vi2).end()) err = true;
      ++pi;
    }
  }
  //and that for all nodes get_in_degree works
  for (typename G::vertex_iterator vi = g.begin(); vi != g.end(); ++vi)
    if(g.get_in_degree(vi->descriptor()) != (*vi).predecessors().size())
      err = true;
  return err;
}

template <class G>
void test_predecessors(G& g){
  typedef typename G::vertex_descriptor VD;
  std::vector<VD> vertices(N);
  size_t NE = 2*N;
  vector<pair<VD,VD> >     edges(NE);
  vector<pair<VD,size_t> > edgeids(NE);
  AddVerts<G,typename G::vertex_property> AV;
  AddEdges<G,typename G::edge_property>   AE;
  bool err;

  AV.add(g,vertices,N);
  AE.add(g, vertices, edges, edgeids,NE);

  cout<<"Test if predecessors are set up correctly...........";
  err = test_pred_info(g);
  if (err) cout<<"Failed\n";
  else cout<<"Passed\n";

  cout<<"Test set lazy predecessors...........";
  g.clear();
  g.set_lazy_update(true);
  AV.add(g,vertices,N);
  AE.add(g,vertices, edges, edgeids,NE);

  g.set_lazy_update(false);
  err = test_pred_info(g);

  if (err) cout<<"Failed\n";
  else cout<<"Passed\n";

  cout<<"Test directed with predecessors graph copy constructor(size) ........";
  err = false;
  g.clear();
  AV.add(g,vertices,N);
  AE.add(g, vertices, edges, edgeids,NE);
  G new_ccg(g) ; //constructor with size
  if (g.get_num_vertices() != new_ccg.get_num_vertices()) err = true;
  if (g.get_num_edges() != new_ccg.get_num_edges())       err = true;
  typename G::vertex_iterator vi = g.begin();
  for (typename G::vertex_iterator nvi = new_ccg.begin();
       nvi != new_ccg.end();++nvi,++vi){
    if ((*nvi).descriptor() != (*vi).descriptor())           err = true;
    if ((*nvi).size() != (*vi).size()) err = true;
    typename G::adj_edge_iterator ei = (*vi).begin();
    for (typename G::adj_edge_iterator nei=(*nvi).begin();
         nei != (*nvi).end();++nei,++ei){
      if ((*ei).source() != (*nei).source()) err = true;
      if ((*ei).target() != (*nei).target()) err = true;
      if ((*ei).id() != (*nei).id()) err = true;
    }
    //test pred info
    if ((*nvi).predecessors().size() != (*vi).predecessors().size()) err = true;
    typename G::preds_iterator pei = (*vi).predecessors().begin();
    for (typename G::preds_iterator npei=(*nvi).predecessors().begin();
         npei != (*nvi).predecessors().end();++npei,++pei){
      if (*pei != *npei) err = true;
    }
  }
  if (err) cout<<"Failed\n";
  else cout<<"Passed\n";
}//end test predecessors

//======================================================= MAIN
int main(int argc, char** argv){
  cout<<"New Graph test concepts\n";
  if (argc < 2) {
    cout<<"by default N is 10\n";
    N=10;
  }
  else N = atoi(argv[1]);
  if (N==1){cout<<"We need at least two vertices\n"; N=2;}

  srand(N);

  std::cout<<"\n>>> Testing graph<DIRECTED, MULTIEDGES, int, weight>\n";
  sequential::graph<DIRECTED,MULTIEDGES,int,weight> g1;
  test_core_graph(g1);
  test_io(g1);

  std::cout<<"\n>>> Testing graph<DIRECTED, NONMULTIEDGES>\n";
  sequential::graph<DIRECTED,MULTIEDGES> g11;
  test_core_graph(g11);

  std::cout<<"\n>>> Testing directed_preds_graph<MULTIEDGES, int, weight>\n";
  sequential::directed_preds_graph<MULTIEDGES,int,weight> dpg;
  test_core_graph(dpg);
  test_predecessors(dpg);

  std::cout<<"\n>>> Testing directed_preds_graph<MULTIEDGES>\n";
  sequential::directed_preds_graph<MULTIEDGES> dpg2;
  test_core_graph(dpg2);
  test_predecessors(dpg2);

  std::cout<<"\n>>> Testing graph<UNDIRECTED, MULTIEDGES, int, weight>\n";
  sequential::graph<UNDIRECTED,MULTIEDGES,int,weight> g2;
  test_core_graph(g2);

  std::cout<<"\n>>> Testing graph<UNDIRECTED, MULTIEDGES>\n";
  sequential::graph<UNDIRECTED,MULTIEDGES> g22;
  test_core_graph(g22);

  std::cout<<
    "\n\n>>> Testing BGL based graph<DIRECTED, MULTIEDGES, int, weight>\n";
  typedef sequential::bgl_adaptor_traits<DIRECTED,MULTIEDGES,int,weight>
    bgl_traits;
  sequential::graph<DIRECTED,MULTIEDGES, int, weight, bgl_traits> g_bgl;
  test_core_graph(g_bgl);

  return 0;
}
