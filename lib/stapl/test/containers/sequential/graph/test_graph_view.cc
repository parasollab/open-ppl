/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "test_util.h"
#include <stapl/containers/sequential/graph/vertex_iterator_adaptor.h>

using namespace stapl;

struct my_property{
  int x;
  int y;
  int z;
  my_property(){
    x=y=z=999;
  }
  my_property(size_t i){
    x=i;
    y=2*i;
    z=3*i;
  }
  my_property& operator=(size_t i){
    x=i;
    y=2*i;
    z=3*i;
    return *this;
  }
};

inline ostream& operator << (ostream& s, const my_property& w) {
  return s <<"["<< w.x <<" "<< w.y <<" "<< w.z<<"]";
}

template <class G>
void build_graph(G& g){
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

  if(N < 10) display(g);
}


template <class G>
void test_iterators(G& g, bool is_directed){
  typename G::vertex_iterator vi;
  //test edge iterator
  cout<<"test edge iterators++.....";
  bool err = false;
  typename G::edge_iterator ei = g.edges_begin();
  vi = g.begin();
  while (vi != g.end()){
    typename G::adj_edge_iterator aei = (*vi).begin();
    while(aei != (*vi).end()){
      if(is_directed){
	if((*ei).source() != (*aei).source() || (*ei).target() != (*aei).target() || (*ei).id() != (*aei).id()) err = true;
	++ei;
      }
      else{//undirected
	if((*aei).source()<(*aei).target()){
	  //cout<<(*ei).source()<<"->"<<(*ei).target()<<" |"<<(*ei).id()<<"\n";
	  if((*ei).source() != (*aei).source() || (*ei).target() != (*aei).target() || (*ei).id() != (*aei).id()) err = true;
	  ++ei;
	}
      }
      ++aei;
    }
    ++vi;
  }
  if(err) cout<<"failed\n";
  else cout<<"passed\n";
  
  cout<<"test edge iterator++ end.....";
  err = false;
  if(ei != g.edges_end()) {
    cout<<(*ei).source()<<"==>"<<(*ei).target()<<" |"<<(*ei).id()<<"\n";
    err = true;
  }
  if(err) cout<<"failed\n";
  else cout<<"passed\n";


  cout<<"test edge iterators--.....";
  err = false;
  ei = g.edges_end();
  vi = g.end();
  do{
    --vi;
    typename G::adj_edge_iterator aei = (*vi).end();
    if(aei != (*vi).begin()) {
      do {
	--aei;
	if(is_directed){
	  --ei;
	  if((*ei).source() != (*aei).source() || (*ei).target() != (*aei).target() || (*ei).id() != (*aei).id()) err = true;
	}
	else {//undirected
	  if((*aei).source() < (*aei).target()){
	    --ei;
	    //cout<<(*ei).source()<<"==>"<<(*ei).target()<<" |"<<(*ei).id()<<"\n";
	    if((*ei).source() != (*aei).source() || (*ei).target() != (*aei).target() || (*ei).id() != (*aei).id()) err = true;
	  }
	}
      } while(aei != (*vi).begin());
    }//if at least one edge
  } while (vi != g.begin()); 
  if(err) cout<<"failed\n";
  else cout<<"passed\n";

  cout<<"test edge iterator-- begin.....";
  err = false;
  if(ei != g.edges_begin()) {
    cout<<(*ei).source()<<"==>"<<(*ei).target()<<" |"<<(*ei).id()<<"\n";
    err = true;
  }
  if(err) cout<<"failed\n";
  else cout<<"passed\n";

}

template <class ViewGraph>
void test_adjacency_view(ViewGraph& v, size_t, size_t){
  size_t cnt = 0;
  size_t ecnt =0;
  
  cout<<"test adjacency view interface.....";
  bool err = false;
  typename ViewGraph::vertex_iterator vi = v.begin();
  while(vi != v.end()){
    //std::cout<<(*vi).descriptor()<<":>"<<(*vi).property()<<"\n";
    typename ViewGraph::adj_edge_iterator aei = (*vi).begin();
    typename ViewGraph::const_adj_edge_iterator caei = (*vi).begin();
    while(aei != (*vi).end()){
      //std::cout<<(*aei).source()<<"->"<<(*aei).target()<<"["<<(*aei).property()<<"]\n";
      ++aei;
      ++ecnt;
    }
    ++cnt;
    ++vi;
  }
  if(err) cout<<"failed ! ! ! ! ! ! ! !\n";
  else cout<<"passed\n";
}

template <class ViewGraph>
void test_static_view(ViewGraph& v, bool is_directed){
  bool err=false;
  typedef typename ViewGraph::edge_descriptor ED;
  typedef std::vector<typename ViewGraph::vertex_descriptor> VDS;
  typedef std::vector<typename ViewGraph::edge_descriptor> EDS;
  VDS  vds;
  EDS  eds;
  size_t cnt  = 0;
  size_t ecnt = 0;
  //test base class
  test_iterators(v,is_directed);
  typename ViewGraph::vertex_iterator vi = v.begin();
  while(vi != v.end()){
    vds.push_back((*vi).descriptor());
    typename ViewGraph::adj_edge_iterator aei = (*vi).begin();
    while(aei != (*vi).end()){ 
      ED ed((*aei).source(), (*aei).target());
      eds.push_back(ed);
      //eds.push_back( ED((*aei).source(), (*aei).target()) );
      ++aei; ++ecnt; 
    }
    ++cnt; ++vi;
  }

  cout<<"test iterator/get_num_vertices.....";
  if(cnt != v.get_num_vertices()) err = true;
  if(err) cout<<"failed\n";
  else cout<<"passed\n";
  cout<<"test iterator/get_num_edges.....";
  if(ecnt != v.get_num_edges()) err = true;
  if(err) cout<<"failed\n";
  else cout<<"passed\n";

  cout<<"test view::find_vertex.....";
  for(typename VDS::iterator it = vds.begin();it!=vds.end();++it){
    typename ViewGraph::vertex_iterator vit = v.find_vertex(*it);
    if(vit == v.end()) err = true;
    if((*vit).descriptor() != *it) err = true;
  }
  if(err) cout<<"failed\n";
  else cout<<"passed\n";

  cout<<"test view::find_edge.....";
  for(typename EDS::iterator it = eds.begin();it!=eds.end();++it){
    typename ViewGraph::vertex_iterator   vit;
    typename ViewGraph::adj_edge_iterator eit;
    bool res = v.find_edge(*it,vit,eit);
    if(res == false) err = true;
    if((*vit).descriptor() != (*it).source()) err = true;
    if((*eit).source() != (*it).source()) err = true;
    if((*eit).target() != (*it).target()) err = true;
  }
  if(err) cout<<"failed\n";
  else cout<<"passed\n";
}

/**
 * Different Adaptors for Vertices and Edges
 */
template<class Graph>
struct adapt_v_to_vd
{
  typedef typename Graph::vertex_iterator   argument_type;
  typedef typename Graph::vertex_descriptor result_type;

  result_type operator()(argument_type it)
  {
    return it.descriptor();
  }
};

template<class Graph>
struct adapt_v_to_property
{
  typedef typename Graph::vertex_iterator argument_type;
  typedef int&                            result_type;

  int& operator()(typename Graph::vertex_iterator it)
  {
    return it.property().x;
  }
};

template<class Graph>
struct adapt_e_to_property
{
  typedef typename Graph::vertex_iterator argument_type;
  typedef size_t&                         result_type;

  size_t& operator()(typename Graph::adj_edge_iterator it)
  {
    return it.property().target;
  }
};

/*
                          TO ADD
 test for adapt source, target
 adapt vertex as a pair<x,y>; it has to be by copy
 adapt an adapted view
 */

//======================================================= MAIN
int main(int argc, char** argv){
  cout<<"Test graph iterators\n";
  if(argc < 2) {
    cout<<"by default N is 10\n";
    N=10;
  }
  else N = atoi(argv[1]);
  srand(N);

  std::cout<<"\n>>> Testing views for graph<DIRECTED, MULTIEDGES, int, weight>\n";
  typedef graph<DIRECTED,MULTIEDGES,my_property,weight> DG1;
  DG1 g1; //graph
  build_graph(g1);
  std::cout<<"-Testing graph who is a view\n";
  test_adjacency_view(g1,g1.get_num_vertices(), g1.get_num_edges());
  test_static_view(g1,g1.is_directed());

  std::cout<<"-Testing adjacency_graph_view<G>\n";
  typedef adjacency_graph_view<DG1>  VG1;
  VG1 v1(g1); //view
  test_adjacency_view(v1,g1.get_num_vertices(), g1.get_num_edges());

  std::cout<<"-Testing static_graph_view<G>\n";
  typedef static_graph_view<DG1>  SVG_1;
  SVG_1 v2(g1); 
  test_static_view(v2,g1.is_directed());
  
  //////////////////////////////////////////////SIMPLE COMPOSED
  std::cout<<"-Testing adjacency_graph_view<AGV>\n";
  typedef adjacency_graph_view<VG1>  VG_2;
  VG_2 v3(v1); 
  test_adjacency_view(v3,g1.get_num_vertices(), g1.get_num_edges());

  std::cout<<"-Testing static_graph_view<SGV>\n";
  typedef static_graph_view<SVG_1>  SVG_2;
  SVG_2 v4(v2); 
  test_static_view(v2,g1.is_directed());

  ////////////////////////////////////// adaptor views
  //adapt vertex to vid
  std::cout<<"= Testing adjacency graph view: vertex adaptor to VD\n";
  typedef adapt_v_to_vd<DG1>  A_VD;
  typedef adjacency_graph_view<DG1,A_VD>  A_VG1;
  A_VG1 a_v1(g1);
  test_adjacency_view(a_v1, g1.get_num_vertices(), g1.get_num_edges());
  bool err = false;
  //testing the adapted values
  {
    std::cout<<"Test the adapted vertices...";
    A_VG1::vertex_iterator vi = a_v1.begin();
    while(vi != a_v1.end()){
      if((*vi).property() != (*vi).descriptor()) err = true;
      ++vi;
    }
    if(err) cout<<"failed\n";
    else cout<<"passed\n";
  }

  std::cout<<"= Testing static graph view: vertex adaptor to VD\n";
  typedef static_graph_view<DG1,A_VD>  SA_VG1;
  SA_VG1 sa_v1(g1);
  test_static_view(sa_v1,g1.is_directed());
  {
    std::cout<<"Test the adapted vertices...";
    SA_VG1::vertex_iterator vi = sa_v1.begin();
    while(vi != sa_v1.end()){
      if((*vi).property() != (*vi).descriptor()) err = true;
      ++vi;
    }
    if(err) cout<<"failed\n";
    else cout<<"passed\n";
  }


  //adapt vertex to proeprty
  std::cout<<"= Testing adjacency graph view; vertex adaptor to property\n";
  typedef adapt_v_to_property<DG1>  A_VP;
  typedef adjacency_graph_view<DG1,A_VP>  AP_VG1;
  AP_VG1 ap_v1(g1);
  test_adjacency_view(ap_v1,g1.get_num_vertices(), g1.get_num_edges());

  std::cout<<"=> Testing static graph view; vertex adaptor to property\n";
  typedef static_graph_view<DG1,A_VP>  SAP_VG1;
  SAP_VG1 sap_v1(g1);
  test_static_view( sap_v1, g1.is_directed() );
  {
    err = false;
    std::cout<<"Test the adapted vertices...";
    SAP_VG1::vertex_iterator vi = sap_v1.begin();
    while(vi != sap_v1.end()){
      if((*vi).property() != (int)(*vi).descriptor()) err = true;
      //std::cout<<(*vi).descriptor()<<":>"<<(*vi).property()<<"\n";
      *vi=5;
      ++vi;
    }
    if(err) cout<<"failed\n";
    else cout<<"passed\n";
  }
  {
    std::cout<<"Test reference to adapted value...";
    err = false;
    SAP_VG1::vertex_iterator vi = sap_v1.begin();
    while(vi != sap_v1.end()){
      //std::cout<<(*vi).descriptor()<<":>"<<(*vi).property()<<"\n";
      if((*vi).property() != 5) err = true;
      ++vi;
    }
    if(err) cout<<"failed\n";
    else cout<<"passed\n";
  }


  //adapt vertex and edge property
  std::cout<<"=> Testing static graph view; vertex adaptor and edge adaptor\n";
  typedef adapt_e_to_property<DG1>  A_EP;
  typedef static_graph_view<DG1,A_VP,A_EP>  SAP_VEG1;
  SAP_VEG1 sap_veg1(g1);
  {
    err = false;
    std::cout<<"Test adapted edges...";
    SAP_VEG1::vertex_iterator vi = sap_veg1.begin();
    while(vi != sap_veg1.end()){
      //std::cout<<(*vi).descriptor()<<":>"<<(*vi).property()<<"["<<*vi<<"]\n";
      SAP_VEG1::adj_edge_iterator aei = (*vi).begin();
      while(aei != (*vi).end()){
	//std::cout<<(*aei).source()<<"->"<<(*aei).target()<<"["<<(*aei).property()<<"]["<<*aei<<"]\n";
	if((*aei).property() != (*aei).target()) err = true;
	++aei;
      }
      ++vi;
    }
    if(err) cout<<"failed\n";
    else cout<<"passed\n";
  }

  std::cout<<"=> Testing static graph view; edge adaptor only\n";
  //adapt edge property only 
  typedef static_graph_view<DG1,use_default,A_EP>  SAP_VEG2;
  SAP_VEG2 sap_veg2(g1);
  {
    err = false;
    std::cout<<"Test adapted edges...";
    SAP_VEG2::vertex_iterator vi = sap_veg2.begin();
    while(vi != sap_veg2.end()){
      SAP_VEG2::adj_edge_iterator aei = (*vi).begin();
      while(aei != (*vi).end()){
	if((*aei).property() != (*aei).target()) err = true;
	++aei;
      }
      ++vi;
    }
    if(err) cout<<"failed\n";
    else cout<<"passed\n";
  }

}//end main
