/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_TEST_CONTAINERS_SEQUENTIAL_GRAPH_TEST_UTIL_H
#define STAPL_TEST_CONTAINERS_SEQUENTIAL_GRAPH_TEST_UTIL_H

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <stapl/containers/sequential/graph/graph.h>
#include <stapl/containers/sequential/graph/directed_preds_graph.h>
#include <stapl/containers/sequential/graph/bgl_core_graph.h>
#include <stapl/containers/sequential/graph/algorithms/graph_input_output.h>

using namespace std;

namespace stapl {

namespace runtime {

void assert_fail(const char* assertion, const char* file, unsigned int line, const char* function)
{
  std::cerr << assertion << " (file: " << file << ", function: " << function << ", line: " << line << ")\n";
  std::abort();
}

} // end namespace runtime

class weight{
public:
  size_t source;
  size_t target;
  size_t id;
  weight(){source=9999;target=9999;id=999;}
  weight(size_t s, size_t t):source(s),target(t){id=999;}
  bool operator==(const weight& _other) const {
    return source==_other.source &&
      target==_other.target &&
      id==_other.id;
  }
};

inline istream& operator >> (istream& s, weight& w) {
  s >> w.source >> w.target>> w.id;
  return s;
}

inline ostream& operator << (ostream& s, const weight& w) {
  return s << w.source <<" "<< w.target <<" "<< w.id<<" ";
}

size_t N;

/**
 * populate the graph with random edges
 */
template <class G, class W>
class AddEdges{
public:
  typedef typename G::vertex_descriptor VD;
  void add(G& g,
           vector<VD>& vertices,
           vector<pair<VD,VD> >& edges,
           vector<pair<VD,size_t> >& edgeids,
           size_t NE){
    if(!g.is_directed() && N==1) return;
    for(size_t i=0;i < NE; ++i) {
      edges[i] = pair<VD,VD>(vertices[rand() % N], vertices[rand() % N]);
      if(!g.is_directed()){
        //repeat until source != dest
        while(edges[i].first == edges[i].second) edges[i] = pair<VD,VD>(vertices[rand() % N], vertices[rand() % N]);

        if(edges[i].first > edges[i].second) {
          size_t temp = edges[i].first;
          edges[i].first = edges[i].second;
          edges[i].second = temp;
        }
      }
      W wt(edges[i].first, edges[i].second);
      typename G::edge_descriptor ed = g.add_edge(edges[i].first, edges[i].second, wt);
      assert(is_valid(ed));
      edgeids[i] = pair<VD,size_t>(edges[i].first,ed.id());
    }
  }

  static bool test_property(typename G::edge_descriptor& ed, typename G::adj_edge_iterator aei){
    //ed.display();
    //std::cout<<(*aei).source()<<" "<<(*aei).target()<<" "<<(*aei).property().source<<" "<<(*aei).property().target<<"\n";
    if (ed.source() == (*aei).source() &&
        ed.target() == (*aei).target() &&
        ed.source() == (*aei).property().source &&
        ed.target() == (*aei).property().target) return true;
    else
      return false;
  }

};//end class

template <class G>
class AddEdges<G, properties::no_property>{
  typedef typename G::vertex_descriptor VD;
public:
  void add(G& g,
           vector<VD>& vertices,
           vector<pair<VD,VD> >& edges,
           vector<pair<VD,size_t> >& edgeids,
           size_t NE){
    if(!g.is_directed() && N==1) return;
    for(size_t i=0;i < NE; ++i) {
      edges[i] = pair<VD,VD>(vertices[rand() % N], vertices[rand() % N]);
      if(!g.is_directed()){
        //repeat until source != dest
        while(edges[i].first == edges[i].second) edges[i] = pair<VD,VD>(vertices[rand() % N], vertices[rand() % N]);
      }
      typename G::edge_descriptor ed = g.add_edge(edges[i].first, edges[i].second);
      assert(is_valid(ed));
      edgeids[i] = pair<VD,size_t>(edges[i].first,ed.id());
    }
  }

  static bool test_property(typename G::edge_descriptor& ed, typename G::adj_edge_iterator aei){
    if (ed.source() == (*aei).source() && ed.target() == (*aei).target() ) return true;
    else return false;
  }

};

template <class G, class Property>
class AddVerts{
public:
  void add(G& g, vector<typename G::vertex_descriptor>& vertices, size_t N){
    for(size_t i=0;i<N;++i){
      Property p(i);
      vertices[i] = g.add_vertex(p);
    }
  }
  static bool test_property(typename G::vertex_iterator vi){
    if((size_t)(*vi).property() != (*vi).descriptor()) return false;
    else return true;
  }
};

template <class G>
class AddVerts<G, properties::no_property>{
public:
  void add(G& g,vector<typename G::vertex_descriptor>& vertices, size_t N){
    for(size_t i=0;i<N;++i){vertices[i] = g.add_vertex();}
  }
  static bool test_property(typename G::vertex_iterator){
    return true;
  }
};

//for directed with pred and no properties for vertices
template <class G, class P>
class AddVerts<G,sequential::preds_property<size_t,P> >{
public:
  void add(G& g,vector<typename G::vertex_descriptor>& vertices, size_t N){
    for(size_t i=0;i<N;++i){vertices[i] = g.add_vertex();}
  }
  static bool test_property(typename G::vertex_iterator){
    return true;
  }
};

template <class G>
void display(G& g){
  typename G::vertex_iterator vi;
  for(vi=g.begin();vi!=g.end();++vi){
    std::cout<<"["<<(*vi).descriptor()<<"] :: ";
    //print adjacency list
    typename G::adj_edge_iterator ei = (*vi).begin();
    while (ei != (*vi).end()){
      std::cout<<"["<<(*ei).source()<<"->"<<(*ei).target()<<"]("<<(*ei).id()<<") ";
      ++ei;
    }
    std::cout<<"\n";
  }
  //typename G::edge_iterator gei, gei_end;
  //gei_end = g.edges_end();
  //for(gei=g.edges_begin();gei != gei_end; ++gei){
  //  std::cout<<"["<<(*gei).source()<<"->"<<(*gei).target()<<"] ";
  //}
  //cout<<"\n";
}

template <class Graph>
void display1(Graph& g) {
  typedef typename Graph::adj_edge_iterator AEI;
  typedef typename Graph::vertex_iterator VI;
  cout << endl;
  for (VI vi = g.begin(); vi != g.end(); ++vi) {
    cout << "[" << (*vi).descriptor() << "] :: ";
    for (AEI ei = (*vi).begin(); ei != (*vi).end(); ++ei) {
      cout << "[" << (*ei).source() << "->" << (*ei).target() << "](" << (*ei).property() << ") ";
    }
    cout << endl;
  }
}

template <class Graph>
void display2(Graph& g) {
  typedef typename Graph::adj_edge_iterator AEI;
  typedef typename Graph::vertex_iterator VI;
  cout << endl;
  for (VI vi = g.begin(); vi != g.end(); ++vi) {
    cout << "[" << (*vi).descriptor() << "] :: ";
    for (AEI ei = (*vi).begin(); ei != (*vi).end(); ++ei) {
      cout << "[" << (*ei).source() << "->" << (*ei).target() << "](" << (*ei).property() << ") ";
    }
    cout << endl;
  }
}

template <class G>
void display_preds(G& g){
  typename G::vertex_iterator vi;
  for(vi=g.begin();vi!=g.end();++vi){
    std::cout<<"["<<(*vi).descriptor()<<"] :: ";
    //print adjacency list
    typename G::adj_edge_iterator ei = (*vi).begin();
    while (ei != (*vi).end()){
      std::cout<<"["<<(*ei).source()<<"->"<<(*ei).target()<<"] ";
      ++ei;
    }
    //display preds
    typename G::preds_iterator pei = (*vi).predecessors().begin();
    while (pei != (*vi).predecessors().end()){
      std::cout<<"{"<<*pei<<"->"<<(*vi).descriptor()<<"} ";
      ++pei;
    }
    std::cout<<"\n";
  }
}

template <class G>
void display_sizes(G& g) {
  std::cout << "size of size_t: " << sizeof(size_t) << std::endl;
  std::cout << "size of vertex: " << sizeof(typename G::vertex_descriptor) << std::endl;
  std::cout << "size of edge: "   << sizeof(typename G::edge_descriptor) << std::endl;
  double og_size = g.get_num_vertices()*sizeof(size_t) + g.get_num_edges()*sizeof(size_t)*3;
  double ng_size = g.get_num_vertices()*sizeof(typename G::vertex_descriptor)
                 + g.get_num_edges()*sizeof(typename G::edge_descriptor);
  std::cout << "Old Graph size: " << og_size
            << "\nNew Graph size: " << ng_size
            << "\nRatio of sizes: " << double(ng_size)/double(og_size) << endl;
}

/*
  * Creates a graph out of pred-list and weight-map
  * if a node is not connected, mark the pred as self.
  */
template <class Graph, class Map, class PMap>
void convert_to_graph (PMap& p,
                       Map& weights,
                       Graph& g) {
  typedef typename Graph::vertex_descriptor VD;
  for (size_t i = 0; i < p.size(); ++i)
    g.add_vertex(i);
  size_t j = 0;
  for (typename PMap::iterator it = p.begin();
       it != p.end(); ++it, ++j) {
    if (*it != j) {
      const VD j_vd = VD(j);
      g.add_edge((const VD)(*it), j_vd, weights[j_vd]);
    }
  }
}

//Convert Undirected graph to directed graph:
// the class Graph should be directed to begin with;
// this will remove bi-directional edges in a directed graph.
template <class Graph>
void convert_to_directed(Graph& g) {
  typedef typename Graph::vertex_iterator VI;
  typedef typename Graph::adj_edge_iterator AEI;

  // display1(g);

    std::vector<AEI> edges;
    edges.reserve(g.get_num_edges());
    for(VI vi=g.begin(); vi!=g.end(); ++vi){
      AEI aei = (*vi).begin();
    while (aei != (*vi).end()){
      edges.push_back(aei);
      ++aei;
    }
  }

  //delete all edges
  for(typename std::vector<AEI>::iterator it=edges.begin();it!=edges.end();++it){
    AEI ei = *it;
    g.delete_edge((*ei).descriptor());
  }

  //reinsert edges with source/target reversed
  for(typename std::vector<AEI>::iterator it=edges.begin();it!=edges.end();++it){
    AEI ei = *it;
    if (!(g.is_edge((*ei).source(), (*ei).target())
          || g.is_edge((*ei).target(), (*ei).source())))
      g.add_edge((*ei).source(), (*ei).target(), (*ei).property());
  }
}

}  // end namespace stapl.
#endif
