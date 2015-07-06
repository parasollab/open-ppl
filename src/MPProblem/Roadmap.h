#ifndef ROADMAP_H_
#define ROADMAP_H_

#include "RoadmapGraph.h"
#include "Environment/Environment.h"

#ifdef _PARALLEL
#include <containers/graph/algorithms/graph_io.hpp>
#else
#include <containers/sequential/graph/algorithms/graph_input_output.h>
#endif

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Roadmap
/// @brief Approximation of @cfree storing a graph and augmenting data
///        structures for use by motion planning algorithms.
///
/// The Roadmap is essentially the graph $G=(V, E)$ which is used to approximate
/// the planning space by sampling-based motion planning algorithms. \f$V\f$, or
/// the set of vertices, are all of type CfgType and \f$E\f$, or the set of
/// edges, are all of type WeightType.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class Roadmap {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef RoadmapGraph<CfgType, WeightType> GraphType;
    typedef typename GraphType::vertex_descriptor VID;

    Roadmap();
    Roadmap(const Roadmap& _rdmp);
    ~Roadmap();

    //Read graph information from roadmap file.
    void Read(string _filename);
    void Write(const string& _os, Environment* _env);

    //Append nodes and edges from one roadmap (_rdmp) into
    //another roadmap (to_rdmp)
    vector<VID> AppendRoadmap(const Roadmap& _rdmp);

    //access the roadmap graph
    GraphType* GetGraph() {return m_graph;}
    const GraphType* GetGraph() const {return m_graph;}
    void SetGraph(GraphType* _graph) { m_graph = _graph; }

  private:
    GraphType* m_graph;
};

template<class MPTraits>
Roadmap<MPTraits>::
Roadmap() : m_graph(new GraphType()){}

template<class MPTraits>
Roadmap<MPTraits>::
Roadmap(const Roadmap& _rdmp) : m_graph(new GraphType()) {
  AppendRoadmap(_rdmp);
}

template<class MPTraits>
Roadmap<MPTraits>::~Roadmap(){
  delete m_graph;
  m_graph = NULL;
}

template <class MPTraits>
void
Roadmap<MPTraits>::Read(string _filename) {
#ifndef _PARALLEL
  ifstream  ifs(_filename.c_str());
  if(!ifs) {
    cerr << "Warning::Cannot open file " << _filename << " in Roadmap::Read." << endl;
    return;
  }
  string tag;
  bool moreFile = true;
  size_t count = 0;
  while(moreFile && (ifs >> tag)) {
    count++;
    if(tag.find("GRAPHSTART") != string::npos)
      moreFile = false;
  }
  ifs.close();

  if(!moreFile) {
    ifstream ifs2(_filename.c_str());
    for(size_t i=0; i<count-1; ++i)
      ifs2 >> tag;
    stapl::sequential::read_graph(*m_graph, ifs2);
    ifs2.close();
  }
  else {
    cerr << "Warning::Did not read GRAPHSTART tag in Roadmap::Read." << endl;
    return;
  }
#endif
}

template<class MPTraits>
void
Roadmap<MPTraits>::Write(const string& _filename, Environment* _env){

  ofstream ofs(_filename);
  ofs << "#####ENVFILESTART#####" << endl
    << _env->GetEnvFileName() << endl
    << "#####ENVFILESTOP#####" << endl;

#ifndef _PARALLEL
  stapl::sequential::write_graph(*m_graph, ofs);         // writes verts & adj lists
#else
  ofs.close();
  stapl::graph_view<GraphType> gv(*m_graph);
  write_PMPL_graph(gv, _filename);
#endif
}

template <class MPTraits>
vector<typename Roadmap<MPTraits>::VID>
Roadmap<MPTraits>::AppendRoadmap(const Roadmap& _rdmp) {
  vector<VID> toVIDs;
  typename vector<VID>::iterator vit;
  //copy vertices
  typedef typename GraphType::VI VI;
  for(VI vit = _rdmp.m_graph->begin(); vit!=_rdmp.m_graph->end(); ++vit){
    toVIDs.push_back(m_graph->AddVertex(m_graph->GetVertex(vit)));
  }
  vector<pair<pair<VID,VID>, WeightType> > edges;
  typename vector<pair<pair<VID,VID>, WeightType> >::iterator eit;
  for(VI vit = _rdmp.m_graph->begin(); vit!=_rdmp.m_graph->end(); ++vit) {
    edges.clear();
    //use iterator to traverse the adj edges and then put the data into edges
    for(typename RoadmapGraph<CfgType, WeightType>::adj_edge_iterator ei =(*vit).begin(); ei!=(*vit).end(); ei++ ){
      pair<pair<VID,VID>, WeightType> singleEdge;
      singleEdge.first.first=(*ei).source();
      singleEdge.first.second=(*ei).target();
      singleEdge.second = (*ei).property();
      edges.push_back(singleEdge); //put the edge into edges
    }
    for(eit = edges.begin(); eit < edges.end(); eit++) {
      if(!m_graph->IsEdge((*eit).first.first, (*eit).first.second)) { //add an edge if it is not yet in m_graph
        m_graph->AddEdge((*eit).first.first, (*eit).first.second, (*eit).second);
      }
    } //endfor eit
  } //endfor vit
  return toVIDs;
}

#endif
