/* Roadmap
 *
 * This is the main class which contains data and methods for accessing the
 * graph and external data structures about changing roadmaps, such as
 * SpillTrees, local planner cache, etc.
 */

#ifndef ROADMAP_H_
#define ROADMAP_H_

#include "RoadmapGraph.h"
#include "Environment.h"

#ifdef _PARALLEL
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#endif

template <class MPTraits>
class Roadmap {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef RoadmapGraph<CfgType, WeightType> GraphType;
    typedef typename GraphType::vertex_descriptor VID;  

    Roadmap();
    Roadmap(Roadmap<MPTraits>& _rdmp);  
    ~Roadmap();
    
    //Read graph information from roadmap file.
    void ReadRoadmap(string _filename);
    void WriteRoadmap(ostream& _os, Environment* _env, 
        const vector<shared_ptr<Boundary> >& _boundaries = vector<shared_ptr<Boundary> >());

    //Append nodes and edges from one roadmap (_rdmp) into 
    //another roadmap (to_rdmp)
    vector<VID> AppendRoadmap(Roadmap<MPTraits>& _rdmp);

    //access and set the LP cache
    bool IsCached(VID _v1, VID _v2);
    void SetCache(VID _v1, VID _v2, bool _b);

    //access the roadmap graph
    GraphType* GetGraph(){return m_graph;}

  private:
    std::map<std::pair<VID,VID>, bool> m_lpCache; //cache of attempted edges
    GraphType* m_graph; //stapl graph
};

template <class MPTraits>
Roadmap<MPTraits>::
Roadmap() : m_graph(new RoadmapGraph<CfgType, WeightType>()){}

template <class MPTraits>
Roadmap<MPTraits>::Roadmap(Roadmap<MPTraits>& _rdmp) : m_graph(new RoadmapGraph<CfgType, WeightType>()) {
  AppendRoadmap(_rdmp);
}

template <class MPTraits>
Roadmap<MPTraits>::~Roadmap(){
  if( m_graph != NULL ) 
    delete m_graph;
  m_graph = NULL;
}

template <class MPTraits>
void 
Roadmap<MPTraits>::ReadRoadmap(string _filename) {
#ifndef _PARALLEL
  cout << endl << "getting nodes from Read: " << _filename << endl;

  ifstream  ifs(_filename.c_str());
  if(!ifs) {
    cerr << endl << "In ReadRoadmap: can't open file: " << _filename ;
    return;
  }
  string tag; 
  bool moreFile = true;
  size_t count = 0;
  while(moreFile && (ifs >> tag)) {
    count++;
    if(tag.find("GRAPHSTART")) 
      moreFile = false;
  }
  ifs.close();

  if(moreFile) {
    ifstream ifs2(_filename.c_str());
    for(size_t i=0; i<count-1; ++i)
      ifs2 >> tag;
    stapl::sequential::read_graph(*m_graph, ifs2);
    ifs2.close();
  }
  else {
    cerr << endl  << "In ReadRoadmap: didn't read GRAPHSTART tag";
    return;
  }
#endif
}

template<class MPTraits>
void 
Roadmap<MPTraits>::WriteRoadmap(ostream& _os, Environment* _env, 
    const vector<shared_ptr<Boundary> >& _boundaries) {
  _os << "PMPL Roadmap Version 041805";
  _os << endl << "#####PREAMBLESTART#####";
  _os << endl << "../pmpl -f " << _env->GetEnvFileName() << " ";//commandLine;
  _os << " -bbox "; _env->GetBoundary()->Print(_os, ',', ',');
  if(!_boundaries.empty()) {
    typedef vector<shared_ptr<Boundary> >::const_iterator BIT;
    for(BIT bit = _boundaries.begin(); bit!=_boundaries.end(); bit++) {
      _os << " -bbox "; (*bit)->Print(_os, ',', ',');
    }
  }
  _os << endl << "#####PREAMBLESTOP#####";

  _os << endl << "#####ENVFILESTART#####";
  _os << endl << _env->GetEnvFileName();
  _os << endl << "#####ENVFILESTOP#####";
  _os << endl;

  _os << "#####LPSTART#####" << endl << "0" << endl << "#####LPSTOP#####" << endl;
  _os << "#####CDSTART#####" << endl << "0" << endl << "#####CDSTOP#####" << endl;
  _os << "#####DMSTART#####" << endl << "0" << endl << "#####DMSTOP#####" << endl;
  _os << "#####RNGSEEDSTART#####" << endl << "0" << endl << "#####RNGSEEDSTOP#####" << endl;
  _os << endl;

#ifndef _PARALLEL
  stapl::sequential::write_graph(*m_graph, _os);         // writes verts & adj lists
#else
  stapl::write_graph(*m_graph, _os);
#endif
}

template <class MPTraits>
vector<typename Roadmap<MPTraits>::VID>
Roadmap<MPTraits>::AppendRoadmap(Roadmap<MPTraits>& _rdmp) {
  vector<VID> fromVIDs, toVIDs;
  _rdmp.m_graph->GetVerticesVID(fromVIDs); // get vertices
  typename vector<VID>::iterator vit;
  //copy vertices
  for(vit = fromVIDs.begin(); vit < fromVIDs.end(); vit++) {
    CfgType cfg = (*(_rdmp.m_graph->find_vertex(*vit))).property();
    toVIDs.push_back(m_graph->AddVertex(cfg));
  }
  vector<pair<pair<VID,VID>, WeightType> > edges;
  typename vector<pair<pair<VID,VID>, WeightType> >::iterator eit;
  for(vit = fromVIDs.begin(); vit < fromVIDs.end(); vit++) {
    edges.clear();
    //use iterator to traverse the adj edges and then put the data into edges
    typename RoadmapGraph<CfgType, WeightType>::vertex_iterator vi = _rdmp.m_graph->find_vertex(*vit);
    for(typename RoadmapGraph<CfgType, WeightType>::adj_edge_iterator ei =(*vi).begin(); ei!=(*vi).end(); ei++ ){
      pair<pair<VID,VID>, WeightType> singleEdge;
      singleEdge.first.first=(*ei).source();
      singleEdge.first.second=(*ei).target();
      singleEdge.second = (*ei).property();
      edges.push_back(singleEdge); //put the edge into edges
    } 
    for(eit = edges.begin(); eit < edges.end(); eit++) {
      if(!m_graph->IsEdge((*eit).first.first, (*eit).first.second)) { //add an edge if it is not yet in m_graph
        CfgType cfgA = (*(_rdmp.m_graph->find_vertex((*eit).first.first))).property();
        CfgType cfgB = (*(_rdmp.m_graph->find_vertex((*eit).first.second))).property();
        m_graph->AddEdge(cfgA, cfgB, (*eit).second);
      }
    } //endfor eit  
  } //endfor vit
  return toVIDs;
}

template<class MPTraits>
bool 
Roadmap<MPTraits>::IsCached(VID _v1, VID _v2) {
  if(m_lpCache.count(make_pair(min(_v1,_v2), max(_v1,_v2))) > 0)
    return true;
  else
    return false;
}

template<class MPTraits>
void
Roadmap<MPTraits>::SetCache(VID _v1, VID _v2, bool _b) {
  m_lpCache[make_pair(min(_v1,_v2),max(_v1,_v2))] = _b;
}

#endif
