#ifndef ROADMAP_GRAPH_H_
#define ROADMAP_GRAPH_H_

#ifdef _PARALLEL
#include <containers/graph/dynamic_graph.hpp>
//#include "graph/view/graph_view_property_adaptor.h"
//#include <containers/graph/view/graph_view_property_adaptor.hpp>
#include <containers/sequential/graph/algorithms/connected_components.h>
#else
#include <containers/sequential/graph/graph.h>
#include <containers/sequential/graph/vertex_iterator_adaptor.h>
#include <containers/sequential/graph/algorithms/connected_components.h>
#endif

#include "RoadmapVCS.h"

#ifndef INVALID_VID
#define INVALID_VID (std::numeric_limits<size_t>::max())
#endif

#ifndef INVALID_EID
#define INVALID_EID (std::numeric_limits<size_t>::max())
#endif

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Roadmap
/// @brief Graph data structure.
/// @tparam VERTEX Configuration type
/// @tparam WEIGHT Weight type
////////////////////////////////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT>
class RoadmapGraph : public
#ifdef _PARALLEL
                     stapl::dynamic_graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT>
#elif defined(VIZMO)
                     stapl::sequential::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT,
                     stapl::sequential::adj_map_int<stapl::DIRECTED, stapl::NONMULTIEDGES,VERTEX,WEIGHT> >
#else
                     stapl::sequential::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT>
#endif
{
  public:
#ifdef _PARALLEL
    typedef stapl::dynamic_graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT> GRAPH;
#elif defined(VIZMO)
    typedef stapl::sequential::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT,
            stapl::sequential::adj_map_int<stapl::DIRECTED, stapl::NONMULTIEDGES,VERTEX,WEIGHT> > GRAPH;
#else
    typedef stapl::sequential::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT> GRAPH;
#endif

    typedef typename GRAPH::vertex_descriptor VID;
    typedef typename GRAPH::edge_descriptor EID;
    typedef typename GRAPH::vertex_iterator VI; ///<VI Vertex Iterator
    typedef typename GRAPH::adj_edge_iterator EI;
#ifndef _PARALLEL
    ///TODO: Remove guard after const issue is fixed in STAPL
    typedef typename GRAPH::const_vertex_iterator CVI;
    typedef typename GRAPH::vertex_property& VP;
    typedef stapl::sequential::vdata_iterator<VI> VPI;
    typedef stapl::sequential::const_vdata_iterator<VI> CVPI;
    typedef stapl::sequential::vector_property_map<GRAPH, size_t> ColorMap;
#else
    typedef typename GRAPH::vertex_iterator CVI;
    typedef typename GRAPH::vertex_property VP;
    typedef stapl::sequential::vector_property_map<GRAPH, size_t> ColorMap;
#endif
    typedef RoadmapChangeEvent<GRAPH> ChangeEvent;
    typedef RoadmapVCS<GRAPH> RoadmapVCSType;

    RoadmapGraph() {}

    //////////////////////////////////
    // Adding & accessing vertices
    //////////////////////////////////
    //Add a new unique vertex to the graph.
    //If it already exists in the graph an error message will be output.
    //Returns a new VID of the added vertex or the VID of the existing vertex.
    VID AddVertex(const VERTEX& _v);

    bool IsVertex(const VERTEX& _v);
    bool IsVertex(const VERTEX& _v, CVI& _vi);

    //helper function to call dereferece on an iterator whose
    //value_type is VID and convert to CfgType
    template<typename T>
      VP GetVertex(T& _t);

    //specialization for a roadmap graph iterator, calls property()
    VP GetVertex(VI& _t);

    //specialization for a RoadmapGraph<CFG, WEIGHT>::VID
    //calls find_vertex(..) on VID to call property()
    //TODO:Temporarily removed constness until it is supported in STAPL
    VP GetVertex(VID _t) ;

    //helper function to call dereferece on an iterator whose value_type is VID
    //needed to get around the fact that a roadmap graph iterator
    //requires an extra descriptor() call
    template<typename T>
      VID GetVID(const T& _t);

    //specialization for a roadmap graph iterator, calls descriptor()
    VID GetVID(const VI& _t);

    //specialization for a Vertex type
    VID GetVID(const VERTEX& _t);

    //////////////////////////////////
    // Adding & accessing edges
    //////////////////////////////////
    void AddEdge(VID _v1, VID _v2, const WEIGHT& _w);
    void AddEdge(VID _v1, VID _v2, const pair<WEIGHT,WEIGHT>& _w);

    bool IsEdge(VID _v1, VID _v2);
    bool IsEdge(VID _v1, VID _v2, EI& _ei);

    //////////////////////////////////
    // CC Operations
    //////////////////////////////////
#ifndef _PARALLEL
    size_t GetNumCCs();
#endif

    ///Temporarily wrapper for some graph methods
    ///Until full migration and change of names in STAPL is completed
#ifdef _PARALLEL
    size_t get_num_edges() { return this->num_edges();}
    size_t get_num_vertices() const { return this->num_vertices();}
    size_t get_degree(const VID& _vd) {
      VI vi = this->find_vertex(_vd);
      return (*vi).size();
    }
    size_t get_out_degree(const VID& _vd){
      return this->get_degree(_vd);
    }
#endif

    const RoadmapVCSType& GetRoadmapVCS() const {return m_roadmapVCS;}

  private:
    RoadmapVCSType m_roadmapVCS;
};

//////////////////////////////////
// Adding & accessing vertices
//////////////////////////////////

template<class VERTEX, class WEIGHT>
typename RoadmapGraph<VERTEX, WEIGHT>::VID
RoadmapGraph<VERTEX,WEIGHT>::AddVertex(const VERTEX& _v) {
#ifndef _PARALLEL
  CVI vi;
  if(IsVertex(_v, vi)){
    cerr << "\nIn AddVertex: vertex already in graph" << endl;
    return vi->descriptor();
  }
  VID vid = this->add_vertex(_v);
  VDAddNode(_v);
  m_roadmapVCS.AddEvent(ChangeEvent(_v, vid));
  return vid;
#else
  return this->add_vertex(_v);
#endif
};

template<class VERTEX, class WEIGHT>
bool
RoadmapGraph<VERTEX,WEIGHT>::IsVertex(const VERTEX& _v){
  CVI vi;
  return IsVertex(_v, vi);
}

template<class VERTEX, class WEIGHT>
bool
RoadmapGraph<VERTEX,WEIGHT>::IsVertex(const VERTEX& _v, CVI& _vi)  {
#ifndef _PARALLEL
  for(CVI vi = this->begin(); vi != this->end(); ++vi){
    if(vi->property() == _v){
      _vi = vi;
      return true;
    }
  }
#else
  cerr << "WARNING::STAPL working on fixing problem with const iterators";
#endif
  return false;
}

//helper function to call dereferece on an iterator whose
//value_type is VID and convert to CfgType
template<class VERTEX, class WEIGHT>
template<typename T>
typename RoadmapGraph<VERTEX, WEIGHT>::VP
RoadmapGraph<VERTEX, WEIGHT>::GetVertex(T& _t) {
  return (*(this->find_vertex(*_t))).property();
}

//specialization for a roadmap graph iterator, calls property()
template<class VERTEX, class WEIGHT>
typename RoadmapGraph<VERTEX, WEIGHT>::VP
RoadmapGraph<VERTEX, WEIGHT>::GetVertex(VI& _t) {
  return (*_t).property();
}

//specialization for a RoadmapGraph<CFG, WEIGHT>::VID
//calls find_vertex(..) on VID to call property()
template<class VERTEX, class WEIGHT>
typename RoadmapGraph<VERTEX, WEIGHT>::VP
RoadmapGraph<VERTEX, WEIGHT>::GetVertex(VID _t)  {
  return (*this->find_vertex(_t)).property();
}

//helper function to call dereferece on an iterator whose value_type is VID
//needed to get around the fact that a roadmap graph iterator
//requires and extra descriptor() call
template<class VERTEX, class WEIGHT>
template<typename T>
typename RoadmapGraph<VERTEX, WEIGHT>::VID
RoadmapGraph<VERTEX, WEIGHT>::GetVID(const T& _t) {
  return *_t;
}

//specialization for a roadmap graph iterator, calls descriptor()
template<class VERTEX, class WEIGHT>
typename RoadmapGraph<VERTEX, WEIGHT>::VID
RoadmapGraph<VERTEX, WEIGHT>::GetVID(const VI& _t) {
  return (*_t).descriptor();
}

//specialization for a Vertex type
template<class VERTEX, class WEIGHT>
typename RoadmapGraph<VERTEX, WEIGHT>::VID
RoadmapGraph<VERTEX,WEIGHT>::GetVID(const VERTEX& _t) {
  CVI vi;
  if(IsVertex(_t, vi))
    return (*vi).descriptor();
  return INVALID_VID;
}

//////////////////////////////////
// Adding & accessing edges
//////////////////////////////////

template <class VERTEX, class WEIGHT>
void
RoadmapGraph<VERTEX,WEIGHT>::AddEdge(VID _v1, VID _v2, const WEIGHT& _w) {
#ifdef VIZMO
  GetVertex(_v1).Lock();
#endif
  this->add_edge(_v1, _v2, _w);
  VDAddEdge(GetVertex(_v1), GetVertex(_v2));
#ifdef VIZMO
  GetVertex(_v1).UnLock();
#endif
}

template <class VERTEX, class WEIGHT>
void
RoadmapGraph<VERTEX,WEIGHT>::AddEdge(VID _v1, VID _v2, const pair<WEIGHT,WEIGHT>& _w) {
#ifdef VIZMO
  GetVertex(_v1).Lock();
  GetVertex(_v2).Lock();
#endif
  this->add_edge(_v1, _v2, _w.first);
  this->add_edge(_v2, _v1, _w.second);
  VDAddEdge(GetVertex(_v1), GetVertex(_v2));
#ifdef VIZMO
  GetVertex(_v2).UnLock();
  GetVertex(_v1).UnLock();
#endif
}

template<class VERTEX, class WEIGHT>
bool
RoadmapGraph<VERTEX,WEIGHT>::IsEdge(VID _v1, VID _v2) {
  EI ei;
  return IsEdge(_v1, _v2, ei);
}

template<class VERTEX, class WEIGHT>
bool
RoadmapGraph<VERTEX,WEIGHT>::
IsEdge(VID _v1, VID _v2, EI& _ei) {
#ifndef _PARALLEL
  VI vi;
  return this->find_edge(EID(_v1, _v2), vi, _ei);
#else
  return false;
#endif
}

#ifndef _PARALLEL
template<class VERTEX, class WEIGHT>
size_t
RoadmapGraph<VERTEX, WEIGHT>::GetNumCCs() {
  ColorMap c;
  return get_cc_count(*this, c);
}
#endif

#endif
