// $Id$
/////////////////////////////////////////////////////////////////////
//  RoadmapGraph.h
//
//  General Description
//      This set of template classes provides an implementation for 
//      graphs to be used as motion planning roadmaps. 
//
//      The user must provide:
//      -- the parameter type VERTEX, which is the data type 
//         (e.g., configurationsstored in each vertex of the graph.
//      -- the parameter type WEIGHT, which is the data type stored in 
//         each edge of the graph (e.g., local planner identification).
//
//      Each graph vertex is assigned a unique vertex identifier (VID). 
//      The VID is typedef'ed as a short int (in Graph.h), which should 
//      be sufficient for roadmaps (this limits the number of vertices 
//      in the roadmap to the maximum value expressed in a short int).
//
//  Created
//      7/18/98  Nancy Amato
/////////////////////////////////////////////////////////////////////
#ifndef RoadmapGraph_h
#define RoadmapGraph_h

#include "Graph.h"

#define DEFAULT_EDGES_PER_VERTEX 10  //slots to reserve in each edgelist

template<class WEIGHT>
class EdgeInfo {
public:
   EdgeInfo() {}
   EdgeInfo(VID _v1, VID _v2, pair<WEIGHT,WEIGHT> _wts) : v1(_v1), v2(_v2), edgewts(_wts) {}
   VID v1;
   VID v2;
   pair<WEIGHT,WEIGHT> edgewts;
};

/////////////////////////////////////////////////////////////////////
//  class RoadmapGraph<VERTEX,WEIGHT>
//
//  General Description
//      Derived from WeightedGraph<VERTEX,WEIGHT> (defined in Graph.h).
//
//      The RoadmapGraph class:
//      -- is an *undirected* weighted graph 
//      -- does not allow multiple vertices with the same VERTEX data 
//         (which is the only difference from WeightedGraph). 
//      -- does not allow multiple (v1,v2) edges 
//
//  Created
//      7/25/98  Nancy Amato
/////////////////////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT>
class RoadmapGraph : public WeightedGraph<VERTEX,WEIGHT> {
public:
  //===================================================================
  //  Constructors and Destructor
  //===================================================================
   RoadmapGraph();         // space reservation done by Init()
   RoadmapGraph(int _sz);  // 'reserve' space for verts (default for edges)
   RoadmapGraph(int _sz, int _edgelistsz); // 'reserve' space for verts & edges
   ~RoadmapGraph();

  //===================================================================
  //  Other Methods
  //===================================================================
        // initialize roadmap (reserve space)
   void Init(const int _numNodes, const int _numEdges);
        // Adding & Deleting Vertices
   virtual VID  AddVertex(VERTEX&); 
   virtual VID  AddVertex(vector<VERTEX>&); 

   virtual int  AddEdges( vector<EdgeInfo<WEIGHT> >& );


protected:
private:
  //===================================================================
  //  Data
  //===================================================================
};

//=====================================================================
//=====================================================================
//  METHODS FOR template RoadmapGraph Class
//=====================================================================
//=====================================================================

  //==================================
  // RoadmapGraph class Methods: Constructors and Destructor
  //==================================
template<class VERTEX, class WEIGHT>
RoadmapGraph<VERTEX,WEIGHT>::
RoadmapGraph() {
};

template<class VERTEX, class WEIGHT>
RoadmapGraph<VERTEX,WEIGHT>::
RoadmapGraph(int _sz) 
 : WeightedGraph<VERTEX,WEIGHT>(_sz,DEFAULT_EDGES_PER_VERTEX)
{
};

template<class VERTEX, class WEIGHT>
RoadmapGraph<VERTEX,WEIGHT>::
RoadmapGraph(int _sz, int _edgelistsz) 
 : WeightedGraph<VERTEX,WEIGHT>(_sz,_edgelistsz)
{
};

template<class VERTEX, class WEIGHT>
RoadmapGraph<VERTEX,WEIGHT>::
~RoadmapGraph() {
};

  //==================================
  // RoadmapGraph class Methods: Initialization
  //==================================

template<class VERTEX, class WEIGHT>
void
RoadmapGraph<VERTEX,WEIGHT>::
Init(const int _numNodes, const int _numEdges) {
    v.reserve(_numNodes);
    reserveEdgesPerVertex = _numEdges;
}

  //==================================
  // RoadmapGraph class Methods: Adding & Deleting Vertices
  //==================================

// require that VERTEX data (configuration) is unique
template<class VERTEX, class WEIGHT>
VID
RoadmapGraph<VERTEX,WEIGHT>::
AddVertex(VERTEX& _v1) {
    CVI v1;
    if ( !IsVertex(_v1,&v1) ) {
      return WeightedGraph<VERTEX,WEIGHT>::AddVertex(_v1);
    } else {
#ifndef QUIETGRAPH
      cout << "\nIn AddVertex: vertex already in graph, not added";
#endif
      return (v1->vid); // return vertex id 
    }
};

// require that VERTEX data (configuration) is unique
template<class VERTEX, class WEIGHT>
VID
RoadmapGraph<VERTEX,WEIGHT>::
AddVertex(vector<VERTEX>& _v) {
    bool added=false;
    VID vertex_id=GetNextVID();
    for (int i = 0; i < _v.size(); i++){
        if (!IsVertex(_v[i])){
           WeightedGraph<VERTEX,WEIGHT>::AddVertex(_v[i]);
           added=true;
        } else {
#ifndef QUIETGRAPH
      cout << "\nIn AddVertex: vertex already in graph, not added";
#endif
        }
    }
    if (added)  return vertex_id;
    else        return INVALID_VID;
}

template<class VERTEX, class WEIGHT>
int
RoadmapGraph<VERTEX,WEIGHT>::
AddEdges( vector<EdgeInfo<WEIGHT> >& _e) {
  for (int i=0; i < _e.size(); i++){
    WeightedGraph<VERTEX,WEIGHT>::AddEdge(_e[i].v1, _e[i].v2, _e[i].edgewts);
  }
  return OK;
}

#endif
