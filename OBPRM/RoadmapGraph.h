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
//  Last Modified By:
//      xx/xx/xx  <Name>
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
//  Last Modified By:
//      xx/xx/xx  <Name>
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

/*---------------------------------------------------------------------
   int  DeleteVertex(VID);           // Methods below this line are
   int  DeleteVertex(VERTEX&);       // from the base class WeightedGraph.
   int  EraseGraph();                // Defs repeated for your convenience.

        // Adding & Deleting Edges
   int  AddEdge(VID, VID, WEIGHT);
   int  AddEdge(VID, VID, pair<WEIGHT,WEIGHT>);
   int  AddPath( vector<VID>&, WEIGHT);
   int  AddPath( vector< pair<VID,WEIGHT> >& );
   int  AddPath( vector< pair<VID, pair<WEIGHT,WEIGHT> > >& );
   int  DeleteEdge(VID, VID);
   int  ChangeEdgeWeight(VID, VID, WEIGHT); 
   void DeleteAllEdges(VID);

   int  AddEdge(VERTEX&, VERTEX&, WEIGHT);
   int  AddEdge(VERTEX&, VERTEX&, pair<WEIGHT,WEIGHT>);
   int  AddPath( vector<VERTEX>&, WEIGHT);
   int  AddPath( vector< pair<VERTEX,WEIGHT> >& );
   int  AddPath( vector< pair<VERTEX, pair<WEIGHT,WEIGHT> > >& );
   int  DeleteEdge(VERTEX&, VERTEX&);
   int  ChangeEdgeWeight(VERTEX&, VERTEX&, WEIGHT);
   void DeleteAllEdges(VERTEX&);

        // Finding Vertices & Edges
   bool IsVertex(VID) const;
   bool IsEdge(VID, VID) const;
   bool IsEdge(VID, VID, WEIGHT) const;

   bool IsVertex(VERTEX&) const;
   bool IsEdge(VERTEX&, VERTEX&) const;
   bool IsEdge(VERTEX&, VERTEX&, WEIGHT) const;

       // Data & Statistics -- Num Verts, Num Edges, Etc
                // global information
   int  GetVertexCount() const;
   int  GetEdgeCount() const;
   VID  GetNextVID() const;
   vector<VID> GetVerticesVID() const;
   vector<VID> GetVerticesVID(VID,int) const;     // get int verts, starting with VID
   vector<VERTEX> GetVerticesData() const;
   vector<VERTEX> GetVerticesData(VID,int) const; // get int verts, starting with VID
   vector< pair<pair<VID,VID>,WEIGHT> > GetEdges() const;
   vector< pair<pair<VERTEX,VERTEX>, WEIGHT> > GetEdgesVData() const;
                // vertex information
   VERTEX  GetData(VID) const;
   vector<VERTEX> GetData(VID _v1id, VID _v2id) const;
   VID     GetVID(VERTEX&) const;
   WEIGHT  GetEdgeWeight(VID, VID) const;
   WEIGHT  GetEdgeWeight(VERTEX&, VERTEX&) const;
   int     GetVertexDegree(VID) const;
   vector<VID> GetAdjacentVertices(VID) const;
   vector< pair<pair<VID,VID>,WEIGHT> > GetIncidentEdges(VID) const;

       // Basic Graph Algorithms
   WeightedMultiDiGraph<VERTEX,WEIGHT> BFS(VID) const; 
   WeightedMultiDiGraph<VERTEX,WEIGHT> BFS(VERTEX&) const; 
   vector< pair<VERTEX,WEIGHT> > FindPathBFS(VID,VID) const;
   vector< pair<VERTEX,WEIGHT> > FindPathBFS(VERTEX&,VERTEX&) const;
   WeightedMultiDiGraph<VERTEX,WEIGHT> DijkstraSSSP(VID) const; //wts=pathlength
   vector< pair<VERTEX,WEIGHT> >  FindPathDijkstra(VID,VID) const; //wts=ewts
   vector< pair<VERTEX,WEIGHT> >  FindPathDijkstra(VERTEX&,VERTEX&) const;

       // Connected Components Utilities
   bool IsSameCC(VID,VID) const;
   bool IsSameCC(VERTEX&,VERTEX&) const;
   vector<VID> GetCC(VID) const;
   vector<VERTEX> GetCC(VERTEX&) const;
   vector< pair<pair<VID,VID>, WEIGHT> > GetCCEdges(VID) const;
   vector< pair<pair<VID,VID>, WEIGHT> > GetCCEdges(VERTEX&) const;
   vector< pair<pair<VERTEX,VERTEX>, WEIGHT> > GetCCEdgesVData(VID) const;
   vector< pair<pair<VERTEX,VERTEX>, WEIGHT> > GetCCEdgesVData(VERTEX&) const;
   vector< vector< pair<VERTEX,VERTEX> > GetEdgesByCCVDataOnly(); //[i,j]=CC i,edge j
   vector< pair<int,VID> > GetCCStats() const;   // (CCsize,VID in CC) pairs
   int GetCCcount() const;

        // Display, Input, Output 
   void DisplayGraph() const;
   void DisplayVertices() const;
   void DisplayVertex(VID) const;
   void DisplayVertexAndEdgelist(VID) const;
   void DisplayCC(VID) const;
   void DisplayCCStats(int _numCCtoPrint = -1) const; // default print all 

   void WriteGraph(ostream& _myostream) const;
   void WriteGraph(const char*  _filename) const;
   void ReadGraph(istream& _myistream);
   void ReadGraph(const char*  _filename);
---------------------------------------------------------------------*/

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
  // void Init(const int _numNodes, const int _numEdges);

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
  // VID AddVertex(VERTEX&);
  // VID AddVertex(vector<VERTEX>&);

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
