// $Id$
/////////////////////////////////////////////////////////////////////
/**@file RoadmapGraph.h
  *
  * General Description
  *    This set of template classes provides an implementation for 
  *    graphs to be used as motion planning roadmaps.
  *
  *    The user must provide:
  *    -- the parameter type VERTEX, which is the data type 
  *       (e.g., configurations stored in each vertex of the graph.
  *    -- the parameter type WEIGHT, which is the data type stored in 
  *       each edge of the graph (e.g., local planner identification).
  *
  *    Each graph vertex is assigned a unique vertex identifier (VID). 
  *    The VID is typedef'ed as a short int (in Graph.h), which should 
  *    be sufficient for roadmaps (this limits the number of vertices 
  *    in the roadmap to the maximum value expressed in a short int).
  *
  * @date 7/18/98
  * @author Nancy Amato
  */
/////////////////////////////////////////////////////////////////////

#ifndef RoadmapGraph_h
#define RoadmapGraph_h

////////////////////////////////////////////////////////////////////////////////////////////
///OBPRM Headers
#include "Graph.h"

////////////////////////////////////////////////////////////////////////////////////////////
#define DEFAULT_EDGES_PER_VERTEX 10  ///< Slots to reserve in each edgelist


////////////////////////////////////////////////////////////////////////////////////////////
template<class WEIGHT>
class EdgeInfo {
public:
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{
       ///Default Constructor. Do nothing.
       EdgeInfo() {}

       /**Constructor, init data members.
         *@param _v1 The ID for the first endpoint of this edge.
         *@param _v2 The ID for the second endpoint of this edge.
         *@param _wts _wts.first is weight for _v1->_v2 and _wts.second is 
         *weight for _v2->_v1
         */
       EdgeInfo(VID _v1, VID _v2, pair<WEIGHT,WEIGHT> _wts) : v1(_v1), v2(_v2), edgewts(_wts) {}
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

   VID v1;  ///<The ID for the first endpoint of this edge.
   VID v2;  ///<The ID for the second endpoint of this edge.
   ///_wts.first is weight for _v1->_v2 and _wts.second is *weight for _v2->_v1
   pair<WEIGHT,WEIGHT> edgewts;

};


/////////////////////////////////////////////////////////////////////
//
//
//
//
//
//          RoadmapGraph<VERTEX,WEIGHT> Definition
//
//
//
//
//
/////////////////////////////////////////////////////////////////////


/**Derived from WeightedGraph<VERTEX,WEIGHT> (defined in Graph.h).
  *
  *The RoadmapGraph class:
  * -# is an *undirected* weighted graph 
  * -# does not allow multiple vertices with the same VERTEX data 
  *    (which is the only difference from WeightedGraph). 
  * -# does not allow multiple (v1,v2) edges 
  *
  *@date7/25/98  
  *@author Nancy Amato
  */
template<class VERTEX, class WEIGHT>
class RoadmapGraph : public WeightedGraph<VERTEX,WEIGHT> {

public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{

       /**Constrcutor. Do nothing.
         *@note space reservation done by Init()
         */
       RoadmapGraph();

       /**Constrcutor, Reserve space.
         *@param _sz How many vertices are going to be reserved.
         *This method uses DEFAULT_EDGES_PER_VERTEX to reserve
         *spaces for edges for each vertex.
         *@see WeightedGraph::WeightedGraph(int,int) 
         */
       RoadmapGraph(int _sz);  // 'reserve' space for verts (default for edges)

       /**Constrcutor, Reserve space.
         *@param _sz How many vertices are going to be reserved.
         *@param _edgelistsz How many edges are going to be reserved 
         *for each vertex.
         *@see WeightedGraph::WeightedGraph(int,int) 
         */
       RoadmapGraph(int _sz, int _edgelistsz); // 'reserve' space for verts & edges

       ///Destrucotr Do nothing.
       ~RoadmapGraph();
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Helpers
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Helper Methods*/
    //@{

       /**Initialize roadmap (reserve space).
         *@param _numNodes How many vertices are going to be reserved in this graph
         *@param _numEdges How many edges are going to be reserved for each vertex
         */
       void Init(const int _numNodes, const int _numEdges);

    //@}


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Adding & Deleting Vertices & Edges
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Adding & Deleting Vertices & Edges*/
    //@{

       /**Add a new (isolated) vertex to graph.
         *If there is same user data (VERTEX&) added to
         *this graph before, error message will be post on standard output.
         *@return New VID if there is a new vertex was created, otherwise
         *existing vertex's VID will be returned.
         *@see WeightedMultiDiGraph::AddVertex(VERTEX&)
         */
       virtual VID  AddVertex(VERTEX&); 

       /**Add a list of new (isolated) vertices to graph.
         *
         *If there is any user data (VERTEX&) in the list added to
         *this graph before, error message will be post on standard output.
         *
         *@return New VID of first created vertex if there is any new vertex 
         *was created, otherwise (no new vertex) INVALID_VID will be returned.
         *@see WeightedMultiDiGraph::AddVertex(VERTEX&)
         */
       virtual VID  AddVertex(vector<VERTEX>&);

       /**Add edges by a list of EdgeInfo<WEIGHT>.
         *@return always return OK.
         *@see WeightedGraph::AddEdge(VID, VID, pair<WEIGHT,WEIGHT>)
         */
       virtual int  AddEdges( vector<EdgeInfo<WEIGHT> >& );

    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

   typedef WtVertexType<VERTEX,WEIGHT> Vertex;
   typedef vector< Vertex > VERTEX_VECTOR;
   typedef typename VERTEX_VECTOR::iterator VI; ///<VI Vertex Iterator
   typedef typename VERTEX_VECTOR::const_iterator CVI; ///<CVI Constant Vertex Iterator

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:

};

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//   METHODS FOR template RoadmapGraph Class
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

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
