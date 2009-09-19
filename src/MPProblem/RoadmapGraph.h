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
#include "GraphAlgo.h"

////////////////////////////////////////////////////////////////////////////////////////////
#define DEFAULT_EDGES_PER_VERTEX 10  ///< Slots to reserve in each edgelist

using stapl::dkinfo;
using stapl::dkinfo_compare;
using stapl::VID;

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
         *@param _wt The weight for _v1->_v2 
         */
       EdgeInfo(VID _v1, VID _v2, WEIGHT _wt) : v1(_v1), v2(_v2), edgewt(_wt) {}
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
   ///_wt The weight for _v1->_v2 
   WEIGHT edgewt;

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
  * -# is a *directed* weighted graph 
  * -# does not allow multiple vertices with the same VERTEX data 
  *    (which is the only difference from WeightedGraph). 
  * -# does not allow multiple (v1,v2) edges 
  *
  *@date7/25/98  
  *@author Nancy Amato
  */



template<class VERTEX, class WEIGHT>
class RoadmapGraph : 
public stapl::Graph<stapl::DIRECTED,stapl::NONMULTIEDGES,stapl::WEIGHTED, VERTEX,WEIGHT> {

public:

typedef stapl::Graph<stapl::DIRECTED, stapl::NONMULTIEDGES, stapl::WEIGHTED, VERTEX,WEIGHT> GRAPH;

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
       virtual void Init(const int _numNodes, const int _numEdges);

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
       
       virtual int  AddEdge(VID, VID, WEIGHT);
       virtual int  AddEdge(VERTEX&, VERTEX&, WEIGHT); 
       virtual int  AddEdge(VID, VID, pair<WEIGHT,WEIGHT>&);
       virtual int  AddEdge(VERTEX&, VERTEX&, pair<WEIGHT,WEIGHT>&);
       virtual int  AddEdge(VID, VID, pair<WEIGHT*,WEIGHT*>&);
       virtual int  AddEdge(VERTEX&, VERTEX&, pair<WEIGHT*,WEIGHT*>&);

    //@}

    /**@name Adding Vertices & Edges from other RoadmapGraph's*/
    //@{
       /**Copy vertices and all incident edges associated with "vids" 
	*from one roadmap to another.
	*@param toMap Target, Cfgs in vids and incident edges in fromMap  
	*will be copied to this submap.
	*@param fromMap Source, edge information will be retrived from here.
	*@param vids Source, vertex information will be retrived from here.
	*Usually, in this list, elements are Cfgs in same connected component.
	*/
       vector<VID> MergeRoadMap(RoadmapGraph<VERTEX,WEIGHT>*, 
				 vector<VID> vids);
     //@}
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  //protected:

   typedef stapl::WtVertexType<VERTEX,WEIGHT> Vertex;
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
 : GRAPH(_sz,DEFAULT_EDGES_PER_VERTEX)
{
};

template<class VERTEX, class WEIGHT>
RoadmapGraph<VERTEX,WEIGHT>::
RoadmapGraph(int _sz, int _edgelistsz) 
 : GRAPH(_sz,_edgelistsz)
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
    this->v.reserve(_numNodes);
    this->reserveEdgesPerVertex = _numEdges;
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
        return GRAPH::AddVertex(_v1);
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
    VID vertex_id=this->GetNextVID();
    for (int i = 0; i < _v.size(); i++){
        if (!IsVertex(_v[i])){
	  GRAPH::AddVertex(_v[i]);
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
        GRAPH::AddEdge(_e[i].v1, _e[i].v2, _e[i].edgewt);
    }
    return OK;
}

template <class VERTEX, class WEIGHT>
int  
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1, VID _v2, WEIGHT _w) {
  return GRAPH::AddEdge(_v1,_v2,_w);
}

template <class VERTEX, class WEIGHT>
int  
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _w) {
  return GRAPH::AddEdge(_v1,_v2,_w);
}

template <class VERTEX, class WEIGHT>
int  
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1, VID _v2, pair<WEIGHT,WEIGHT>& _w) {
  GRAPH::AddEdge(_v1,_v2,_w.first);
  return GRAPH::AddEdge(_v2,_v1,_w.second);
}

template <class VERTEX, class WEIGHT>
int  
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, pair<WEIGHT,WEIGHT>& _w) {
  GRAPH::AddEdge(_v1,_v2,_w.first);
  return GRAPH::AddEdge(_v2,_v1,_w.second);
}

template <class VERTEX, class WEIGHT>
int
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1, VID _v2, pair<WEIGHT*,WEIGHT*>& _w) {
  pair<WEIGHT,WEIGHT> tmp;
  tmp.first = *_w.first;
  tmp.second = *_w.second;
  return AddEdge(_v1,_v2,tmp);
}

template <class VERTEX, class WEIGHT>
int
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, pair<WEIGHT*,WEIGHT*>& _w) {
  pair<WEIGHT,WEIGHT> tmp;
  tmp.first = *_w.first;
  tmp.second = *_w.second;
  return AddEdge(_v1,_v2,tmp);
}

template <class VERTEX, class WEIGHT>
vector<VID> 
RoadmapGraph<VERTEX,WEIGHT>::
MergeRoadMap(RoadmapGraph<VERTEX, WEIGHT>* _fromMap, 
	      vector<VID> vids) {
  VERTEX t;
  int i;
  vector<VID> newVids;

  //Add vertex
  for (i=0;i<vids.size();++i) {
    t=_fromMap->GetData(vids[i]);
    
    newVids.push_back(this->AddVertex(t));
  } //endfor i


  //-- get edges from _rm connected component and add to submap
  for (i=0;i<vids.size();++i) {
    vector< pair<pair<VID,VID>,WEIGHT> > edges; 
    _fromMap->GetOutgoingEdges(vids[i], edges);
    
    for (int j=0;j<edges.size();++j) {
      VERTEX t1=_fromMap->GetData(edges[j].first.first);
      VERTEX t2=_fromMap->GetData(edges[j].first.second);
      
      this->AddEdge(t1,t2, edges[j].second);
    } //endfor j
    
  } //endfor i


  return newVids;


}




//////////////FROM MARCO//////////////////
template<class GRAPH>
double DijkstraSSSP(GRAPH &g,VID start_vid, VID *far_vid)
{
  typename GRAPH::WEIGHT_TYPE _w_;
  typename GRAPH::CVI cv1;
  typename GRAPH::VI v1;

  vector<VID> vec_cc;
  vector<dkinfo> pq; //The priority Queue
  double  maxdist = g.GetVertexCount() * _w_.MaxWeight();
  double max_shortest_path_length=0;
  *far_vid = start_vid;

  GetCC(g, start_vid, vec_cc);
  vector<VID>::iterator vid_itr;
  for ( vid_itr = vec_cc.begin(); vid_itr != vec_cc.end(); vid_itr++) 
  {
    if (*vid_itr == start_vid) 
    {
      pq.push_back( dkinfo(true,*vid_itr,*vid_itr,0) );
    } else {
      pq.push_back( dkinfo(false,*vid_itr,-1,-1000) );
    }
  }

  sort( pq.begin(), pq.end(),  (dkinfo_compare<dkinfo>) );

  while ( pq.size() != 0  && (pq.back().valid()) )
  {
    bool relax = false;
    dkinfo u = pq.back();

    if (u.dist >max_shortest_path_length) { //return value
      max_shortest_path_length = u.dist;
      *far_vid = u.vid;
    }

    pq.pop_back();
    vector<VID> adj;
    g.GetDijkstraInfo(u.vid,adj);

    // check all u's successors
    for (int i = 0; i < adj.size(); i++)
    {
      for (int j=0; j < pq.size(); j++)
      {
        if (adj[i] == pq[j].vid)
        {
          double wt = g.GetEdgeWeight(u.vid,pq[j].vid).Weight();
          if(!pq[j].valid())
          {
            relax = true;
            pq[j].dist = u.dist + wt;
            pq[j].predvid = u.vid;
            pq[j].VertexValid=true;
          }
          else if ( pq[j].dist > u.dist + wt )
          {
            relax = true;
            pq[j].dist = u.dist + wt;
            pq[j].predvid = u.vid;
            pq[j].VertexValid=true;
          }
        }
      } // endfor
    } // endfor

    if (relax) sort( pq.begin(), pq.end(),dkinfo_compare<dkinfo>);
  }
  return max_shortest_path_length;
}

////////////////END FROM MARCO///////////////



static bool dkinfo_ptr_compare (const dkinfo* d1, const dkinfo* d2)
{
  //d1.print();
  //d2.print();
  if ((!d1->valid())&&(!d2->valid()))
  {
    //cout<<"Both d1 and d2 are not valid"<<endl;
    //The returned value does not make much sense
    return ( d1->dist > d2->dist );
  }
  else if(!d1->valid())
  {
    //cout<<"D1 not valid"<<endl;
    //So d1.dist > d2.dist
    //return true;
    return true;
  }
  else if(!d2->valid())
  {
    //cout<<"D2 is not valid"<<endl;
    //return false;
    return false;
  }
  else
  {
    //cout<<"Both are valid"<<endl;   
    return d1->dist > d2->dist;
  }
  
}



template<class GRAPH>
double ComponentDiameter(GRAPH &g,VID start_vid, VID *far_vid)
{
  typename GRAPH::WEIGHT_TYPE _w_;
  typename GRAPH::CVI cv1;
  typename GRAPH::VI v1;

  vector<VID> vec_cc;
  double  maxdist = g.GetVertexCount() * _w_.MaxWeight();
  double max_shortest_path_length=0;
  *far_vid = start_vid;

  GetCC(g, start_vid, vec_cc);
  int CC_size = vec_cc.size();
  vector<dkinfo> pq_dk(CC_size); //The priority Queue -- Data only
  vector<dkinfo*> pq_ptr; //The REAL priority Queue -- heap
  pq_ptr.reserve(CC_size);
  map<VID,dkinfo*> vid_dk_map;
  for(int i=0; i<CC_size; ++i) {
    if (vec_cc[i] == start_vid) 
    {
      pq_dk[i] = dkinfo(true,start_vid,start_vid,0);
      pq_ptr.push_back(&(pq_dk[i]));
    } else {
      pq_dk[i] = dkinfo(false,vec_cc[i],-1,maxdist);
    }
    //pq_ptr[i] = &(pq_dk[i]);
    vid_dk_map[vec_cc[i]] = &(pq_dk[i]);
  }

  //make_heap( pq_ptr.begin(), pq_ptr.end(), dkinfo_ptr_compare);
  //pop_heap(pq_ptr.begin(),pq_ptr.end(),dkinfo_ptr_compare); //

  while ( pq_ptr.size() != 0  && (pq_ptr.back()->valid()) )
  {
    bool relax = false;
    dkinfo* u = pq_ptr.back(); 
    pq_ptr.pop_back();
    if(pq_ptr.size() > 0)
    pop_heap(pq_ptr.begin(),pq_ptr.end(),dkinfo_ptr_compare);
    if (u->dist >max_shortest_path_length) { //return value
      max_shortest_path_length = u->dist;
      *far_vid = u->vid;
    }
    vid_dk_map.erase(u->vid); //mapping is no longer needed.
    vector<VID> adj;
    g.GetDijkstraInfo(u->vid,adj);

    // check all u's successors
    for (int i = 0; i < adj.size(); i++)
    {
      if(vid_dk_map.count(adj[i]) > 0) {
        dkinfo* succ= vid_dk_map[adj[i]];
        double wt = g.GetEdgeWeight(u->vid,succ->vid).Weight();
        if(!succ->valid()) {
          //relax = true;
          succ->dist = u->dist + wt;
          succ->predvid = u->vid;
          succ->VertexValid=true;
          pq_ptr.push_back(succ);
          push_heap(pq_ptr.begin(), pq_ptr.end(),dkinfo_ptr_compare);
          pop_heap(pq_ptr.begin(),pq_ptr.end(),dkinfo_ptr_compare);
        }
        else if ( succ->dist > u->dist + wt ) {
          relax = true;
          succ->dist = u->dist + wt;
          succ->predvid = u->vid;
          succ->VertexValid=true;
        }
      }
    } // endfor
    if (relax && pq_ptr.size() > 0) { 
      make_heap( pq_ptr.begin(), pq_ptr.end(),dkinfo_ptr_compare);
      pop_heap(pq_ptr.begin(),pq_ptr.end(),dkinfo_ptr_compare);
    }
  }
  return max_shortest_path_length;
}
















#endif
