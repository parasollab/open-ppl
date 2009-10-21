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
#include "BasicDefns.h"

////////////////////////////////////////////////////////////////////////////////////////////
#define DEFAULT_EDGES_PER_VERTEX 10  ///< Slots to reserve in each edgelist

#ifndef INVALID_VID
#define INVALID_VID (std::numeric_limits<size_t>::max())
#endif

#ifndef INVALID_EID
#define INVALID_EID (std::numeric_limits<size_t>::max())
#endif


using stapl::dkinfo;
using stapl::dkinfo_compare;
//using stapl::VID;

////////////////////////////////////////////////////////////////////////////////////////////
//is it still being used? 
template<class VERTEX, class WEIGHT>
class EdgeInfo {
public:
typedef typename stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, VERTEX,WEIGHT>::vertex_descriptor VID;
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
public stapl::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT> {
//public stapl::Graph<stapl::DIRECTED,stapl::NONMULTIEDGES,stapl::WEIGHTED, VERTEX,WEIGHT> {

public:

typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, VERTEX,WEIGHT> GRAPH;
//typedef stapl::Graph<stapl::DIRECTED, stapl::NONMULTIEDGES, stapl::WEIGHTED, VERTEX,WEIGHT> GRAPH;
typedef typename GRAPH::vertex_descriptor VID;
typedef typename stapl::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT>::vertex_iterator VI; ///<VI Vertex Iterator
typedef typename stapl::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT>::const_vertex_iterator CVI; ///<CVI Constant Vertex Iterator
typedef typename stapl::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT>::adj_edge_iterator EI;
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
       virtual int  AddEdges( vector<EdgeInfo<VERTEX, WEIGHT> >& );
       
       virtual int  AddEdge(VID, VID, WEIGHT);
       virtual int  AddEdge(VERTEX&, VERTEX&, WEIGHT); 
       virtual int  AddEdge(VID, VID, pair<WEIGHT,WEIGHT>&);
       virtual int  AddEdge(VERTEX&, VERTEX&, pair<WEIGHT,WEIGHT>&);
       virtual int  AddEdge(VID, VID, pair<WEIGHT*,WEIGHT*>&);
       virtual int  AddEdge(VERTEX&, VERTEX&, pair<WEIGHT*,WEIGHT*>&);
       virtual int  GetVerticesData(vector<VERTEX>& ) const; //get all vertices data from whole graph
       virtual int  GetVerticesVID(vector<VID>& ) const;     //get all VIDs from whole graph
       virtual vector<VID> ConvertVertices2VIDs(vector<VERTEX>&);  //convert vertices to corresponding VIDs
       virtual vector<VERTEX> ConvertVIDs2Vertices(vector<VID>&);  //convert VIDs to vertices
       virtual VID  GetVID(VERTEX&);
       virtual bool IsVertex(VERTEX&);
       virtual bool IsVertex(VERTEX&, CVI*) ; 
       virtual bool IsEdge(VERTEX& v1, VERTEX& v2);
       virtual bool IsEdge(VID vid1, VID vid2);
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

   //typedef stapl::WtVertexType<VERTEX,WEIGHT> Vertex;
   //typedef vector< Vertex > VERTEX_VECTOR;
   //typedef typename VERTEX_VECTOR::iterator VI; ///<VI Vertex Iterator
   //typedef typename VERTEX_VECTOR::const_iterator CVI; ///<CVI Constant Vertex Iterator

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
    //this->v.reserve(_numNodes); // fix_lantao
    //this->reserveEdgesPerVertex = _numEdges;
}

  //==================================
  // RoadmapGraph class Methods: Adding & Deleting Vertices
  //==================================

// require that VERTEX data (configuration) is unique
template<class VERTEX, class WEIGHT>
//VID
typename stapl::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT>::vertex_descriptor
RoadmapGraph<VERTEX,WEIGHT>::
AddVertex(VERTEX& _v1) {
    CVI v1;
    if ( !IsVertex(_v1,&v1) ) {
        //return GRAPH::AddVertex(_v1);
        return GRAPH::add_vertex(_v1);
    } else {
#ifndef QUIETGRAPH
        cout << "\nIn AddVertex: vertex already in graph, not added";
#endif
        //return (v1->vid); // return vertex id 
        return (v1.descriptor()); // return vertex id 
    }
};

// require that VERTEX data (configuration) is unique
template<class VERTEX, class WEIGHT>
//VID
typename stapl::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT>::vertex_descriptor
RoadmapGraph<VERTEX,WEIGHT>::
AddVertex(vector<VERTEX>& _v) {
    bool added=false;
    VID vertex_id; //=this->GetNextVID(); fix_lantao
    for (int i = 0; i < _v.size(); i++){
        if (!IsVertex(_v[i])){
        //if (!this->find_vertex(_v[i])){
	  //GRAPH::AddVertex(_v[i]);
	  vertex_id = GRAPH::add_vertex(_v[i]);
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

//fix_lantao does this really in need? EdgeInfo?
template<class VERTEX, class WEIGHT>
int
RoadmapGraph<VERTEX,WEIGHT>::
AddEdges( vector<EdgeInfo<VERTEX, WEIGHT> >& _e) {
    for (unsigned int i=0; i < _e.size(); i++){
        GRAPH::add_edge(_e[i].v1, _e[i].v2, _e[i].edgewt);
    }
    return OK;
}

template <class VERTEX, class WEIGHT>
int  
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1, VID _v2, WEIGHT _w) {
  //return GRAPH::add_edge(_v1,_v2,_w);
  GRAPH::add_edge(_v1,_v2,_w);
  return OK;  //fix_lantao  the return type and the following AddEdge funcs.
}

template <class VERTEX, class WEIGHT>
int  
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _w) {
  //return GRAPH::add_edge(_v1,_v2,_w);
  GRAPH::add_edge(GetVID(_v1),GetVID(_v2),_w);
  return OK;
}

template <class VERTEX, class WEIGHT>
int  
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1, VID _v2, pair<WEIGHT,WEIGHT>& _w) {
  GRAPH::add_edge(_v1,_v2,_w.first);
  //return GRAPH::add_edge(_v2,_v1,_w.second);
  GRAPH::add_edge(_v2,_v1,_w.second);
  return OK;
}

template <class VERTEX, class WEIGHT>
int  
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, pair<WEIGHT,WEIGHT>& _w) {
  GRAPH::add_edge(GetVID(_v1),GetVID(_v2),_w.first);
  //return GRAPH::add_edge(_v2,_v1,_w.second);
  GRAPH::add_edge(GetVID(_v2),GetVID(_v1),_w.second);
  return OK;
}

template <class VERTEX, class WEIGHT>
int
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1, VID _v2, pair<WEIGHT*,WEIGHT*>& _w) {
  pair<WEIGHT,WEIGHT> tmp;
  tmp.first = *_w.first;
  tmp.second = *_w.second;
  AddEdge(_v1,_v2,tmp);
  return OK;
}

template <class VERTEX, class WEIGHT>
int
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, pair<WEIGHT*,WEIGHT*>& _w) {
  pair<WEIGHT,WEIGHT> tmp;
  tmp.first = *_w.first;
  tmp.second = *_w.second;
  AddEdge(_v1,_v2,tmp);
  return OK;
}

template <class VERTEX, class WEIGHT>
//typename vector<VID> 
vector<typename stapl::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT>::vertex_descriptor> 
RoadmapGraph<VERTEX,WEIGHT>::
MergeRoadMap(RoadmapGraph<VERTEX, WEIGHT>* _fromMap, 
	      vector<VID> vids) {
  VERTEX t;
  int i;
  vector<VID> newVids;

  //Add vertex
  for (i=0;i<vids.size();++i) {
    t=_fromMap->find_vertex(vids[i]).property();
    
    newVids.push_back(this->AddVertex(t));
  } //endfor i


  //-- get edges from _rm connected component and add to submap
  for (i=0;i<vids.size();++i) {
    vector< pair<pair<VID,VID>,WEIGHT> > edges; 
    //_fromMap->GetOutgoingEdges(vids[i], edges); fix_lantao
    //use iterator to traverse the adj edges and then put the data into edges
    typename RoadmapGraph<VERTEX, WEIGHT>::vertex_iterator vi = _fromMap->find_vertex(vids[i]);
    for(typename RoadmapGraph<VERTEX, WEIGHT>::adj_edge_iterator ei =vi.begin(); ei!=vi.end(); ei++ ){
        pair<pair<VID,VID>,WEIGHT> single_edge;
        single_edge.first.first=ei.source();
        single_edge.first.second=ei.target();
        single_edge.second = ei.property();
        edges.push_back(single_edge); //put the edge into edges
    }

    for (int j=0;j<edges.size();++j) {
      VERTEX t1=_fromMap->find_vertex(edges[j].first.first).property();
      VERTEX t2=_fromMap->find_vertex(edges[j].first.second).property();
      
      this->AddEdge(t1,t2, edges[j].second);
    } //endfor j
    
  } //endfor i


  return newVids;


}

template <class VERTEX, class WEIGHT>
vector<typename stapl::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT>::vertex_descriptor>
RoadmapGraph<VERTEX,WEIGHT>::
ConvertVertices2VIDs(vector<VERTEX>& vps){
  vector<VID> vds;
  vds.reserve(sizeof(vps));
  for(typename vector<VERTEX>::iterator itr = vps.begin(); itr != vps.end(); itr++)
	vds.push_back(this->GetVID(*itr));
  return vds;
}

template <class VERTEX, class WEIGHT>
vector<VERTEX>
RoadmapGraph<VERTEX,WEIGHT>::
ConvertVIDs2Vertices(vector<VID>& vds){
  vector<VERTEX> vps;
  vps.reserve(sizeof(vds));
  for(typename vector<VID>::iterator itr = vds.begin(); itr!=vds.end(); itr++)
	vps.push_back(this->find_vertex(*itr).property());
  return vps;
}


template <class VERTEX, class WEIGHT>
int
RoadmapGraph<VERTEX,WEIGHT>::
GetVerticesData(vector<VERTEX>& v_vp) const{
  v_vp.clear();
  GRAPH g = *this;
  //verts.reserve( g.size() ); //g has no member size();
    for (CVI vi = g.begin(); vi!= g.end(); vi++) {
        v_vp.push_back(vi.property());
    }
    return v_vp.size();
}

template <class VERTEX, class WEIGHT>
int
RoadmapGraph<VERTEX,WEIGHT>::
GetVerticesVID(vector<VID>& v_vd) const{
  v_vd.clear();
  GRAPH g = *this;
  //verts.reserve( g.size() ); //g has no member size();
    for (CVI vi = g.begin(); vi!= g.end(); vi++) {
        v_vd.push_back(vi.descriptor());
    }
    return v_vd.size();
}


//lantao
template<class VERTEX, class WEIGHT>
typename RoadmapGraph<VERTEX, WEIGHT>::VID
RoadmapGraph<VERTEX,WEIGHT>::
GetVID(VERTEX& _v1) {
    CVI v1;
    if ( IsVertex(_v1,&v1) ) {
        //return v1->vid;
        return v1.descriptor();
    } else {
        return INVALID_VID;
    }
}

template<class VERTEX, class WEIGHT>
bool
RoadmapGraph<VERTEX,WEIGHT>::
IsVertex(VERTEX& _v1){
    CVI v1;
    return ( IsVertex(_v1,&v1) );
}

/*
//used as predicate of searching a vertex
template<class VERTEX,class WEIGHT>
class Vertex_equal_to{
public:
//typedef typename stapl::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT>::const_vertex_iterator CVI;
typedef typename RoadmapGraph<VERTEX, WEIGHT>::const_vertex_iterator CVI;
Vertex_equal_to(VERTEX& vertex):v(vertex){}
~Vertex_equal_to(){}
bool operator()(CVI& cvi){
if(cvi.property() == v) return true;
else return false;
}

//data member
private:
VERTEX v;

};
*/

//added lantao, double check fix_lantao
template<class VERTEX, class WEIGHT>
bool
RoadmapGraph<VERTEX,WEIGHT>::
IsVertex(VERTEX& _v1, CVI*  _v1ptr)  {

    RoadmapGraph<VERTEX, WEIGHT> g = *this;
    //CVI v1 = find_if(g.begin(), g.end(), std::bind2nd(std::equal_to<VERTEX>(), _v1 ));
    //CVI v1=g.begin() ;//= find_if(g.begin(), g.end(), Vertex_equal_to<VERTEX, WEIGHT>(_v1) ); 

/*
  //non constant stuff
    VI v1 = my_find_VDATA_eq(_v1);
    
  //definition here
    my_find_VDATA_eq(const VERTEX& _v)  {
    VI vi = v.begin();
    bool found = false;
    while (vi != v.end() && !found) {
      if ( vi->data == _v) {
        found = true;
      } else {
        vi++;
      }
    }
    return (vi);

*/
    VI vi = g.begin();
    bool found = false;
    while(vi != g.end() && !found){
	if(vi.property() == _v1)
	  found = true;
	else
	  vi++;
    }// end while

    if (vi != g.end() ) {
        *_v1ptr = vi;
        return true;
    } else {
        return false;
    }
}


template<class VERTEX, class WEIGHT>
bool
RoadmapGraph<VERTEX,WEIGHT>::
IsEdge(VID vid1, VID vid2){
bool is_edge;
typename RoadmapGraph<VERTEX, WEIGHT>::vertex_iterator vi;
typename RoadmapGraph<VERTEX, WEIGHT>::adj_edge_iterator ei;
typename RoadmapGraph<VERTEX, WEIGHT>::edge_descriptor ed(vid1,vid2);
is_edge=find_edge(ed, vi, ei);
return is_edge;
}

template<class VERTEX, class WEIGHT>
bool
RoadmapGraph<VERTEX,WEIGHT>::
IsEdge(VERTEX& v1, VERTEX& v2){
VID vid1, vid2;
vid1= GetVID(v1);
vid2= GetVID(v2);
return IsEdge(v1, v2);
}


/*****************************************************************************/
//Dijkstra related.


//static bool dkinfo_ptr_compare (const dkinfo<VID>* d1, const dkinfo<VID>* d2)
template<class DKINFO>
static bool dkinfo_ptr_compare (const DKINFO* d1, const DKINFO* d2)
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


// Remove worning: function decleared but never refernced
template<class DKINFO>
bool my_dkinfo_compare (const DKINFO& d1, const DKINFO& d2)
{
        //d1.print();
        //d2.print();
        if ((!d1.valid())&&(!d2.valid()))
        {
                //cout<<"Both d1 and d2 are not valid"<<endl;
                //The returned value does not make much sense
                return ( d1.dist > d2.dist );
        }
        else if(!d1.valid())
        {
                //cout<<"D1 not valid"<<endl;
                //So d1.dist > d2.dist
                return true;
        }
        else if(!d2.valid())
        {
                //cout<<"D2 is not valid"<<endl;
                return false;
        }
        else
        {
                //cout<<"Both are valid"<<endl;         
                return d1.dist > d2.dist;
        }

}



//////////////FROM MARCO//////////////////
template<class GRAPH>
//double DijkstraSSSP(GRAPH &g,VID start_vid, VID *far_vid)
double DijkstraSSSP(GRAPH &g,typename GRAPH::vertex_descriptor start_vid, typename GRAPH::vertex_descriptor *far_vid)
{
  typedef typename GRAPH::vertex_descriptor VID;
  //typename GRAPH::WEIGHT_TYPE _w_;
  typename GRAPH::edge_property _w_;
  typename GRAPH::const_vertex_iterator cv1;
  typename GRAPH::vertex_iterator v;
  stapl::vector_property_map< stapl::stapl_color<size_t> > cmap;

  vector<VID> vec_cc;
  vector<dkinfo<VID> > pq; //The priority Queue
  double  maxdist = g.get_num_vertices() * _w_.MaxWeight();
  double max_shortest_path_length=0;
  *far_vid = start_vid;

  get_cc(g, cmap, start_vid, vec_cc);
  typename vector<VID>::iterator vid_itr;
  for ( vid_itr = vec_cc.begin(); vid_itr != vec_cc.end(); vid_itr++) 
  {
    if (*vid_itr == start_vid) 
    {
      pq.push_back( dkinfo<VID>(true,*vid_itr,*vid_itr,0) );
    } else {
      pq.push_back( dkinfo<VID>(false,*vid_itr,-1,-1000) );
    }
  }

  sort( pq.begin(), pq.end(),  (my_dkinfo_compare<dkinfo<VID> >) );
  //sort( pq.begin(), pq.end(),  dkinfo_compare<VID, dkinfo<VID>, GRAPH>(g) );

  while ( pq.size() != 0  && (pq.back().valid()) )
  {
    bool relax = false;
    dkinfo<VID> u = pq.back();

    if (u.dist >max_shortest_path_length) { //return value
      max_shortest_path_length = u.dist;
      *far_vid = u.vd;
    }

    pq.pop_back();
    vector<VID> adj;
    //g.GetDijkstraInfo(u.vd,adj);
    g.get_adjacent_vertices(u.vd,adj);

    // check all u's successors
    for (int i = 0; i < adj.size(); i++)
    {
      for (int j=0; j < pq.size(); j++)
      {
        if (adj[i] == pq[j].vd)
        {
	  typename GRAPH::edge_descriptor ed(u.vd,pq[j].vd);
    	  typename GRAPH::vertex_iterator vi; 
	  typename GRAPH::adj_edge_iterator ei;
    	  g.find_edge(ed, vi, ei); 
          double wt = ei.property().Weight(); //g.GetEdgeWeight(u.vd,pq[j].vd).Weight(); fix_lantao

          if(!pq[j].valid())
          {
            relax = true;
            pq[j].dist = u.dist + wt;
            pq[j].predvd = u.vd;
            pq[j].VertexValid=true;
          }
          else if ( pq[j].dist > u.dist + wt )
          {
            relax = true;
            pq[j].dist = u.dist + wt;
            pq[j].predvd = u.vd;
            pq[j].VertexValid=true;
          }
        }
      } // endfor
    } // endfor

    if (relax) sort( pq.begin(), pq.end(),my_dkinfo_compare<dkinfo<VID> >);
    //if (relax) sort( pq.begin(), pq.end(),dkinfo_compare< VID, dkinfo<VID>, GRAPH >(g) );
  }
  return max_shortest_path_length;

}

////////////////END FROM MARCO///////////////



template<class GRAPH>
//double ComponentDiameter(GRAPH &g,VID start_vid, VID *far_vid)
double ComponentDiameter(GRAPH &g,typename GRAPH::vertex_descriptor start_vid, typename GRAPH::vertex_descriptor *far_vid)
{

  typedef typename GRAPH::vertex_descriptor VID;
  //typename GRAPH::WEIGHT_TYPE _w_;
  typename GRAPH::edge_descriptor _w_;
  typename GRAPH::const_vertex_iterator cv1;
  typename GRAPH::vertex_iterator v1;
  stapl::vector_property_map< stapl::stapl_color<size_t> > cmap;

  vector<VID> vec_cc;
  double  maxdist = 0;//g.get_num_vertices() * _w_.MaxWeight(); fix_lantao
  double max_shortest_path_length=0;
  *far_vid = start_vid;

  get_cc(g, cmap,  start_vid, vec_cc);
  int CC_size = vec_cc.size();
  vector<dkinfo<VID> > pq_dk(CC_size); //The priority Queue -- Data only
  vector<dkinfo<VID>*> pq_ptr; //The REAL priority Queue -- heap
  pq_ptr.reserve(CC_size);
  map<VID,dkinfo<VID>*> vid_dk_map;
  for(int i=0; i<CC_size; ++i) {
    if (vec_cc[i] == start_vid) 
    {
      pq_dk[i] = dkinfo<VID>(true,start_vid,start_vid,0);
      pq_ptr.push_back(&(pq_dk[i]));
    } else {
      pq_dk[i] = dkinfo<VID>(false,vec_cc[i],-1,maxdist);
    }
    //pq_ptr[i] = &(pq_dk[i]);
    vid_dk_map[vec_cc[i]] = &(pq_dk[i]);
  }

  //make_heap( pq_ptr.begin(), pq_ptr.end(), dkinfo_ptr_compare);
  //pop_heap(pq_ptr.begin(),pq_ptr.end(),dkinfo_ptr_compare); //

  while ( pq_ptr.size() != 0  && (pq_ptr.back()->valid()) )
  {
    bool relax = false;
    dkinfo<VID>* u = pq_ptr.back(); 
    pq_ptr.pop_back();
    if(pq_ptr.size() > 0)
    pop_heap(pq_ptr.begin(),pq_ptr.end(),dkinfo_ptr_compare<dkinfo<VID> >);
    if (u->dist >max_shortest_path_length) { //return value
      max_shortest_path_length = u->dist;
      *far_vid = u->vd;
    }
    vid_dk_map.erase(u->vd); //mapping is no longer needed.
    vector<VID> adj;
    //g.GetDijkstraInfo(u->vd,adj);
    g.get_adjacent_vertices(u->vd,adj);

    // check all u's successors
    for (int i = 0; i < adj.size(); i++)
    {
      if(vid_dk_map.count(adj[i]) > 0) {
        dkinfo<VID>* succ= vid_dk_map[adj[i]];
        //double wt =g.GetEdgeWeight(u->vd,succ->vd).Weight(); fix_lantao
	typename GRAPH::edge_descriptor ed(u->vd, succ->vd);
        typename GRAPH::vertex_iterator vi;
        typename GRAPH::adj_edge_iterator ei;
        g.find_edge(ed, vi, ei);
        double wt = ei.property().Weight();
        
        if(!succ->valid()) {
          //relax = true;
          succ->dist = u->dist + wt;
          succ->predvd = u->vd;
          succ->VertexValid=true;
          pq_ptr.push_back(succ);
          push_heap(pq_ptr.begin(), pq_ptr.end(),dkinfo_ptr_compare<dkinfo<VID> >);
          pop_heap(pq_ptr.begin(),pq_ptr.end(),dkinfo_ptr_compare<dkinfo<VID> >);
        }
        else if ( succ->dist > u->dist + wt ) {
          relax = true;
          succ->dist = u->dist + wt;
          succ->predvd = u->vd;
          succ->VertexValid=true;
	}
      }
    } // endfor
    if (relax && pq_ptr.size() > 0) { 
      make_heap( pq_ptr.begin(), pq_ptr.end(),dkinfo_ptr_compare<dkinfo<VID> >);
      pop_heap(pq_ptr.begin(),pq_ptr.end(),dkinfo_ptr_compare<dkinfo<VID> >);
    }
  }
  return max_shortest_path_length;

}

#endif
