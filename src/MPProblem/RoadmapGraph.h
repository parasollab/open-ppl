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
#ifdef _PARALLEL
//#include "p_graph.h"
//#include <containers/graph/graph.hpp>
#include <containers/graph/dynamic_graph.hpp>
#else
#include "Graph.h"
#endif
#include "GraphAlgo.h"

#include "RoadmapVCS.h"

////////////////////////////////////////////////////////////////////////////////////////////
#define DEFAULT_EDGES_PER_VERTEX 10  ///< Slots to reserve in each edgelist

#ifndef INVALID_VID
#define INVALID_VID (std::numeric_limits<size_t>::max())
#endif

#ifndef INVALID_EID
#define INVALID_EID (std::numeric_limits<size_t>::max())
#endif


//using stapl::dkinfo;
//using stapl::dkinfo_compare;
//using stapl::VID;
// store info for djistra algo i.e edgeweight etc

////////////////////////////////////////////////////////////////////////////////////////////
//is it still being used? 
template<class VERTEX, class WEIGHT>
class EdgeInfo {
public:
#ifdef _PARALLEL
typedef typename stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, VERTEX,WEIGHT> GRAPH;
#else 
typedef typename stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, VERTEX,WEIGHT> GRAPH;
#endif 
typedef typename GRAPH::vertex_descriptor VID;
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


template<class VD>
class dkinfo{
	public:
  bool VertexValid; //Keeps track of whether the second vertex is valid or not
  VD vd;        //This vertex ID
  // If the VertexValid flag is set to false then the next two variables does not have any meaning and should not be used
  
  VD predvd; //ID of the previous vertex in the graph
  double dist; //Distance between the two vertices.
  
  dkinfo()
    {
      VertexValid=false;
      vd=0;
      predvd=INVALID_VALUE;
      dist=-1000;
      // cout<<"Check initialization of dist "<<__LINE__<<" "<<__FILE__<<endl;
    }
  
  dkinfo(bool flag,VD _vd, VD _pvd,double _dist)
    {
      VertexValid=flag;
      vd=_vd;
      predvd=_pvd;
      dist=_dist;
    }
  
  void print() const
  {
    std::cout<<"Vertex : "<<vd
             <<"  Prev Vertex : "<<predvd
             <<"  Distance : "<<dist
             <<"  Valid ? "<<VertexValid<<std::endl;
  }
  bool valid() const  { return VertexValid; }  
};


// Remove worning: function decleared but never refernced
// Removed #ifdef (
template <class VD, class DKinfo, /*class Comparator,*/ typename Map>
struct dkinfo_compare {
  // Comparator comp;
  Map* map;
  dkinfo_compare(/*Comparator& comp_, */Map* map_) : /*comp(comp_),*/ map(map_) { }
  bool operator() (const VD& d1, const VD& d2) const {
    //(*map)[d1].print();
    //(*map)[d2].print();
    if ((!(*map)[d1].valid())&&(!(*map)[d2].valid())) {
      //cout<<"Both d1 and d2 are not valid"<<endl;
      //The returned value does not make much sense
      return ( (*map)[d1].dist > (*map)[d2].dist );
    } else if(!(*map)[d1].valid()) {
      //cout<<"D1 not valid"<<endl;
      //So d1.dist > d2.dist
      return false;
    } else if(!(*map)[d2].valid()) {
      //cout<<"D2 is not valid"<<endl;
      return true;
    } else {
      //cout<<"Both are valid"<<endl;                
      return (*map)[d1].dist > (*map)[d2].dist;
    }
  }
};

template <typename VertexIterator>
class vertex_descriptor_iterator
  : public boost::iterator_adaptor<
                                   vertex_descriptor_iterator<VertexIterator>   //Derived
                                   , VertexIterator                             //Base
                                   , typename VertexIterator::value_type::vertex_descriptor //Value
                                   , boost::use_default                         //CategoryOrTraversal
                                   , typename VertexIterator::value_type::vertex_descriptor //Reference
                                  >
{
 public:
  vertex_descriptor_iterator()
    : vertex_descriptor_iterator::iterator_adaptor_() 
  {}
  
  explicit vertex_descriptor_iterator(VertexIterator vi)
    : vertex_descriptor_iterator::iterator_adaptor_(vi)
  {}
  
  template <typename OtherVertexIterator>
  vertex_descriptor_iterator(vertex_descriptor_iterator<OtherVertexIterator> const other)
    : vertex_descriptor_iterator::iterator_adaptor_(other.base())
  {}

  //overload dereference to call descriptor() instead
  typename VertexIterator::value_type::vertex_descriptor dereference() const  
  { 
    return this->base()->descriptor(); 
  }
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


#ifdef _PARALLEL
template<class VERTEX, class WEIGHT>
class RoadmapGraph : 
public stapl::dynamic_graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT> {
#else 
template<class VERTEX, class WEIGHT>
class RoadmapGraph : 
public stapl::sequential::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT> {
#endif 

public:

#ifdef _PARALLEL
typedef stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, VERTEX,WEIGHT> GRAPH;
#else 
typedef stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, VERTEX,WEIGHT> GRAPH;
#endif 
typedef typename GRAPH::vertex_descriptor VID;
typedef typename GRAPH::edge_descriptor EID;
typedef typename GRAPH::vertex_iterator VI; ///<VI Vertex Iterator
#ifndef _PARALLEL
///TODO: Remove guard after const issue is fixed in STAPL
typedef typename GRAPH::const_vertex_iterator CVI; ///< not sure CVI Constant Vertex Iterator
typedef vertex_descriptor_iterator<CVI> CVDI;
typedef typename GRAPH::vertex_property& VP;
#else
typedef typename GRAPH::vertex_iterator CVI; 
typedef vertex_descriptor_iterator<CVI> CVDI;
typedef typename GRAPH::vertex_property VP;
#endif 
typedef typename GRAPH::adj_edge_iterator EI;
typedef vertex_descriptor_iterator<VI> VDI;
typedef RoadmapChangeEvent<VERTEX, WEIGHT> ChangeEvent;

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
       virtual vector<VID>  AddVertex(vector<VERTEX>&);

       /**Add edges by a list of EdgeInfo<WEIGHT>.
         *@return always return OK.
         *@see WeightedGraph::AddEdge(VID, VID, pair<WEIGHT,WEIGHT>)
         */
       ///add_edge is broken in new pGraph, remove ifdef after fix
      // #ifndef _PARALLEL
       virtual int  AddEdges( vector<EdgeInfo<VERTEX, WEIGHT> >& );
       virtual int  AddEdge(VID, VID, WEIGHT);
       virtual int  AddEdge(VID, VID, pair<WEIGHT,WEIGHT>&);
       virtual int  AddEdge(VID, VID, pair<WEIGHT*,WEIGHT*>&);
       virtual int  AddEdge(VERTEX&, VERTEX&, pair<WEIGHT*,WEIGHT*>&);
       virtual bool IsVertex(VERTEX&, CVI*) ;
       virtual int  AddEdge(VERTEX&, VERTEX&, WEIGHT); 
       virtual int  AddEdge(VERTEX&, VERTEX&, pair<WEIGHT,WEIGHT>&);
       //#endif
       virtual int  GetVerticesData(vector<VERTEX>& ) const; //get all vertices data from whole graph
       virtual int  GetVerticesVID(vector<VID>& ) const;     //get all VIDs from whole graph
       virtual vector<VID> ConvertVertices2VIDs(vector<VERTEX>&);  //convert vertices to corresponding VIDs
       virtual vector<VERTEX> ConvertVIDs2Vertices(vector<VID>&);  //convert VIDs to vertices
       virtual VID  GetVID(VERTEX&);
       virtual bool IsVertex(VERTEX&); 
       virtual bool IsEdge(VERTEX& v1, VERTEX& v2);
       virtual bool IsEdge(VID vid1, VID vid2);
       
       //helper function to call dereferece on an iterator whose
       //value_type is VID and convert to CfgType
       template<typename T>
       VP GetCfg(T& _t);

       //specialization for a roadmap graph iterator, calls property()
       //template<typename RDMP::VI>
       VP  GetCfg(VI& _t);

       //specialization for a RoadmapGraph<CFG, WEIGHT>::VID
       //calls find_vertex(..) on VID to call property()
       //To do:Temporarily removed constness until it is supported in STAPL
       VP  GetCfg(VID _t) ;

       //helper function to call dereferece on an iterator whose value_type is VID
       //needed to get around the fact that a roadmap graph iterator
       //requires an extra descriptor() call
       template<typename T>
       VID GetVid(T& _t);

       //specialization for a roadmap graph iterator, calls descriptor()
       //template<typename RDMP::VI>
       VID GetVid(VI& _t);
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


   #ifndef _PARALLEL
   VDI descriptor_begin() { return VDI(this->begin()); }
   VDI descriptor_end() { return VDI(this->end()); }
   CVDI descriptor_begin() const { return CVDI(this->begin()); }
   CVDI descriptor_end() const { return CVDI(this->end()); }
   #endif

   ///Temporarily wrapper for some graph methods
   ///Until full migration and change of names in STAPL is completed
   #ifdef _PARALLEL
   VI descriptor_begin() { return GRAPH::begin(); }
   VI descriptor_end() { return GRAPH::end(); }
   size_t get_num_edges() { return GRAPH::num_edges();}
   size_t get_num_vertices() { return GRAPH::num_vertices();}
   size_t get_degree(const VID& _vd) {
    VI vi = GRAPH::find_vertex(_vd);
    return (*vi).size();
  }
  size_t get_out_degree(const VID& _vd){
	  return this->get_degree(_vd);
  }
  
  void delete_vertex(const VID& _vd){
    cout << "WARNING- Delete vertex not working pgraph" << endl;   
  }
  
  /*void AddEdge(VID _v1, VID _v2, pair<WEIGHT,WEIGHT>& _w){
	  GRAPH::add_edge_async(_v1,_v2,_w.first);
  }*/
	  

   #endif

  typedef RoadmapVCS<VERTEX, WEIGHT> RoadmapVCSType;

  RoadmapVCSType roadmapVCS;
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
typename RoadmapGraph<VERTEX, WEIGHT>::VID
RoadmapGraph<VERTEX,WEIGHT>::
AddVertex(VERTEX& _v1) {
    #ifndef _PARALLEL
    CVI v1;
    if ( !IsVertex(_v1,&v1) ) {
        VID vertex_id = GRAPH::add_vertex(_v1);
        VDAddNode(_v1);
        ChangeEvent event(ChangeEvent::ADD_VERTEX, _v1, vertex_id);
        //cout << "Adding through single" << endl;
        roadmapVCS.addEvent(event);
        return vertex_id;
    } else {
#ifndef QUIETGRAPH
        cout << "\nIn AddVertex: vertex already in graph, not added" << endl;
#endif
        //return (v1->vid); // return vertex id 
        return ((*v1).descriptor()); // return vertex id 
    }
   #else 
        VID vertex_id = GRAPH::add_vertex(_v1);
        return vertex_id;
    #endif
};

// require that VERTEX data (configuration) is unique
template<class VERTEX, class WEIGHT>
vector<typename RoadmapGraph<VERTEX,WEIGHT>::VID>
RoadmapGraph<VERTEX,WEIGHT>::AddVertex(vector<VERTEX>& _v) {
    vector<VID> vids;
    for (size_t i = 0; i < _v.size(); i++){
      vids.push_back(AddVertex(_v[i]));
    }
    return vids;
}

//fix_lantao does this really in need? EdgeInfo?
///add_edge is broken in new pGraph remove ifdef after fix
//#ifndef _PARALLEL
template<class VERTEX, class WEIGHT>
int
RoadmapGraph<VERTEX,WEIGHT>::
AddEdges( vector<EdgeInfo<VERTEX, WEIGHT> >& _e) {
    for (unsigned int i=0; i < _e.size(); i++){
        GRAPH::add_edge(_e[i].v1, _e[i].v2, _e[i].edgewt);
	VDAddEdge(this->GetCfg(_e[i].v1), this->GetCfg(_e[i].v2));
    }
    return 0;
}

template <class VERTEX, class WEIGHT>
int  
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1, VID _v2, WEIGHT _w) {
  GRAPH::add_edge(_v1,_v2,_w);
  VDAddEdge(this->GetCfg(_v1), this->GetCfg(_v2));
  return 0;  //fix_lantao  the return type and the following AddEdge funcs.
}

template <class VERTEX, class WEIGHT>
int  
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1, VID _v2, pair<WEIGHT,WEIGHT>& _w) {
  GRAPH::add_edge(_v1,_v2,_w.first);
  GRAPH::add_edge(_v2,_v1,_w.second);
  VDAddEdge(this->GetCfg(_v1), this->GetCfg(_v2));
  return 0;
}

template <class VERTEX, class WEIGHT>
int
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VID _v1, VID _v2, pair<WEIGHT*,WEIGHT*>& _w) {
  pair<WEIGHT,WEIGHT> tmp;
  tmp.first = *_w.first;
  tmp.second = *_w.second;
  AddEdge(_v1,_v2,tmp);
  return 0;
}

template <class VERTEX, class WEIGHT>
int
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, pair<WEIGHT*,WEIGHT*>& _w) {
  pair<WEIGHT,WEIGHT> tmp;
  tmp.first = *_w.first;
  tmp.second = *_w.second;
  AddEdge(_v1,_v2,tmp);
  VDAddEdge(_v1, _v2);
  return 0;
}

template <class VERTEX, class WEIGHT>
int  
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _w) {
  GRAPH::add_edge(GetVID(_v1),GetVID(_v2),_w);
  VDAddEdge(_v1, _v2);
  return 0;
}

template <class VERTEX, class WEIGHT>
int  
RoadmapGraph<VERTEX,WEIGHT>::
AddEdge(VERTEX& _v1, VERTEX& _v2, pair<WEIGHT,WEIGHT>& _w) {
  GRAPH::add_edge(GetVID(_v1),GetVID(_v2),_w.first);
  GRAPH::add_edge(GetVID(_v2),GetVID(_v1),_w.second);
  VDAddEdge(_v1, _v2);
  return 0;
}


template <class VERTEX, class WEIGHT>
vector<typename RoadmapGraph<VERTEX, WEIGHT>::VID> 
RoadmapGraph<VERTEX,WEIGHT>::
MergeRoadMap(RoadmapGraph<VERTEX, WEIGHT>* _fromMap, 
	      vector<VID> vids) {
  VERTEX t;
  int i;
  vector<VID> newVids;

  //Add vertex
  for (i=0;i<vids.size();++i) {
    t=(*(_fromMap->find_vertex(vids[i]))).property();
    
    newVids.push_back(this->AddVertex(t));
  } //endfor i


  //-- get edges from _rm connected component and add to submap
  for (i=0;i<vids.size();++i) {
    vector< pair<pair<VID,VID>,WEIGHT> > edges; 
    //_fromMap->GetOutgoingEdges(vids[i], edges); fix_lantao
    //use iterator to traverse the adj edges and then put the data into edges
    typename RoadmapGraph<VERTEX, WEIGHT>::VI vi = _fromMap->find_vertex(vids[i]);
    for(typename RoadmapGraph<VERTEX, WEIGHT>::EI ei = vi.begin(); ei!=vi.end(); ei++ ){
        pair<pair<VID,VID>,WEIGHT> single_edge;
        single_edge.first.first=*ei.source();
        single_edge.first.second=*ei.target();
        single_edge.second = *ei.property();
        edges.push_back(single_edge); //put the edge into edges
    }

    for (int j=0;j<edges.size();++j) {
      VERTEX t1=(*(_fromMap->find_vertex(edges[j].first.first))).property();
      VERTEX t2=(*(_fromMap->find_vertex(edges[j].first.second))).property();
      
      this->AddEdge(t1,t2, edges[j].second);
    } //endfor j
    
  } //endfor i


  return newVids;


}

template <class VERTEX, class WEIGHT>
vector<typename RoadmapGraph<VERTEX, WEIGHT>::VID> 
RoadmapGraph<VERTEX,WEIGHT>::
ConvertVertices2VIDs(vector<VERTEX>& vps){
  
  vector<VID> vds;
  #ifndef _PARALLEL
  vds.reserve(sizeof(vps));
  for(typename vector<VERTEX>::iterator itr = vps.begin(); itr != vps.end(); itr++)
	vds.push_back(this->GetVID(*itr));
  #endif
  return vds;
     
}

template <class VERTEX, class WEIGHT>
vector<VERTEX>
RoadmapGraph<VERTEX,WEIGHT>::
ConvertVIDs2Vertices(vector<VID>& vds){
  vector<VERTEX> vps;
  #ifndef _PARALLEL
  vps.reserve(sizeof(vds));
  for(typename vector<VID>::iterator itr = vds.begin(); itr!=vds.end(); itr++)
	vps.push_back((*(this->find_vertex(*itr))).property());
  #endif
  return vps;
}


template <class VERTEX, class WEIGHT>
int
RoadmapGraph<VERTEX,WEIGHT>::
GetVerticesData(vector<VERTEX>& v_vp) const{
  #ifndef _PARALLEL
  v_vp.clear();
  //verts.reserve( g.size() ); //g has no member size();
    for (CVI vi = (*this).begin(); vi!= (*this).end(); vi++) {
        v_vp.push_back((*vi).property());
    }
    #endif
    return v_vp.size();
}

template <class VERTEX, class WEIGHT>
int
RoadmapGraph<VERTEX,WEIGHT>::
GetVerticesVID(vector<VID>& v_vd) const{
  #ifndef _PARALLEL
  v_vd.clear();
  //verts.reserve( g.size() ); //g has no member size();
    for (CVI vi = (*this).begin(); vi!= (*this).end(); vi++) {
        v_vd.push_back((*vi).descriptor());
    }
   #endif
    return v_vd.size();
}


//lantao
template<class VERTEX, class WEIGHT>
typename RoadmapGraph<VERTEX, WEIGHT>::VID
RoadmapGraph<VERTEX,WEIGHT>::
GetVID(VERTEX& _v1) {
    #ifndef _PARALLEL
    CVI v1;
    if ( IsVertex(_v1,&v1) ) {
        //return v1->vid;
        return ((*v1).descriptor());
    } else {
        return INVALID_VID;
    }
    #else 
    cout << "WARNING::STAPL working on fixing problem with const iterators";
    return INVALID_VID;
    #endif
}

template<class VERTEX, class WEIGHT>
bool
RoadmapGraph<VERTEX,WEIGHT>::
IsVertex(VERTEX& _v1){
    #ifndef _PARALLEL ////implement parallel version as map reduce
    CVI v1;
    return ( IsVertex(_v1,&v1) );
    #else
   // cout << "WARNING::STAPL working on fixing problem with const iterators";
    return false;
    #endif
}

/*
//used as predicate of searching a vertex
template<class VERTEX,class WEIGHT>
class Vertex_equal_to{
public:
//typedef typename stapl::dynamic_graph<stapl::DIRECTED,stapl::NONMULTIEDGES,VERTEX,WEIGHT>::const_vertex_iterator CVI;
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

     VI vi = this->begin();
    bool found = false;
    while(vi != this->end() && !found){
	if((*vi).property() == _v1)
	  found = true;
	else
	  vi++;
    }// end while
    if (vi != this->end() ) {
        *_v1ptr = vi;
        //return true;
	found = true;
    } else {
        found = false;
    }
    return found;
}


template<class VERTEX, class WEIGHT>
bool
RoadmapGraph<VERTEX,WEIGHT>::
IsEdge(VID vid1, VID vid2){
bool is_edge=false;
#ifndef _PARALLEL //rm guard after find_edge is implemented for pgraph
typename RoadmapGraph<VERTEX, WEIGHT>::VI vi;
typename RoadmapGraph<VERTEX, WEIGHT>::EI ei;
typename RoadmapGraph<VERTEX, WEIGHT>::EID ed(vid1,vid2);
is_edge=this->find_edge(ed, vi, ei);
#endif
return is_edge;
}

template<class VERTEX, class WEIGHT>
bool
RoadmapGraph<VERTEX,WEIGHT>::
IsEdge(VERTEX& v1, VERTEX& v2){
VID vid1, vid2;
vid1= GetVID(v1);
vid2= GetVID(v2);
return IsEdge(vid1, vid2);
}

//helper function to call dereferece on an iterator whose
//value_type is VID and convert to CfgType
template<class VERTEX, class WEIGHT>
template<typename T>
typename RoadmapGraph<VERTEX, WEIGHT>::VP
RoadmapGraph<VERTEX, WEIGHT>::GetCfg(T& _t) {
  return (*(this->find_vertex(*_t))).property();
}

//specialization for a roadmap graph iterator, calls property()
//template<typename RDMP::VI>
template<class VERTEX, class WEIGHT>
typename RoadmapGraph<VERTEX, WEIGHT>::VP
RoadmapGraph<VERTEX, WEIGHT>::GetCfg(VI& _t) {
  return (*_t).property();
}

//specialization for a RoadmapGraph<CFG, WEIGHT>::VID
//calls find_vertex(..) on VID to call property()
template<class VERTEX, class WEIGHT>
typename RoadmapGraph<VERTEX, WEIGHT>::VP
RoadmapGraph<VERTEX, WEIGHT>::GetCfg(VID _t)  {
  return (*this->find_vertex(_t)).property();
}
//helper function to call dereferece on an iterator whose value_type is VID
//needed to get around the fact that a roadmap graph iterator 
//requires and extra descriptor() call
template<class VERTEX, class WEIGHT>
template<typename T>
typename RoadmapGraph<VERTEX, WEIGHT>::VID
RoadmapGraph<VERTEX, WEIGHT>::GetVid(T& _t) {
  return *_t;
}

//specialization for a roadmap graph iterator, calls descriptor()
//template<typename RDMP::VI>
template<class VERTEX, class WEIGHT>
typename RoadmapGraph<VERTEX, WEIGHT>::VID
RoadmapGraph<VERTEX, WEIGHT>::GetVid(VI& _t) {
  return (*_t).descriptor();
}

/*****************************************************************************/
//Dijkstra related.


//static bool dkinfo_ptr_compare (const dkinfo<VID>* d1, const dkinfo<VID>* d2)
#ifndef _PARALLEL
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
  stapl::sequential::vector_property_map<GRAPH,size_t> cmap;


  vector<VID> vec_cc;
  vector<dkinfo<VID> > pq; //The priority Queue
  double  maxdist = g.get_num_vertices() * (_w_.MaxWeight()).Weight();
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
          double wt = (*ei).property().Weight(); //g.GetEdgeWeight(u.vd,pq[j].vd).Weight(); fix_lantao

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
  stapl::sequential::vector_property_map<GRAPH,size_t> cmap;

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
    for (size_t i = 0; i < adj.size(); i++)
    {
      if(vid_dk_map.count(adj[i]) > 0) {
        dkinfo<VID>* succ= vid_dk_map[adj[i]];
        //double wt =g.GetEdgeWeight(u->vd,succ->vd).Weight(); fix_lantao
	typename GRAPH::edge_descriptor ed(u->vd, succ->vd);
        typename GRAPH::vertex_iterator vi;
        typename GRAPH::adj_edge_iterator ei;
        g.find_edge(ed, vi, ei);
       // double wt = (*ei).property().Weight();
       //to do - fix this 
        double wt = 1.0;
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

#endif

