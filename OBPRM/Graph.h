//this will be the new graph

/*******************
TO DO:
- clean the code and do some renamings
- see why crashes GetCC statistics(crashes for directed !!! Normal)
- write graph and read graph needs some standardization;!!!!!
*/

#include "BaseGraph.h"

#ifndef Graph_h
#define Graph_h


////////////////////////////////////////////////////////
//
// undirected / directed graph
///////////////////////////////////////////////////////

template<class VERTEX, class WEIGHT=int>
class UG: public  virtual BaseGraph<VERTEX,WEIGHT>{
  public:
  typedef WtVertexType<VERTEX,WEIGHT> Vertex;
  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;
  
  typedef typename BaseGraph<VERTEX,WEIGHT>::VERTEX_VECTOR VERTEX_VECTOR;
  
  typedef typename VERTEX_VECTOR::iterator VI;   ///<VI Vertex Iterator
  typedef typename VERTEX_VECTOR::const_iterator CVI;           ///<CVI Constant Vertex Iterator
  typedef typename VERTEX_VECTOR::reverse_iterator RVI;         ///<RVI Reverse Vertex Iterator
  typedef typename VERTEX_VECTOR::const_reverse_iterator CRVI;  ///<CRVI Constant Reverse Vertex Iterator
  
  typedef typename BaseGraph<VERTEX,WEIGHT>::WtEdge_VECTOR WtEdge_VECTOR;
  typedef typename WtEdge_VECTOR::iterator EI;                  ///<EI Edge Iterator
  typedef typename WtEdge_VECTOR::const_iterator CEI;           ///<CEI Constant Edge Iterator
  typedef typename WtEdge_VECTOR::reverse_iterator REI;         ///<REI Reverse Edge Iterator
  typedef typename WtEdge_VECTOR::const_reverse_iterator CREI;  ///<CREI Constant Reverse Edge Iterator
  inline UG(){
  }
  inline UG(int _sz): BaseGraph<VERTEX,WEIGHT> (_sz){
  }
  inline UG(int _sz,int _edgelistsz) : BaseGraph<VERTEX,WEIGHT> (_sz,_edgelistsz) {
  }
  ~UG(){}
  inline int check_directed() const {
    return 0;
  }

  inline int AddEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight) {
    //cout<<"UG::AddEdge"<<endl;
    //pair<WEIGHT,WEIGHT> p(_weight,_weight);
    //return BaseGraph<VERTEX,WEIGHT>::AddEdge(_v1,_v2,p);
    if(BaseGraph<VERTEX,WEIGHT>::AddEdge(_v1,_v2,_weight) == ERROR) return ERROR;
    return BaseGraph<VERTEX,WEIGHT>::AddEdge(_v2,_v1,_weight);
  }

  inline int AddEdge(VID _v1, VID _v2, WEIGHT _weight) {
    //cout<<"UG::AddEdge"<<endl;
    //pair<WEIGHT,WEIGHT> p(_weight,_weight);
    //return BaseGraph<VERTEX,WEIGHT>::AddEdge(_v1,_v2,p);
    if(BaseGraph<VERTEX,WEIGHT>::AddEdge(_v1,_v2,_weight)==ERROR) return ERROR;
    return BaseGraph<VERTEX,WEIGHT>::AddEdge(_v2,_v1,_weight);
  }

  int  AddEdge(VID _v1, VID _v2, pair<WEIGHT,WEIGHT>& _p){
    if(BaseGraph<VERTEX,WEIGHT>::AddEdge(_v1,_v2,_p.first)==ERROR) return ERROR;
    return BaseGraph<VERTEX,WEIGHT>::AddEdge(_v2,_v1,_p.second);
  }

  int  AddEdge(VERTEX& _v1, VERTEX& _v2, pair<WEIGHT,WEIGHT>& _p){
    if(BaseGraph<VERTEX,WEIGHT>::AddEdge(_v1,_v2,_p.first)==ERROR) return ERROR;
    return BaseGraph<VERTEX,WEIGHT>::AddEdge(_v2,_v1,_p.second);
  }

  int DeleteEdge(VID _v1id, VID _v2id, int _n=-1) {
    VI v1, v2;
    VI vi1;EI ei;
    if ( IsVertex(_v1id,&v1) && IsVertex(_v2id,&v2) && this->IsEdge(_v1id,_v2id,&vi1,&ei) ) {
      int ok1 = v1->DeleteXEdges(_v2id,-1);
      int ok2 = v2->DeleteXEdges(_v1id,-1);
      if ( ok1==1 && ok2==1 ) { //each should have found only 1 
	this->numEdges--;
	return OK;
      } 
    }
    return ERROR;
  }

  int DeleteWtEdge(VID _v1id, VID _v2id, WEIGHT _w, int _n=-1) {
    VI v1, v2;
    EI ei;VI cv1;
    if ( IsVertex(_v1id,&v1) && IsVertex(_v2id,&v2) &&this->IsEdge(_v1id,_v2id,_w,&cv1,&ei) ) {
      int ok1 = v1->DeleteXEdges(_v2id,-1);
      int ok2 = v2->DeleteXEdges(_v1id,-1);
      if ( ok1==1 && ok2==1 ) { //each should have found only 1 
	this->numEdges--;
	return OK;
      } 
    }
    return ERROR;
  }
  
  int DeleteEdge(VERTEX& _v1, VERTEX& _v2, int _n=-1) {
    VI v1, v2;
    VI vi1;
    EI ei;

    if ( IsVertex(_v1,&v1) && IsVertex(_v2,&v2) && this->IsEdge(_v1,_v2,&vi1,&ei) ) {
      int ok1 = v1->DeleteXEdges(v2->vid,-1);
      int ok2 = v2->DeleteXEdges(v1->vid,-1);
      if ( ok1==1 && ok2==1 ) { //each should have found only 1
	this->numEdges--; 
	return OK;
      }
    }
    return ERROR;
  }
  
  int
  DeleteWtEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _w, int _n=-1) {
    VI v1, v2;
    VI cv1;
    EI ei;
    if ( IsVertex(_v1,&v1) && IsVertex(_v2,&v2) && this->IsEdge(_v1,_v2,_w,&cv1,&ei) ) {
      int ok1 = v1->DeleteXEdges(v2->vid,-1);
      int ok2 = v2->DeleteXEdges(v1->vid,-1);
      if ( ok1==1 && ok2==1 ) { //each should have found only 1
	this->numEdges--; 
	return OK;
      }
    }
    return ERROR;
  }

  int GetVertexOutDegree(VID _v1) const {
    CVI v1;
    if ( IsVertex(_v1,&v1) ) {
      return v1->edgelist.size();
    } else {
      cout << "\nGetVertexDegree: vertex "<< v1->vid << " not in graph";
      return ERROR;
    }
  }
  
  int GetAdjacentVertices(VID _v1id, vector<VID>& _succ) const {
    CVI v1;    
    if ( IsVertex(_v1id,&v1) ) {
      _succ.clear();
      _succ.reserve( v1->edgelist.size() );
      for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
	_succ.push_back(ei->vertex2id);
      }
    } else {
      cout << "\nGetSuccessors: vertex "<< _v1id << " not in graph";
    }
    return _succ.size();
  }
  
  
  int GetDijkstraInfo(VID _v1id, vector<VID>& _succ) const {
  	return GetAdjacentVertices(_v1id,_succ);
  }
  int GetAdjacentVertices(VERTEX& _v1, vector<VID>& _succ) const {
    return GetAdjacentVertices( GetVID(_v1), _succ);
  }

  int GetAdjacentVerticesDATA(VID _v1id, vector<VERTEX>& _succ) const {
    CVI v1,v2;
    if ( IsVertex(_v1id,&v1) ) {
      _succ.clear();
      _succ.reserve( v1->edgelist.size() );
      for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
	if ( IsVertex(ei->vertex2id,&v2) )
	  _succ.push_back(v2->data);
      }
    } else {
      cout << "\nGetSuccessors: vertex "<< _v1id << " not in graph";
    }
    return _succ.size();
  }
  
  int GetAdjacentVerticesDATA(VERTEX& _v1, vector<VERTEX>& _succ) const {
    return GetAdjacentVerticesDATA( GetVID(_v1), _succ );
  }

  int GetIncidentEdges(VID _v1id,vector< pair<VID,VID> >& iedges) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
      iedges.clear();
      iedges.reserve( v1->edgelist.size() );
      for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
	pair<VID,VID> nextedge(_v1id,ei->vertex2id);
	iedges.push_back( nextedge );
      }
    } else {
      cout << "\nGetIncidentEdges: vertex "<< _v1id << " not in graph";
    }
    return iedges.size();
  }
  
  int GetIncidentEdgesVData(VID _v1id, vector< pair<VERTEX,VERTEX> >& iedges) const{
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
      iedges.clear();
      iedges.reserve( v1->edgelist.size() );
      for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
	pair<VERTEX,VERTEX> nextedge( this->GetData(_v1id), this->GetData(ei->vertex2id));
	iedges.push_back( nextedge );
      }
    } else {
      cout << "\nGetIncidentEdgesVData: vertex "<< _v1id << " not in graph";
    }
    return iedges.size();
  }

  int GetIncidentEdges(VID _v1id,vector< pair<pair<VID,VID>,WEIGHT> >& iedges) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
      iedges.clear();
      iedges.reserve( v1->edgelist.size() );
      for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
	pair<VID,VID> nextedge(_v1id,ei->vertex2id);
	pair<pair<VID,VID>,WEIGHT> nextedgewt(nextedge,ei->weight);
	iedges.push_back( nextedgewt );
      }
    } else {
      cout << "\nGetIncidentEdges: vertex "<< _v1id << " not in graph";
    }
    return iedges.size();
  }
  
  
  int GetIncidentEdgesVData(VID _v1id,vector< pair<pair<VERTEX,VERTEX>,WEIGHT> >& iedges) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
      iedges.clear();
      iedges.reserve( v1->edgelist.size() );
      for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
	pair<VERTEX,VERTEX> nextedge( this->GetData(_v1id), this->GetData(ei->vertex2id));
	pair<pair<VERTEX,VERTEX>,WEIGHT> nextedgewt(nextedge,ei->weight);
	iedges.push_back( nextedgewt );
      }
    } else {
      cout << "\nGetIncidentEdgesVData: vertex "<< _v1id << " not in graph";
    }
    return iedges.size();
  }

};

template<class VERTEX, class WEIGHT=int>
class DG: public  virtual BaseGraph<VERTEX,WEIGHT>{
  public:
  typedef WtVertexType<VERTEX,WEIGHT> Vertex;
  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;

  typedef typename BaseGraph<VERTEX,WEIGHT>::VERTEX_VECTOR VERTEX_VECTOR;
  typedef typename VERTEX_VECTOR::iterator VI;   ///<VI Vertex Iterator
  typedef typename VERTEX_VECTOR::const_iterator CVI;           ///<CVI Constant Vertex Iterator
  typedef typename VERTEX_VECTOR::reverse_iterator RVI;         ///<RVI Reverse Vertex Iterator
  typedef typename VERTEX_VECTOR::const_reverse_iterator CRVI;  ///<CRVI Constant Reverse Vertex Iterator

  typedef typename BaseGraph<VERTEX,WEIGHT>::WtEdge_VECTOR WtEdge_VECTOR;
  typedef typename WtEdge_VECTOR::iterator EI;                  ///<EI Edge Iterator
  typedef typename WtEdge_VECTOR::const_iterator CEI;           ///<CEI Constant Edge Iterator
  typedef typename WtEdge_VECTOR::reverse_iterator REI;         ///<REI Reverse Edge Iterator
  typedef typename WtEdge_VECTOR::const_reverse_iterator CREI;  ///<CREI Constant Reverse Edge Iterator

  inline DG(){
    //setDirectness(1);
  }
  inline DG(int _sz): BaseGraph<VERTEX,WEIGHT> (_sz){
    //setDirectness(0);
  }
  inline DG(int _sz,int _edgelistsz) : BaseGraph<VERTEX,WEIGHT> (_sz,_edgelistsz) {
    //setDirectness(0);
  }
  ~DG(){}
  inline int check_directed() const {
    return 1;  
  }

  //=======================================================
  //The following methods need to call SetPredecessors() first
  //to initialize predecessor vector
  //=======================================================

  void SetPredecessors() {
    VI v1, v2;
    VID _v2id;
    bool DoneSet = false;
    
    //check if SetPredecessors() already called
    for(v1 = this->v.begin(); v1 < this->v.end(); v1++) {
      if(!v1->predecessors.empty()) {
	DoneSet = true;
	cout<<"\nSetPredecessors() already called."<<endl;
	return;
	}
    }
    
    for(v1 = this->v.begin(); v1 < this->v.end(); v1++) {
      for (EI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
	_v2id = ei->vertex2id;
	if ( IsVertex(_v2id, &v2) ) {
	  WtEdge newEdge( v1->vid, ei->weight );
	  v2->predecessors.push_back(newEdge);
	}
      }
    }
  }

	  
  int GetVertexOutDegree(VID _v1) const {
    CVI v1;
    if ( IsVertex(_v1,&v1) ) {
      return v1->edgelist.size();
    } else {
      return ERROR;
    }
  }
  
  int GetSuccessors(VID _v1id, vector<VID>& _succ) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
      _succ.clear();
      _succ.reserve( v1->edgelist.size() );
      for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
	_succ.push_back(ei->vertex2id);
      }
    } else {
        cout << "\nGetSuccessors: vertex "<< _v1id << " not in graph";
    }
    return _succ.size();
  }
  
  int GetDijkstraInfo(VID _v1id, vector<VID>& _succ) const {
  	return GetSuccessors(_v1id,_succ);
  }

  int GetSuccessorsDATA(VID _v1id, vector<VERTEX>& _succ) const {
    CVI v1,v2;
    if ( IsVertex(_v1id,&v1) ) {
      _succ.clear();
      _succ.reserve( v1->edgelist.size() );
      for (CEI ei = v1->edgelist.begin(); ei != v1->edgelist.end(); ei++) {
	if ( IsVertex(ei->vertex2id,&v2) )
	  _succ.push_back(v2->data);
      }
    } else {
      cout << "\nGetSuccessors: vertex "<< _v1id << " not in graph";
    }
    return _succ.size();
  }

  int GetSuccessors(VERTEX& _v1, vector<VID>& _succ) const {
    return GetSuccessors( GetVID(_v1), _succ );
  }


  int GetSuccessorsDATA(VERTEX& _v1, vector<VERTEX>& _succ) const {
    return GetSuccessorsDATA( GetVID(_v1), _succ );
  }

  int GetPredecessors(VID _v1id, vector<VID>& _pred) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
      _pred.clear();
      _pred.reserve( v1->predecessors.size() );
      for (CEI ei = v1->predecessors.begin(); ei != v1->predecessors.end(); ei++) {
	_pred.push_back(ei->vertex2id);
      }
    } else {
      cout << "\nGetPredecessors: vertex "<< _v1id << " not in graph";
    }
    return _pred.size();
  }

  int GetPredecessorsDATA(VID _v1id,vector<VERTEX>& _pred) const {
    CVI v1,v2;
    if ( IsVertex(_v1id,&v1) ) {
      _pred.clear();
      _pred.reserve( v1->predecessors.size() );
      for (CEI ei = v1->predecessors.begin(); ei != v1->predecessors.end(); ei++) {
        if ( IsVertex(ei->vertex2id,&v2) )  
          _pred.push_back(v2->data);
      }
    } else {
      cout << "\nGetPredecessors: vertex "<< _v1id << " not in graph";
    }
    return _pred.size();
  }
  
  int GetPredecessors(VERTEX& _v1,vector<VID>& _pred) const {
    return GetPredecessors( GetVID(_v1), _pred );
  }
  
  int GetPredecessorsDATA(VERTEX& _v1,vector<VERTEX>& _pred) const {
    return GetPredecessorsDATA( GetVID(_v1), _pred );
  }
  
  int GetSources(vector<VID>& sourcevids) const {
    CVI v1;
    sourcevids.clear();
    for(v1=this->v.begin(); v1!=this->v.end(); v1++) {
      if(v1->predecessors.empty()) sourcevids.push_back(v1->vid);
    }
    return sourcevids.size();
  }

  
  int GetSinks(vector<VID>& sinkvids) const {
    CVI cv1;
    VI v1;
    sinkvids.clear();
    for(cv1=this->v.begin(); cv1!=this->v.end(); cv1++) {
      //        v1 = const_cast<VI> (cv1);
      if(cv1->edgelist.empty()) sinkvids.push_back(cv1->vid);
    }
    return sinkvids.size();//CHANGE
  }
};


////////////////////////////////////////////////////////
//
//  weighted/ unweighted graph
///////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT=int>
class WG: public virtual BaseGraph<VERTEX,WEIGHT>{

typedef WtVertexType<VERTEX,WEIGHT> Vertex;
typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;

typedef typename BaseGraph<VERTEX,WEIGHT>::VERTEX_VECTOR VERTEX_VECTOR;
typedef typename VERTEX_VECTOR::iterator VI;   ///<VI Vertex Iterator
typedef typename VERTEX_VECTOR::const_iterator CVI;           ///<CVI Constant Vertex Iterator
typedef typename VERTEX_VECTOR::reverse_iterator RVI;         ///<RVI Reverse Vertex Iterator
typedef typename VERTEX_VECTOR::const_reverse_iterator CRVI;  ///<CRVI Constant Reverse Vertex Iterator

typedef typename BaseGraph<VERTEX,WEIGHT>::WtEdge_VECTOR WtEdge_VECTOR;
typedef typename WtEdge_VECTOR::iterator EI;                  ///<EI Edge Iterator
typedef typename WtEdge_VECTOR::const_iterator CEI;           ///<CEI Constant Edge Iterator
typedef typename WtEdge_VECTOR::reverse_iterator REI;         ///<REI Reverse Edge Iterator
typedef typename WtEdge_VECTOR::const_reverse_iterator CREI;  ///<CREI Constant Reverse Edge Iterator
  public:
  inline int check_weighted() const {
    return 1;
  }


  bool IsEdge(VID _v1id, VID _v2id)  {  
   VI v1;
   EI e12;
   return (BaseGraph<VERTEX,WEIGHT>::IsEdge(_v1id,_v2id,&v1,&e12) );
 }  

  bool IsEdge(VERTEX& _v1, VERTEX& _v2)  {
    VI v1;
    EI e12;
    return (BaseGraph<VERTEX,WEIGHT>::IsEdge(_v1,_v2,&v1,&e12) );
  }

  bool IsEdge(VID _v1id, VID _v2id, WEIGHT _weight)  {
    VI v1;
    EI e12;
    return ( BaseGraph<VERTEX,WEIGHT>::IsEdge(_v1id,_v2id,_weight,&v1,&e12) );
  }
  bool IsEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight) {
    VI v1;
    EI e12;
    return ( BaseGraph<VERTEX,WEIGHT>::IsEdge(_v1,_v2,_weight,&v1,&e12) );
  }

  WEIGHT GetEdgeWeight(VID _v1id, VID _v2id) {
    VI v1;
    EI e12;
    if (BaseGraph<VERTEX,WEIGHT>::IsEdge(_v1id,_v2id,&v1,&e12)) {
      return  e12->weight;
    } else {
      return WEIGHT::InvalidWeight();
    }
  }

  WEIGHT GetEdgeWeight(VERTEX& _v1, VERTEX& _v2)  {
    return GetEdgeWeight( GetVID(_v1), GetVID(_v2) );
  }

  protected:
  
};

template<class VERTEX, class WEIGHT=int>
class NWG:public virtual BaseGraph<VERTEX,WEIGHT>{
typedef typename BaseGraph<VERTEX,WEIGHT>::VERTEX_VECTOR VERTEX_VECTOR;

typedef typename VERTEX_VECTOR::iterator VI;   ///<VI Vertex Iterator
typedef typename VERTEX_VECTOR::const_iterator CVI;           ///<CVI Constant Vertex Iterator
typedef typename VERTEX_VECTOR::reverse_iterator RVI;         ///<RVI Reverse Vertex Iterator
typedef typename VERTEX_VECTOR::const_reverse_iterator CRVI;  ///<CRVI Constant Reverse Vertex Iterator

typedef typename BaseGraph<VERTEX,WEIGHT>::WtEdge_VECTOR WtEdge_VECTOR;
typedef typename WtEdge_VECTOR::iterator EI;                  ///<EI Edge Iterator
typedef typename WtEdge_VECTOR::const_iterator CEI;           ///<CEI Constant Edge Iterator
typedef typename WtEdge_VECTOR::reverse_iterator REI;         ///<REI Reverse Edge Iterator
typedef typename WtEdge_VECTOR::const_reverse_iterator CREI;  ///<CREI Constant Reverse Edge Iterator

  public:
  inline int check_weighted() const {
    return 0;
  }

  bool IsEdge(VID _v1id, VID _v2id) {
    VI v1;
    EI e12;
    return (BaseGraph<VERTEX,WEIGHT>::IsEdge(_v1id,_v2id,&v1,&e12) );
  }

  bool IsEdge(VERTEX& _v1, VERTEX& _v2)  {
    VI v1;
    EI e12;
    return (BaseGraph<VERTEX,WEIGHT>::IsEdge(_v1,_v2,&v1,&e12) );
  }
};


////////////////////////////////////////////////////////
//
// multi/ non multi graph
///////////////////////////////////////////////////////
template<class VERTEX, class WEIGHT=int>
class MG: public virtual BaseGraph<VERTEX,WEIGHT>{
  public:
  inline int check_multi() const {
    return 1;
  }
  inline int check_edge(VID v1, VID v2) const {
    //cout<<"M::check_edge"<<endl;
    return 1;
  }
  inline int check_edge(VERTEX& v1, VERTEX& v2) const {
    //cout<<"M::check_edge"<<endl;
    return 1;
  }
};


template<class VERTEX, class WEIGHT=int>
class NMG: public virtual BaseGraph<VERTEX,WEIGHT>{
  public:
  typedef typename BaseGraph<VERTEX,WEIGHT>::VERTEX_VECTOR VERTEX_VECTOR;
  typedef typename VERTEX_VECTOR::iterator VI;   ///<VI Vertex Iterator
  typedef typename VERTEX_VECTOR::const_iterator CVI;           ///<CVI Constant Vertex Iterator
  typedef typename BaseGraph<VERTEX,WEIGHT>::WtEdge_VECTOR WtEdge_VECTOR;
  typedef typename WtEdge_VECTOR::iterator EI;                  ///<EI Edge Iterator
  typedef typename WtEdge_VECTOR::const_iterator CEI;           ///<CEI Constant Edge Iterator

  inline int check_multi() const{
    return 0;
  }

  inline int check_edge(VID _v1, VID _v2){
    //cout<<"NM::check_edge"<<endl;
    VI v1;
    EI e12;
    if(_v1 == _v2) return 0;
    if(BaseGraph<VERTEX,WEIGHT>::IsEdge(_v1, _v2, &v1,&e12)) return 0;
    else return 1;
  }
  inline int check_edge(VERTEX& _v1, VERTEX& _v2){
    //cout<<"NM::check_edge"<<endl;
    VI v1;
    EI e12;
    VI vi1,vi2;
    if(!IsVertex(_v1,&vi1)) return 0;
    if(!IsVertex(_v2,&vi2)) return 0;
    if(vi1->vid == vi2->vid) return 0;
    if(BaseGraph<VERTEX,WEIGHT>::IsEdge(_v1,_v2,&v1,&e12)) return 0;
    else return 1;
  }
};


////////////////////////////////////////////////////////
//
//    the graph class
///////////////////////////////////////////////////////
template <class D, class M, class W, class VERTEX,class WEIGHT=int>
class Graph:public D, public M, public W{
  public:

  typedef WtVertexType<VERTEX,WEIGHT> Vertex;
  typedef WtEdgeType<VERTEX,WEIGHT> WtEdge;
  
  typedef  VERTEX VERTEX_TYPE;
  typedef  WEIGHT WEIGHT_TYPE ;
  
  typedef typename D::VERTEX_VECTOR VERTEX_VECTOR;
  
  typedef typename VERTEX_VECTOR::iterator VI;   ///<VI Vertex Iterator
  typedef typename VERTEX_VECTOR::const_iterator CVI;           ///<CVI Constant Vertex Iterator
  typedef typename VERTEX_VECTOR::reverse_iterator RVI;         ///<RVI Reverse Vertex Iterator
  typedef typename VERTEX_VECTOR::const_reverse_iterator CRVI;  ///<CRVI Constant Reverse Vertex Iterator
  
  typedef typename D::WtEdge_VECTOR WtEdge_VECTOR;
  typedef typename WtEdge_VECTOR::iterator EI;                  ///<EI Edge Iterator
  typedef typename WtEdge_VECTOR::const_iterator CEI;           ///<CEI Constant Edge Iterator
  typedef typename WtEdge_VECTOR::reverse_iterator REI;         ///<REI Reverse Edge Iterator
  typedef typename WtEdge_VECTOR::const_reverse_iterator CREI;  ///<CREI Constant Reverse Edge Iterator

  Graph(){}
  Graph(int _sz) : D(_sz){}
  Graph(int _sz,int _edgelistsz) : D(_sz,_edgelistsz){}
  ~Graph(){}
  //the addVertex methods will be inherited from weighted multi di graph
  //the addEdge methods have to be modified to chech for multiplicity/ weight 


  VI begin(){
    return this->v.begin();
  }

  VI end(){
    return this->v.end();
  }

  inline int getVertIDs(){
    return this->vertIDs;
  }
  inline void setVertIDs(int _vids){
    this->vertIDs = _vids;
  }

  inline int IsDirected() const {
    return D::check_directed();
  }
  inline int IsWeighted() const {
    return W::check_weighted();
  }
  inline int IsMulti() const {
    return M::check_multi();
  }

  inline int size() const {
    return this->v.size();
  }
  inline int GetEdgeCount() const{
    if(D::check_directed()) return this->numEdges;
    else return this->numEdges/2;
  }
  inline int AddEdge(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight=WEIGHT(-1)) {
    //cout<<"Graph::AddEdge"<<endl;
    if(M::check_edge(_v1,_v2)) return D::AddEdge(_v1,_v2,_weight);
    else return ERROR;
  }
  inline int AddEdge(VID _v1, VID _v2, WEIGHT _weight=WEIGHT(-1)) {
    //cout<<"Graph::AddEdge"<<endl;
    //cout<<"directness "<<D::isDirected()<<endl;
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
    if(M::check_edge(_v1,_v2)) return D::AddEdge(_v1,_v2,_weight);
    //return D::AddEdge(_v1,_v2,_weight);
    else return ERROR;
  }
  int  AddEdge(VID _v1, VID _v2, pair<WEIGHT,WEIGHT>& _p){
    //if(AddEdge(_v1,_v2,p.first) == -1) return -1;
    //return AddEdge(_v2,_v1,p.second); 
    //!!!!! the correct way of doing this ???????????????????????????
    if(M::check_edge(_v1,_v2)&&M::check_edge(_v2,_v1)) return D::AddEdge(_v1,_v2,_p);
    //else {cout<<"ERROR"<<endl;return ERROR;}
    else return ERROR;
  }
  int  AddEdge(VERTEX& _v1, VERTEX& _v2, pair<WEIGHT,WEIGHT>& _p){
    //cout<<D::isDirected()<<endl;
    //if(AddEdge(_v1,_v2,p.first) == -1) return -1;
    //return AddEdge(_v2,_v1,p.second); 
    if(M::check_edge(_v1,_v2)&&M::check_edge(_v2,_v1)) return D::AddEdge(_v1,_v2,_p);
    //else {cout<<"ERROR"<<endl;return ERROR;}
    else return ERROR;
  }

  int AddPath( vector<VID>& _path, WEIGHT _wt=WEIGHT(-1)) {
    int i;
    int cnt=0;
    for (i = 0; i < _path.size(); i++){
        if (!this->IsVertex(_path[i])) return ERROR;
    }
    for (i = 0; i < _path.size() - 1; i++){
	if(AddEdge(_path[i],_path[i+1],_wt)!=ERROR) cnt++;
    }
    return cnt;
  }

  int AddPath( vector<VERTEX>& _path, WEIGHT _wt=WEIGHT(-1)) {
    int i;
    int cnt;
    if (!this->IsVertex(_path[0])) AddVertex(_path[0]);
    for (i = 0; i < _path.size() - 1; i++){
      if (!this->IsVertex(_path[i+1])) AddVertex(_path[i+1]);
      if(AddEdge(_path[i],_path[i+1],_wt) != ERROR) cnt++;
    }
    return cnt;
  }

  //the following AddPath methods are only for weighted graph;

  int AddPath( vector< pair<VID,WEIGHT> >& _path) {
    int i;
    int cnt = 0;
    for (i = 0; i < _path.size(); i++){
        if (!this->IsVertex(_path[i].first)) return ERROR;
    }
    for (i = 0; i < _path.size() - 1; i++){
        if(AddEdge(_path[i].first,_path[i+1].first,_path[i].second)!=ERROR) cnt++;
    }
    return cnt;
}


  int AddPath( vector< pair<VERTEX,WEIGHT> >& _path) {
    int i;
    int cnt;
    if (!IsVertex(_path[0].first)) AddVertex(_path[0].first);
    for (i = 0; i < _path.size() - 1; i++){
      if (!this->IsVertex(_path[i+1].first)) AddVertex(_path[i+1].first);
      if(AddEdge(_path[i].first,_path[i+1].first,_path[i].second) != ERROR) cnt++;
    }
    return OK;
  }
  
  int AddPath( vector< pair<VID, pair<WEIGHT,WEIGHT> > >& _path) {
    int i;
    int cnt=0;
    for (i = 0; i < _path.size(); i++){
        if (!this->IsVertex(_path[i].first)) return ERROR;
    }
    for (i = 0; i < _path.size() - 1; i++){
        if(AddEdge(_path[i].first,_path[i+1].first,_path[i].second)!=ERROR) cnt++;
    }
    return cnt;
  }
  
  int AddPath( vector< pair<VERTEX, pair<WEIGHT,WEIGHT> > >& _path) {
    int i;
    int cnt=0;
    if (!this->IsVertex(_path[0].first)) AddVertex(_path[0].first);
    for (i = 0; i < _path.size() - 1; i++){
      if (!this->IsVertex(_path[i+1].first)) AddVertex(_path[i+1].first);
      if(AddEdge(_path[i].first,_path[i+1].first,_path[i].second)!=ERROR) cnt++;
    }
    return cnt;
  }

  
  int
  GetEdges(vector< pair<VID,VID> >& edges) const {
    //vector< pair<VID,VID> > edges;    
    edges.clear();
    edges.reserve(GetEdgeCount());
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
	if(!(IsDirected()) && 
	   (vi->vid > ei->vertex2id)) continue;
	pair<VID,VID> newedge(vi->vid, ei->vertex2id);
	edges.push_back( newedge );
      }
    }
    return edges.size();
  }

  int
  GetEdgesVData(vector< pair<VERTEX,VERTEX> >& edges) const {
    //vector< pair<VERTEX,VERTEX> > edges; 
    edges.clear();
    edges.reserve(GetEdgeCount());
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
	if(!(IsDirected()) && 
	   (vi->vid > ei->vertex2id)) continue;
	VERTEX v2data = GetData(ei->vertex2id);
	pair<VERTEX,VERTEX> newedge(vi->data, v2data);
	edges.push_back( newedge );
      }
    }
    return edges.size();
  }

  int
  GetEdges(vector< pair< pair<VID,VID>, WEIGHT> >& edges) const  {
    //vector< pair< pair<VID,VID>, WEIGHT> > edges;    
    edges.clear();
    edges.reserve(GetEdgeCount());
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
	if(!(IsDirected()) && 
	   (vi->vid > ei->vertex2id)) continue;
	pair<VID,VID> newedge(vi->vid, ei->vertex2id);
	pair<pair<VID,VID>,WEIGHT> newedgewt(newedge, ei->weight);
	edges.push_back( newedgewt );
      }
    }
    return edges.size();
  }

  int
  GetEdgesVData(vector< pair< pair<VERTEX,VERTEX>, WEIGHT> >& edges) const  {
    //vector< pair< pair<VERTEX,VERTEX>, WEIGHT> > edges; 
    edges.clear();
    edges.reserve(GetEdgeCount());
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
	if(!(IsDirected()) && 
	   (vi->vid > ei->vertex2id)) continue;
	VERTEX v2data = GetData(ei->vertex2id);
	pair<VERTEX,VERTEX> newedge(vi->data, v2data);
	pair<pair<VERTEX,VERTEX>,WEIGHT> newedgewt(newedge, ei->weight);
	edges.push_back( newedgewt );
      }
    }
    return edges.size();
  }

  int
  ChangeEdgeWeight(VID _v1id, VID _v2id, WEIGHT _weight) {
    //this is only for weighted one;
    //something such that the compiler will generate errors
    //or return -1 immediate
    //and I think this can be done faster; and moved in weighted class!!!
    if(!W::check_weighted()) return ERROR;
    if ( D::DeleteEdge(_v1id, _v2id) == OK) {
      return AddEdge(_v1id, _v2id,_weight);
    } else {
      return ERROR;
    }  
  }
  int
  ChangeEdgeWeight(VERTEX& _v1, VERTEX& _v2, WEIGHT _weight) {
    if(!W::check_weighted()) return ERROR;
    if ( DeleteEdge(_v1, _v2) == OK) {
        return AddEdge(_v1, _v2,_weight);
    } else {
      return ERROR;
    }  
  } 

  //IO stuff
  void 
  ReadDotGraph(const char*  _fname) {
    
    ifstream  myifstream(_fname);
    if (!myifstream) {
      cout << "\nIn ReadGraph: can't open infile: " << _fname ;
      return;
    }
    ReadDotGraph(myifstream);
    myifstream.close();
  }

  void 
  ReadDotGraph(istream& _myistream) {
    VID v1id, v2id, maxVID;
    CVI  cv1;
    //VI  v1;
    VERTEX data;
    WEIGHT weight = WEIGHT(1);
    int nVerts=0, nEdges=0;
    char tagstring[100];
    _myistream >> tagstring;
    if ( !strstr(tagstring,"GRAPHSTART") ) {
        cout << endl << "In ReadGraph: didn't read GRAPHSTART tag right";
        return;
    }
    
    if (this->numVerts != 0) {
        D::EraseGraph(); // empty graph before filling it in
    }
    data.SetTaskWeight(1);
    _myistream >> v1id;
    while(v1id != -1){
      //data.SetTaskWeight((v1id*1000)+(random()%100));
      data.SetTaskWeight(v1id);
      AddVertex(data,v1id);
      nVerts++;
      if ( !IsVertex(v1id,&cv1) ) {
	cout << "\nIn ReadGraph: didn't add v1...";
	//return -1;
      }
      _myistream >> v1id;
    }

    //weight.SetEdgeWeight(1);
    _myistream >> v1id;
    while(v1id != -1){

      _myistream >> v2id;
      // weight.SetEdgeWeight((v2id*1000)+random()%100);
      //weight.SetEdgeWeight(1);
      if(AddEdge(v1id,v2id,weight) < 0) {
	cout<<"Error while trying to insert edge "<<v1id<<" "<<v2id<<endl;
	//return -1;
      }
      nEdges++;
      _myistream >> v1id;
    }
    maxVID = nVerts;
    this->numVerts = nVerts;

    if(this->IsDirected()) this->numEdges = nEdges;
    else this->numEdges = nEdges*2;
    //this->numEdges = nEdges;
    this->vertIDs = maxVID; // set the maximum VID used so far...
    _myistream >> tagstring;
    if ( !strstr(tagstring,"GRAPHSTOP") ) {
      cout << endl << "In ReadGraph: didn't read GRAPHSTOP tag right";
      return;
    }
  }
  // end gti
 
  void DisplayGraph() const {
    CVI vi;
    int i;
    cout<<endl;
    for (vi = this->v.begin(), i=0; vi < this->v.end(); vi++, i++) {
      cout << setw(3) << i << ": ";
      vi->DisplayEdgelist(W::check_weighted());
      cout<<endl;
    }
  }

  void DisplayVertices() const {
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      DisplayVertex(vi->vid); 
    } 
  }

  void DisplayVertex(VID _v1id) const{
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
      cout << "vertex: id =" << setw(3) << _v1id; 
      cout << ", data = [" << v1->data;
      cout << "]";
      cout<< endl;
    } else {
      cout << "vertex with id=" << _v1id << " not in graph.";
    }
  }

  void DisplayVertexAndEdgelist(VID _v1id) const {
    CVI v1;
    if ( IsVertex(_v1id,&v1) ) {
      cout << "vertex: ";
      v1->DisplayEdgelist(W::check_weighted());
    } else {
      cout << "vertex with id=" << _v1id << " not in graph.";
    }
  }
  
  
  void WriteGraph(ostream& _myostream) const {
  
#ifdef _ASCI_
    _myostream << endl << "GRAPHSTART";
#else
    _myostream << endl << "#####GRAPHSTART#####";
#endif
    if(this->IsDirected())
    _myostream << endl << this->numVerts << " " << this->numEdges << " " << this->vertIDs; 
    else 
    _myostream << endl << this->numVerts << " " << this->numEdges / 2 << " " << this->vertIDs; 
 
    //format: VID VERTEX #edges VID WEIGHT VID WEIGHT ... 
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      _myostream << endl;
      vi->WriteEdgelist(_myostream,W::check_weighted());
    } 
    
#ifdef _ASCI_
    _myostream << endl << "GRAPHSTOP";
#else
    _myostream << endl << "#####GRAPHSTOP#####";
#endif
    _myostream << endl; 
  }

  void WriteGraph(const char* _fname) const {
    
    ofstream  myofstream(_fname);
    if (!myofstream) {
      cout << "\nInWriteGraph: can't open outfile: " << _fname ; 
    }
    WriteGraph(myofstream);
    myofstream.close();
  }

  
  void ReadGraph(istream& _myistream) {
    VID v1id, v2id, maxVID;
    VI  v1;
    VERTEX data;
    WEIGHT weight;
    int nVerts, nEdges, nedges;
    char tagstring[100];
    
    _myistream >> tagstring;
    if ( !strstr(tagstring,"GRAPHSTART") ) {
      cout << endl << "In ReadGraph: didn't read GRAPHSTART tag right";
      return;
    }
    
    if (this->numVerts != 0) {
      this->EraseGraph(); // empty graph before filling it in
    }
    
    _myistream >> nVerts >> nEdges >> maxVID;
    
    for (int i = 0; i < nVerts; i++){
      _myistream >> v1id >> data;             // read and add vertex 
      AddVertex(data,v1id);
      if ( !IsVertex(v1id,&v1) ) {
	cout << "\nIn ReadGraph: didn't add v1...";
      }
      
      _myistream >> nedges;               // read and add its edges
      for (int j = 0; j < nedges; j++){
	if(W::check_weighted())
	  _myistream >> v2id >> weight; 
	else _myistream >> v2id;//read only the id and put a default weight	  
	v1->AddEdge(v2id,weight);
      }
    }
    
    this->numVerts = nVerts;
    //internally we keep trace to all the edges in the graph;
    //which for undirected is double
    if(this->IsDirected()) this->numEdges = nEdges;
    else this->numEdges = nEdges*2;
    this->vertIDs = maxVID; // set the maximum VID used so far...
    // should sort verts & find biggest used...
    
    _myistream >> tagstring;
    if ( !strstr(tagstring,"GRAPHSTOP") ) {
      cout << endl << "In ReadGraph: didn't read GRAPHSTOP tag right";
      return;
    }
  }

  void ReadGraph(const char*  _fname) {
    ifstream  myifstream(_fname);
    if (!myifstream) {
      cout << "\nIn ReadGraph: can't open infile: " << _fname ;
      return;
    }
    ReadGraph(myifstream);
    myifstream.close();
  }

  
  void ReadGraphwithAutoVID(istream& _myistream) {
    //this is not tested ?
    VID v1id, v2id, maxVID;
    VI  v1;
    VERTEX data;
    WEIGHT weight;
    int nVerts, nEdges, nedges;
    char tagstring[100];
    
    _myistream >> tagstring;
    if ( !strstr(tagstring,"GRAPHSTART") ) {
      cout << endl << "In ReadGraph: didn't read GRAPHSTART tag right";
      return;
    }
    
    if (this->numVerts != 0) {
      this->EraseGraph(); // empty graph before filling it in
    }
    
    _myistream >> nVerts >> nEdges >> maxVID;
    
    for (int i = 0; i < nVerts; i++){
      _myistream >> data;             // read and add vertex 
      AddVertex(data);
      v1id = i;       //start vid from 0
      if ( !IsVertex(v1id,&v1) ) {
	cout << "\nIn ReadGraph: didn't add v1...";
      }
      
      _myistream >> nedges;               // read and add its edges
      for (int j = 0; j < nedges; j++){
	_myistream >> v2id >> weight; 
	v1->AddEdge(v2id,weight);
      }
    }
    
    this->numVerts = nVerts;

    if(this->IsDirected()) this->numEdges = nEdges;
    else this->numEdges = nEdges*2;

    this->vertIDs = maxVID; // set the maximum VID used so far...
    // should sort verts & find biggest used...
    
    _myistream >> tagstring;
    if ( !strstr(tagstring,"GRAPHSTOP") ) {
      cout << endl << "In ReadGraph: didn't read GRAPHSTOP tag right";
      return;
    }
  }
  void ReadGraphwithAutoVID(const char*  _fname) {

    ifstream  myifstream(_fname);
    if (!myifstream) {
      cout << "\nIn ReadGraph: can't open infile: " << _fname ;
      return;
    }
    ReadGraphwithAutoVID(myifstream);
    myifstream.close();
  }

};//end class GRAPH  
#endif

/*
1)
virtual vector< pair<pair<VERTEX,VERTEX>, WEIGHT> > GetEdgesVData() const;
	this function has to be implemented different in W/ NW classes
	vector< <pair<VERTEX,VERTEX>> > GetEdgesVData() const; for NW class
2)
WEIGHT  GetEdgeWeight(VID, VID) const;	
3)
       	virtual vector<VID> GetPredecessors(VID) const;
		(also for succesors)
	vector<VID> GetSinks() const;
       	vector<VID> GetSources() const;

this methods should be only for directed graph
Undirected graph should not have this methods;
Undirected should have getEdges(VID) instead;
*/
/*  
  vector< pair<VID,VID> >
  GetEdges()  {
    vector< pair<VID,VID> > edges;    
    edges.reserve(this->numEdges);
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
	if (vi->vid < ei->vertex2id){
	  pair<VID,VID> newedge(vi->vid, ei->vertex2id);
	  edges.push_back( newedge );
	}
      }
    }
    return edges;
  } 

  vector< pair<VERTEX,VERTEX> >
  GetEdgesVData() const {
    vector< pair<VERTEX,VERTEX> > edges;
    edges.reserve(this->numEdges);
    for (CVI vi = this->v.begin(); vi != this->v.end(); vi++) {
      for (CEI ei = vi->edgelist.begin(); ei != vi->edgelist.end(); ei++ ) {
	if ( vi->vid < ei->vertex2id) {
	  VERTEX v2data = GetData(ei->vertex2id);
	  pair<VERTEX,VERTEX> newedge(vi->data, v2data);
	  edges.push_back( newedge );
	}
      }
    }
    return edges;
  }
*/
