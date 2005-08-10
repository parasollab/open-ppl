#ifndef ClosestVE_h
#define ClosestVE_h
#include "NodeConnectionMethod.h"


   /**Connect nodes in map to their k closest neighbors, which
    *could be vertex or point on edges in roadmap.
    *
    *This method not only creates edges, but creates new verteices 
    *also. New vertices are always on the existing edges.
    *
    *@param info provides inforamtion other than connection, like
    *collision dection, local planner, and distance metrics.
    *@param _cn provides information for specific node connection 
    *paramters.
    *@param lp Local planner for connecting given 2 Cfgs.
    *
    *@see ClosestVE for more information.
    */

/**Cfg VE Type.
  *@todo Not well documented. I don't know what this class for.
  */
template <class CFG>
class CfgVEType {
  public:
  
  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Constructors and Destructor*/
  //@{

  /**Default Constructor.
   *   -# #cfg1 = Cfg::InvalidData
   *   -# #cfg2 = Cfg::InvalidData
   *   -# #cfg2_IsOnEdge = false
   */
  CfgVEType();
  
  /**Constructor.
   *   -# #cfg1 = _cfg1
   *   -# #cfg2 = _cfg2
   *   -# #cfg2_IsOnEdge = false
   */
  CfgVEType(CFG& _cfg1, CFG& _cfg2);
  
  /**Constructor.
   *   -# #cfg1 = _cfg1
   *   -# #cfg2 = _cfg2
   *   -# #cfg2_IsOnEdge = true
   *   -# #endpt = (_endpt1,_endpt2)
   */
  CfgVEType(CFG& _cfg1, CFG& _cfg2, CFG& _endpt1, CFG& _endpt2);
  
  ///Destructor. Do nothing.
  ~CfgVEType();
  
  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  CFG         cfg1, ///< Start of an edge
              cfg2; ///< End of an edge
  ///True if cfg2 is on line segment defined by #endpt.
  bool        cfg2_IsOnEdge;
  ///Defines a line segment where cfg2 is. (Defined only if #cfg2_IsOnEdge is true.)
  vector<CFG> endpt;

}; //End of CfgVEType


#define KCLOSESTVE 5 


template <class CFG, class WEIGHT>
class ClosestVE: public NodeConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  ClosestVE();
  ~ClosestVE();
 
  //////////////////////
  // Access
  void SetDefault();

  //////////////////////
  // I/O methods

  void ParseCommandLine(std::istringstream& is);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os); 
  virtual NodeConnectionMethod<CFG, WEIGHT>* CreateCopy();
  //////////////////////
  // Core: Connection method
  static bool VE_DIST_Compare(const pair<CfgVEType<CFG>,double> _cc1, const pair<CfgVEType<CFG>,double> _cc2);
  /**Find k pairs of closest Cfgs from a given Cfg to Cfgs in a Cfg vector
   *or on the edges in edge vector.
   *
   *This method check distances from given Cfg to every Cfg in verts
   *and distances from given Cfg to every edge in edges.
   *
   *The first k cloeset pair will be returned among all pairs.
   *
   *Following is short Alg for calculating result list:
   *   -# for every Cfg, c1, in Cfg vector
   *       -# if distance(cfg,c1)< largest distance in return list.
   *           -# then replace largest distance by this path (cfg->c1)
   *           -# sort return list.
   *       -# end if
   *   -# end for
   *   -# for every edge, e1, in edge vector
   *       -# if distance(cfg,e1)< largest distance in return list.
   *           -# then let e1_c be closest point on e1 to cfg.
   *           -# replace largest distance by this path (cfg->e1_c)
   *           -# sort return list.
   *   -# end for
   *
   *@param verts A list of Cfgs.
   *@param edges A list of edge with eight.
   *@param cfg Distances are calculated from this cfg.
   *@param k Find k pairs.
   *@param midpoint Not used currently.
   *
   *@return A (k-elemet) list of CfgVEType.
   *@see VE_DIST_Compare for sorting.
   */
  vector<CfgVEType<CFG> > FindKClosestPairs(Roadmap<CFG, WEIGHT>*, 
					      DistanceMetric*,  
					      CFG cfg, vector<CFG>& verts,
					      vector<pair<pair<VID,VID>,WEIGHT> >& edges, 
					      int k, bool midpoint);

  //@}
  /**For each Cfg in newV, find the k closest Cfgs
   *(which could be Cfgs in oldV and/or points on edges of roadmap)
   *and try to connect with them.
   *
   *Algorithm:
   *   -# Get all edges in roadmap
   *   -# for every Cfg, c1, in newV
   *       -# Find k closest Cfgs (which could be Cfgs in oldV 
   *          and/or points on edges of roadmap) for c1
   *       -# for each Cfg, c2, in k-closest Cfgs
   *           -# if c1 and c2 are conncteced.
   *               -# add new edge to roadmap
   *           -# end if
   *       -# end for
   *   -# end for
   *
   *@note it is possilbe that c2 in algorithm is nor in roadmap, (i.e.
   *created from edges), c2 will be added to roadmap and relavant edges
   *will be added too.
   *
   *@param newV contains Cfgs. This method tries to connect these Cfgs to those
   *contained in oldV.
   *@param oldV contains Cfgs to be connected.
   *@see RoadmapGraph::GetEdges
   */
  //@}
  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
			 CollisionDetection*, 
			 DistanceMetric *,
			 LocalPlanners<CFG,WEIGHT>*,
			 bool addPartialEdge=false,
			 bool addAllEdges=false);

  void Connect(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
	       CollisionDetection* cd, DistanceMetric* dm,
	       LocalPlanners<CFG,WEIGHT>* lp,
	       bool addPartialEdge, bool addAllEdges,
	       vector<CFG>& cfgs1, vector<CFG>& cfgs2);

 private:
  //////////////////////
  // Data
  int tag;                            ///< Nodes can be marked by user
  int dupeNodes,dupeEdges;            ///< used for acct'ing w/ closestVE
  int kclosest;
};


template <class CFG, class WEIGHT>
ClosestVE<CFG,WEIGHT>::ClosestVE():NodeConnectionMethod<CFG,WEIGHT>() { 
  element_name = "closestVE"; 

  SetDefault();
}


template <class CFG, class WEIGHT>
ClosestVE<CFG,WEIGHT>::~ClosestVE() { 
}


template <class CFG, class WEIGHT>
void ClosestVE<CFG,WEIGHT>::
ParseCommandLine(std::istringstream& is) {
  char c;
  SetDefault();
  try {
    c = is.peek();
    while (c == ' ' || c == '\n') {
      is.get();
      c = is.peek();
    }
    if (c >= '0' && c <= '9') {
      if (is >> kclosest) {
        if (kclosest < 0)
  	  throw BadUsage();
      } else
        throw BadUsage();
    }

  } catch (BadUsage) {
    cerr << "Error in \'closestVE\' parameters" << endl;    
    PrintUsage(cerr);
    exit(-1);
  }
}


template <class CFG, class WEIGHT>
void ClosestVE<CFG,WEIGHT>::SetDefault() {
  kclosest = KCLOSESTVE;
  tag = -999;                            ///< Nodes can be marked by user
  dupeNodes = 0;
  dupeEdges = 0; 
}


template <class CFG, class WEIGHT>
void
ClosestVE<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\tINTEGER (default " << KCLOSESTVE << ")";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
ClosestVE<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " kclosest = ";
  _os << kclosest;
  _os << endl;
}


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>* 
ClosestVE<CFG,WEIGHT>::
CreateCopy() {
  NodeConnectionMethod<CFG,WEIGHT>* _copy = 
           new ClosestVE<CFG,WEIGHT>(*this);
  return _copy;
}


//
// used to "sort" by distance
//
template <class CFG, class WEIGHT>
bool
ClosestVE<CFG, WEIGHT>::
VE_DIST_Compare (const pair<CfgVEType<CFG>,double> _cc1, const pair<CfgVEType<CFG>,double> _cc2) {
  return (_cc1.second < _cc2.second ) ;
}


//----------------------------------------------------------------------
// Given: k, ONE Cfg and ONE vector of vertices and ONE vector of edges
// Find : find k pairs of closest cfg from "cfg" to "vector"
//----------------------------------------------------------------------
template <class CFG, class WEIGHT>
vector<CfgVEType<CFG> > 
ClosestVE<CFG, WEIGHT>::
FindKClosestPairs(Roadmap<CFG, WEIGHT>* _rm,
		  DistanceMetric* dm, 
		  CFG cfg, vector<CFG>& verts, 
		  vector< pair<pair<VID,VID>,WEIGHT> >& edges, 
		  int k, bool midpoint) {  
  vector<CfgVEType<CFG> > pairs;
  pair<CfgVEType<CFG>,double> tmp2;
  
  // if valid number of pairs requested
  if (k<=0) return pairs;
  
  // initialize w/ k elements each with huge distance...
  vector<pair<CfgVEType<CFG>,double> > kp;
  for (int i=0; i < k; i++) {
    tmp2.first = CfgVEType<CFG>();
    tmp2.second = MAX_DIST;
    kp.push_back(tmp2);
  }
  
  //-- VERTICES:
  //   Note: need to keep the distances so can't just call one of 
  //         the other versions of vertices compatable FindKClosestPairs
  
  // now go through all kp and find closest k
  for (int c1 = 0; c1 < verts.size(); c1++) {
    if (cfg != verts[c1] ) { 
      double dist = dm->Distance(_rm->GetEnvironment(), cfg, verts[c1]);
      if ( dist < kp[k-1].second) {
	tmp2.first = CfgVEType<CFG>(cfg,verts[c1]);
	tmp2.second = dist;
	kp[k-1] = tmp2;
	sort (kp.begin(), kp.end(), ptr_fun(VE_DIST_Compare) );
      }
    }// if (cfg != verts[c1])
  }//endfor c1
  
  //-- EDGES:
  for (int e1 = 0; e1 < edges.size(); e1++) {    
    CFG endpt1 = _rm->m_pRoadmap->GetData(edges[e1].first.first);
    CFG endpt2 = _rm->m_pRoadmap->GetData(edges[e1].first.second);
    CFG tmp;
    tmp.ClosestPtOnLineSegment(cfg,endpt1,endpt2);
    
    if (tmp != endpt1 && tmp != endpt2){
      double dist = dm->Distance(_rm->GetEnvironment(), cfg, tmp);
      
      if ( dist < kp[k-1].second) {
	tmp2.first = CfgVEType<CFG>(cfg, tmp, endpt1, endpt2);
	tmp2.second = dist;
	kp[k-1] = tmp2;
	sort (kp.begin(), kp.end(), ptr_fun(VE_DIST_Compare) );
      } //endif dist
      
    } // endif (tmp != endpt1 && tmp != endpt2)
  } //endfor e1
  
  // now construct vector of k pairs to return (don't need distances...)
  CFG invalid;
  invalid.InvalidData();
  for (int p=0; p < k && p<kp.size(); p++)
    if (kp[p].first.cfg1 != invalid && 
	kp[p].first.cfg2 != invalid)
      pairs.push_back( kp[p].first );
  
  return pairs;
}

 
// ------------------------------------------------------------------
// ClosestVE:
//
// For each cfg find the k closest CFG or PT_on_EDGE 
// and try to connect with them.
//
// ------------------------------------------------------------------
template <class CFG, class WEIGHT>
void ClosestVE<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
		  CollisionDetection* cd , 
		  DistanceMetric * dm,
		  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge,
		  bool addAllEdges) {

#ifndef QUIET
  cout << "closestVE(k="<< kclosest <<"): "<<flush;
#endif
  if (lp->UsesPlannerOtherThan("straightline")){
    cout <<"\n\nWARNING: Skipping call to ClosestVE."
	 <<  "\n         'straightline' ONLY local planner "
	 <<"for which ClosestVE works.\n\n";
    return;
  }
  
  vector<CFG> oldV,newV,verts;
  _rm->m_pRoadmap->GetVerticesData(verts);
  // if separation of vertices into two sets is desired
  if (tag != -1) {
    // separate on tag values
    for( int iV=0;iV<verts.size();++iV ) {
      if(verts[iV].tag == tag) 
	newV.push_back(verts[iV]);
      else                         
	oldV.push_back(verts[iV]);
    }
  }
  // only one set desired
  else { oldV = newV = verts; }

  // Get edges	 
  // reserve extra space ...will update local copy for efficiency
  vector< pair<pair<VID,VID>,WEIGHT> > edges;
  edges.reserve(_rm->m_pRoadmap->GetEdgeCount() + newV.size()*kclosest);
  _rm->m_pRoadmap->GetEdges(edges);
  
  // May have to adjust user's desired k wrt what is actually possible
  int k = min(kclosest, oldV.size()+edges.size());
  
  
  ///Modified for VC
#if defined(_WIN32)
  using namespace std;
#endif
  
  // for each "real" cfg in roadmap
  LPOutput<CFG,WEIGHT> lpOutput;
  for (typename vector<CFG>::iterator v=newV.begin();v<newV.end();++v) {
    // Find k closest cfgs in the roadmap
    bool midpt_approx_of_closestPt = false;
    vector<CfgVEType<CFG> > KP = FindKClosestPairs(_rm, dm, 
						   *v, oldV, edges,
						   k, midpt_approx_of_closestPt);
    // for each pair identified	
    for (typename vector<CfgVEType<CFG> >::iterator kp=KP.begin();kp<KP.end();++kp){
      
#if CHECKIFSAMECC
      if(IsSameCC(*(_rm->m_pRoadmap),kp->cfg1,kp->cfg2)) continue;
#endif
      //-- if new edge is collision free
      if (!_rm->m_pRoadmap->IsEdge(kp->cfg1,kp->cfg2) 
          && lp->IsConnected(_rm->GetEnvironment(),Stats, cd,dm, 
			     kp->cfg1,kp->cfg2,
	  	             &lpOutput,
			     connectionPosRes, connectionOriRes, 
			     (!addAllEdges) )) {
	//-- may have to add a new node in "middle" of existing edge
	if ( kp->cfg2_IsOnEdge ) {
	  _rm->m_pRoadmap->AddVertex(kp->cfg2);
	  
	  ++dupeNodes;  // keep count of duplicated nodes
	}//endif cfg2_IsOnEdge
	
	//-- add new edge to map
	_rm->m_pRoadmap->AddEdge(kp->cfg1, kp->cfg2, lpOutput.edge);
	
	//-- if did add an explicit interior node to edge(endpt0,endpt1)
	if ( kp->cfg2_IsOnEdge ) {
	  _rm->m_pRoadmap->AddEdge(kp->endpt[0],kp->cfg2,lpOutput.edge);
	  _rm->m_pRoadmap->AddEdge(kp->cfg2,kp->endpt[1],lpOutput.edge);
	  
	  dupeEdges += 2;  // keep count of duplicated edges <--for what??
	}//endif cfg2_IsOnEdge
      } //endif lp->IsConnected
    } //endfor kp
  } //endfor v

}


template <class CFG, class WEIGHT>
void ClosestVE<CFG,WEIGHT>::
Connect(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
	       CollisionDetection* cd, DistanceMetric* dm,
	       LocalPlanners<CFG,WEIGHT>* lp,
	       bool addPartialEdge, bool addAllEdges,
	       vector<CFG>& cfgs1, vector<CFG>& cfgs2){


  // Left empty because I didn't know how to implement it...

}

// ------------------------------------------------------------------
// CfgVEType is a private class used by ConnectMapNodes class
// to store information about connecting to what may either be
// another Cfg _OR_ a new Cfg generated in the "middle" of an existing
// edge in the map.  
//
// In order to keep the same structure (for sorting by distance) and 
// yet not allocate space for endpoints when they are not needed
// (ie, connecting to another existing map node--aka,Cfg), endpoints
// are stored as a vector.
// ------------------------------------------------------------------
template <class CFG>
CfgVEType<CFG>::
~CfgVEType() {
}


template <class CFG>
CfgVEType<CFG>::
CfgVEType() {
  CFG invalid;
  invalid.InvalidData();
  cfg1 = invalid;
  cfg2 = invalid;
  cfg2_IsOnEdge = false;
}


template <class CFG>
CfgVEType<CFG>::
CfgVEType(CFG& _cfg1, CFG& _cfg2){
  cfg1 = _cfg1;
  cfg2 = _cfg2;
  cfg2_IsOnEdge = false;
}


template <class CFG>
CfgVEType<CFG>::
CfgVEType(CFG& _cfg1, CFG& _cfg2, CFG& _endpt1, CFG& _endpt2){
  cfg1   = _cfg1;
  cfg2   = _cfg2;
  cfg2_IsOnEdge = true;
  endpt.push_back(_endpt1);
  endpt.push_back(_endpt2);
}


#endif
