// $Id$
/////////////////////////////////////////////////////////////////////
//
//  ConnectMapNodes.c
//
//  General Description
//     This set of classes supports a "Connection Strategy Algobase".
//     This file contains the definitions of the prototypes
//     declared in "ConnectMapNodes.h".
//
//  Created
//      8/27/98  Daniel Vallejo 
//
/////////////////////////////////////////////////////////////////////

#include "ConnectMapNodes.h"

#include "Cfg.h"
#include "DistanceMetrics.h"
#include "Environment.h"
#include "Roadmap.h"
#include "Clock_Class.h"
#include "util.h"

#define K_OTHER      10        // default for obst-other connections
#define K_SELF        3        // default for obst-self  connections
#define STEP_FACTOR   3        // default for rrt stepFactor
#define ITERATIONS   10        // default for rrt iterations
#define MAX_NUM      20        // default for modifiedLM maxNum
#define RFACTOR       2        // default for modifiedLM 'radius' factor

double ConnectMapNodes::connectionPosRes = 0.05;
double ConnectMapNodes::connectionOriRes = 0.05;

/////////////////////////////////////////////////////////////////////
//
//  METHODS for class ConnectMapNodes
//      
/////////////////////////////////////////////////////////////////////

ConnectMapNodes::
ConnectMapNodes(){
   DefaultInit(); 
}


ConnectMapNodes::
~ConnectMapNodes(){
}


void 
ConnectMapNodes::
DefaultInit(){
    //-----------------------------------------------
    // The default algorithms are specified
    //-----------------------------------------------
    cnInfo.cnsetid = CLOSEST10;
    cnInfo.lpsetid = SL_R5;
    cnInfo.dmsetid = S_EUCLID9;

#if defined USE_CSTK
    cnInfo.cdsetid = CSTK;
#elif defined USE_RAPID
    cnInfo.cdsetid = RAPID;
#elif defined USE_PQP
    cnInfo.cdsetid = PQP;
#elif defined USE_VCLIP
    cnInfo.cdsetid = VCLIP;
#else
    #error You have to specify at least one collision detection library.
#endif

    cnInfo.dupeEdges = cnInfo.dupeNodes = 0;
    cnInfo.gn.gnInfo.gnsetid = -1;
}


void
ConnectMapNodes::
UserInit(Input * input, Environment * env){
   //-----------------------------------------------
   // User Initialize Node Connectors
   //-----------------------------------------------

   // Initialize cn Sets
   connectors.MakeCNSet("random");           // enum RANDOM
   connectors.MakeCNSet("closest 10");       // enum CLOSEST10
   connectors.MakeCNSet("closest 20");       // enum CLOSEST20

   if ( input->numCNs == 0 ) { //use default CN sets
   } else {
     cnInfo.cnsetid=CN_USER1;
     for (int i = 0; i < input->numCNs; i++) {
       connectors.MakeCNSet(input->CNstrings[i]->GetValue());
     }
   }

   cnInfo.addPartialEdge = input->addPartialEdge.GetValue();

   connectionPosRes = env->GetPositionRes();
   connectionOriRes = env->GetOrientationRes();
}


//------------------------------------------------------
// Driver,driver
// Connect nodes according to all CN's in the set
//------------------------------------------------------
void
ConnectMapNodes::
ConnectNodes(Roadmap * rdmp,
             CollisionDetection *cd,LocalPlanners* lp, DistanceMetric * dm, 
             SID _cnsetid, CNInfo &info) {

  vector<CN> cnset = connectors.GetCNSet(_cnsetid);

  for (int cn=0; cn < cnset.size(); ++cn) {

       // newly generated edges to the roadmap go here
       info.edges.erase(info.edges.begin(),info.edges.end());

       #ifndef QUIET
       Clock_Class clock;
       clock.StartClock(cnset[cn].GetName());
       cout<<"\n ";
       clock.PrintName();
       cout << flush;
       #endif

       CNF cnfcn = cnset[cn].GetConnector();
       cnfcn(rdmp,cd,lp,dm,cnset[cn], info);

       // add newly generated edges to the roadmap
       rdmp->m_pRoadmap->AddEdges(info.edges);
       // move this part to the inside of each function.

       #ifndef QUIET
       clock.StopClock();
       cout << clock.GetClock_SEC() << " sec, "
		<< rdmp->m_pRoadmap->GetCCcount() 
		<< " connected components\n"<< flush;
       #endif
  }
}


//------------------------------------------------------
// Another Driver,driver
//
// ConnectNodes
//------------------------------------------------------
void
ConnectMapNodes::
ConnectNodes(Environment * environment, RoadmapGraph<Cfg,WEIGHT>& roadmap,
             CollisionDetection *cd,LocalPlanners* lp,
             DistanceMetric * dm, SID _cnsetid, CNInfo &info) {
  Roadmap rdmp;
  rdmp.environment = environment;
  rdmp.m_pRoadmap = &roadmap;

  ConnectNodes(&rdmp, cd, lp, dm, _cnsetid, info);
  roadmap = *rdmp.m_pRoadmap;
}

// ------------------------------------------------------------------
// set connection resolutions, since it might be different from global values.
// ------------------------------------------------------------------
void ConnectMapNodes::setConnectionResolution(double _posRes, double _oriRes) {
    if(_posRes > 0) // make sure it is positive.
	connectionPosRes = _posRes;
    if(_oriRes > 0)
	connectionOriRes = _oriRes;
}


// ------------------------------------------------------------------
// some info needs to be set up for check connection between cfg's
// ------------------------------------------------------------------
LPInfo
ConnectMapNodes::
Initialize_LPinfo(Roadmap * _rm,CNInfo& info){

  LPInfo lpInfo(_rm,info);

  lpInfo.positionRes    = connectionPosRes;
  lpInfo.orientationRes = connectionOriRes;
  lpInfo.checkCollision = true;
  lpInfo.savePath       = false;
  lpInfo.cdsetid        = info.cdsetid;
  lpInfo.dmsetid        = info.dmsetid;

  return lpInfo;
}


// ------------------------------------------------------------------
// Connect nodes in a random way.
// ------------------------------------------------------------------
void
ConnectMapNodes::
ConnectNodes_Random(Roadmap * _rm,
					CollisionDetection* cd,
					LocalPlanners* lp,
					DistanceMetric * dm,
					CN& _cn, CNInfo& info)
{

    //-- initialize information needed to check connection between cfg's
    LPInfo lpInfo=Initialize_LPinfo(_rm,info);

    vector<Cfg> vertices = _rm->m_pRoadmap->GetVerticesData();
    vector<pair<SID,vector<LP> > > sets = lp->planners.GetLPSets();

    for (int i=0; i < _cn.GetNumEdges(); i++) {

      	int c1id = (int)(lrand48()%_rm->m_pRoadmap->GetVertexCount());
      	int c2id = (int)(lrand48()%_rm->m_pRoadmap->GetVertexCount());
      	Cfg c1 = vertices[c1id];
      	Cfg c2 = vertices[c2id];

      	int thisset = (int)(lrand48()%sets.size());
      	SID setid = sets[thisset].first;

      	if ( lp->IsConnected(_rm,cd,dm,c1,c2,setid,&lpInfo) ) {
	    _rm->m_pRoadmap->AddEdge(c1id, c2id, lpInfo.edge);
      	}
     }
}

// ------------------------------------------------------------------
// ConnectNodes_Closest: 
//
// For each cfg find the k closest CFG 
// and try to connect with them.
//
// ------------------------------------------------------------------
void
ConnectMapNodes::
ConnectNodes_Closest(Roadmap * _rm,CollisionDetection* cd,
					 LocalPlanners* lp,DistanceMetric * dm,
					 CN& _cn, CNInfo& info)
{
#ifndef QUIET
	cout << "(k="<<_cn.GetKClosest()<<"): "<<flush;
#endif
	
	vector<Cfg> vertices = _rm->m_pRoadmap->GetVerticesData();
	const int verticeSize = vertices.size();
	const int k = min(_cn.GetKClosest(),verticeSize);
	
	ConnectNodes_Closest(_rm, cd, lp, dm, _cn, info, vertices, vertices, k);
}

void
ConnectMapNodes::
ConnectNodes_Closest(Roadmap * _rm,CollisionDetection* cd,
					 LocalPlanners* lp,DistanceMetric * dm,
					 CN& _cn, CNInfo& info, vector<Cfg>& vec1, 
					 vector<Cfg>& vec2, const int kclosest)
{
	//-- initialize information needed to check connection between cfg's
	LPInfo lpInfo=Initialize_LPinfo(_rm,info);
	RoadmapGraph<Cfg, WEIGHT> * pMap = _rm->m_pRoadmap;

	vector< pair<VID,VID> > kp;
	// Find k closest cfgs to each cfg in the roadmap
	if(kclosest < vec2.size() - 1) {
		kp = FindKClosestPairs(_rm, dm, info, vec1, vec2, kclosest);
	} 
	else { // all the pairs
		for(int i=0; i<vec1.size(); ++i)
			for(int j=0; j<vec2.size(); ++j){
				if( vec1[i]==vec2[j] ) continue;
				kp.push_back(pair<VID,VID>(pMap->GetVID(vec1[i]), pMap->GetVID(vec2[j])));
			}
	}

	// for each pair identified
	for (int j=0; j < kp.size(); j++) {
		if( _rm->m_pRoadmap->IsEdge(kp[j].first, kp[j].second)) continue;
#if CHECKIFSAMECC
		if(_rm->m_pRoadmap->IsSameCC(kp[j].first,kp[j].second)) continue;
#endif
		if (lp->IsConnected(_rm,cd,dm,
			_rm->m_pRoadmap->GetData(kp[j].first),
			_rm->m_pRoadmap->GetData(kp[j].second),
			info.lpsetid,&lpInfo))
			_rm->m_pRoadmap->AddEdge(kp[j].first, kp[j].second, lpInfo.edge);
	} //endfor j
}

// ------------------------------------------------------------------
// Cfg_VE_Type is a private class used by ConnectMapNodes class
// to store information about connecting to what may either be
// another Cfg _OR_ a new Cfg generated in the "middle" of an existing
// edge in the map.  
//
// In order to keep the same structure (for sorting by distance) and 
// yet not allocate space for endpoints when they are not needed
// (ie, connecting to another existing map node--aka,Cfg), endpoints
// are stored as a vector.
// ------------------------------------------------------------------
Cfg_VE_Type::~Cfg_VE_Type(){
}
Cfg_VE_Type::
Cfg_VE_Type(){
        cfg1 = Cfg::InvalidData();
        cfg2 = Cfg::InvalidData();
        cfg2_IsOnEdge = false;
}
Cfg_VE_Type::
Cfg_VE_Type(Cfg& _cfg1,Cfg& _cfg2){
        cfg1 = _cfg1;
        cfg2 = _cfg2;
        cfg2_IsOnEdge = false;
}
Cfg_VE_Type::
Cfg_VE_Type(Cfg& _cfg1,Cfg& _cfg2,  Cfg& _endpt1,Cfg& _endpt2){
        cfg1   = _cfg1;
        cfg2   = _cfg2;
        cfg2_IsOnEdge = true;
        endpt.push_back(_endpt1);
        endpt.push_back(_endpt2);
}
// ------------------------------------------------------------------
// ClosestVE:
//
// For each cfg find the k closest CFG or PT_on_EDGE 
// and try to connect with them.
//
// ------------------------------------------------------------------
void
ConnectMapNodes::
ConnectNodes_ClosestVE(
        Roadmap * _rm,CollisionDetection* cd,
        LocalPlanners* lp,DistanceMetric * dm,
        CN& _cn, CNInfo& info){

#ifndef QUIET
	cout << "(k="<<_cn.GetKClosest()<<"): "<<flush;
#endif
	
	if (lp->UsesPlannerOtherThan("straightline",info.lpsetid)){
        cout <<"\n\nWARNING: Skipping call to ClosestVE."
			<<  "\n         'straightline' ONLY local planner "
			<<"for which ClosestVE works.\n\n";
        return;
	}
	
	vector<Cfg> oldV,newV,verts= _rm->m_pRoadmap->GetVerticesData();
	// if separation of vertices into two sets is desired
	if (info.tag != InfoCfg::NULL_INFO){
        // separate on tag values
		for( int iV=0;iV<verts.size();++iV ){
			if (verts[iV].info.tag == info.tag) newV.push_back(verts[iV]);
			else                         oldV.push_back(verts[iV]);
		}
	}
	// only one set desired
	else { oldV = newV = verts; }
	ClosestVE(_rm,cd,lp,dm,_cn,info,oldV,newV);
}

// ------------------------------------------------------------------
// ClosestVE:
//
// For each cfg find the k closest CFG or PT_on_EDGE 
// and try to connect with them.
//
// ------------------------------------------------------------------
void
ConnectMapNodes::
ClosestVE(
        Roadmap * _rm,CollisionDetection* cd,
        LocalPlanners* lp,DistanceMetric * dm,
        CN& _cn, CNInfo& info, vector<Cfg>& oldV, vector<Cfg>& newV){

  //-- initialize information needed to check connection between cfg's
  LPInfo lpInfo=Initialize_LPinfo(_rm,info);

  // Get edges 
  // reserve extra space ...will update local copy for efficiency
  vector< pair<pair<VID,VID>,WEIGHT> > edges;
  edges.reserve(_rm->m_pRoadmap->GetEdgeCount() + newV.size()*_cn.GetKClosest());
  edges = _rm->m_pRoadmap->GetEdges();

  // May have to adjust user's desired k wrt what is actually possible
  int k = min(_cn.GetKClosest(), oldV.size()+edges.size());

  
///Modified for VC
#if defined(_WIN32)
	using namespace std;
#endif

  // for each "real" cfg in roadmap
  for (vector<Cfg>::iterator v=newV.begin();v<newV.end();++v){

      // Find k closest cfgs in the roadmap
      bool midpt_approx_of_closestPt = false;
      vector<Cfg_VE_Type> KP = FindKClosestPairs(_rm,dm,info,
                                                 *v,oldV,edges,
                                                 k, midpt_approx_of_closestPt);
      // for each pair identified
      for (vector<Cfg_VE_Type>::iterator kp=KP.begin();kp<KP.end();++kp){

         #if CHECKIFSAMECC
         if(_rm->m_pRoadmap->IsSameCC(kp->cfg1,kp->cfg2)) continue;
         #endif

         //-- if new edge is collision free
         if (lp->IsConnected(_rm,cd,dm,
                         kp->cfg1,kp->cfg2,
                         info.lpsetid,&lpInfo)){

              //-- may have to add a new node in "middle" of existing edge
              if ( kp->cfg2_IsOnEdge ) {

                _rm->m_pRoadmap->AddVertex(kp->cfg2);

                ++info.dupeNodes;  // keep count of duplicated nodes

              }//endif cfg2_IsOnEdge

              //-- add new edge to map
              _rm->m_pRoadmap->AddEdge(kp->cfg1, kp->cfg2, lpInfo.edge);

              //-- if did add an explicit interior node to edge(endpt0,endpt1)
              if ( kp->cfg2_IsOnEdge ) {
                        _rm->m_pRoadmap->AddEdge(kp->endpt[0],kp->cfg2,lpInfo.edge);
                        _rm->m_pRoadmap->AddEdge(kp->cfg2,kp->endpt[1],lpInfo.edge);

                        info.dupeEdges += 2;  // keep count of duplicated edges <--for what??
              }//endif cfg2_IsOnEdge
         } //endif lp->IsConnected
      } //endfor kp
  } //endfor v
}

// ------------------------------------------------------------------
// ConnectNodes_ConnectCCs: 
//
// Try to connect different connected components of the m_pRoadmap->
// We try to connect all pairs of connected components. If both
// components are small (less than "smallcc" nodes), then we try to 
// connect all pairs of nodes.  If at least one of the components is 
// large, we try to connect the "kpairs" closest pairs of nodes.
// ------------------------------------------------------------------
void ConnectMapNodes::ConnectNodes_ConnectCCs
(Roadmap * _rm, CollisionDetection* cd, LocalPlanners* lp, DistanceMetric * dm,
 CN& _cn, CNInfo& info)
{	
	vector< pair<int,VID> > allCCs = _rm->m_pRoadmap->GetCCStats();
	ConnectNodes_ConnectCCs(_rm, cd, lp, dm, _cn, info, allCCs, allCCs);
}

void ConnectMapNodes::ConnectNodes_ConnectCCs
(Roadmap * _rm, CollisionDetection* cd, LocalPlanners* lp, DistanceMetric * dm,
 CN& _cn, CNInfo& info, vector< pair<int,VID> > & ccs1, vector< pair<int,VID> > ccs2)
{
	// a user doesn't need to provide another lp set.(If 
	// the user does provide, it should be used). 
	if(info.addPartialEdge && lp->planners.GetLPSets().size()-1 > info.lpsetid)
		++info.lpsetid;

#ifndef QUIET
	cout << "(kpairs=" << _cn.GetKPairs();
	cout << ", smallcc=" << _cn.GetSmallCCSize() << "): "<<flush;
#endif

	int smallcc = _cn.GetSmallCCSize();

	// process components from smallest to biggest  
	for (int cc1 = ccs1.size()-1 ; cc1 >= 0 ; cc1--) {
		for (int cc2 = ccs2.size() - 2 ; cc2 >= 0 ; cc2--){

			VID cc1id = ccs1[cc1].second;
			VID cc2id = ccs2[cc2].second;
			
			// if cc1 & cc2 not already connected, try to connect them 
			if ( !_rm->m_pRoadmap->IsSameCC(cc1id,cc2id) ) {
				if(_rm->m_pRoadmap->GetCC(cc1id).size() < smallcc &&
					_rm->m_pRoadmap->GetCC(cc2id).size() < smallcc ) {
					ConnectSmallCCs(_rm,cd,lp,dm,_cn,info,cc1id,cc2id);
				} else {
					ConnectBigCCs(_rm,cd,lp,dm,_cn,info,cc1id,cc2id);
				}
			} 
		}/*endfor cc2*/ 
		ccs2.pop_back();
	}/*endfor cc1*/
}

//
// try to connect all pairs of cfgs in the two CCs
//
void
ConnectMapNodes::
ConnectSmallCCs(Roadmap* _rm,CollisionDetection *cd, LocalPlanners* lp,DistanceMetric * dm,CN& _cn, CNInfo& info, VID _cc1id, VID _cc2id) {

  //-- initialize information needed to check connection between cfg's
  LPInfo lpInfo=Initialize_LPinfo(_rm,info);

  // created a temporary variable since GetCC requires &

  Cfg t;
  t=_rm->m_pRoadmap->GetData(_cc1id);
  vector<Cfg> cc1vec = _rm->m_pRoadmap->GetCC(t);
  t=_rm->m_pRoadmap->GetData(_cc2id);
  vector<Cfg> cc2vec = _rm->m_pRoadmap->GetCC(t);
  bool connected = false;

  for (int c1 = 0; c1 < cc1vec.size(); c1++) {
    for (int c2 = 0; c2 < cc2vec.size(); c2++) {

       if (lp->IsConnected(_rm,cd,dm,
			cc1vec[c1],cc2vec[c2],info.lpsetid,&lpInfo)) {
	   _rm->m_pRoadmap->AddEdge(cc1vec[c1], cc2vec[c2], lpInfo.edge);
           connected = true;
           break;
       } else if(info.addPartialEdge) {
	   Cfg &tmp = lpInfo.savedEdge.second;
	   if(!tmp.AlmostEqual(cc1vec[c1])) {
	      _rm->m_pRoadmap->AddVertex(tmp);
	      _rm->m_pRoadmap->AddEdge(cc1vec[c1], tmp, lpInfo.edge);
	   }
       }
    }

    if (connected) break;
  }

}

//
// try to connect kclosest pairs of cfgs in the two CCs
//
void
ConnectMapNodes::
ConnectBigCCs(
        Roadmap* _rm,CollisionDetection *cd, 
        LocalPlanners* lp,DistanceMetric * dm,
        CN& _cn, CNInfo& info, 
        VID _cc1id, VID _cc2id) {

  //-- initialize information needed to check connection between cfg's
  LPInfo lpInfo=Initialize_LPinfo(_rm,info);

  int kpairs = _cn.GetKPairs();
  
  Cfg t;
  t=_rm->m_pRoadmap->GetData(_cc1id);
  vector<Cfg> cc1vec = _rm->m_pRoadmap->GetCC(t);
  t=_rm->m_pRoadmap->GetData(_cc2id);
  vector<Cfg> cc2vec = _rm->m_pRoadmap->GetCC(t);

  kpairs = min(kpairs, cc2vec.size());

  vector< pair<Cfg,Cfg> > kp = FindKClosestPairs(_rm->GetEnvironment(),
	dm,info,cc1vec,cc2vec,kpairs);

  for (int i = 0; i < kp.size(); i++) {
      if (lp->IsConnected(_rm,cd,dm,kp[i].first,
		kp[i].second,info.lpsetid,&lpInfo)) {
	   _rm->m_pRoadmap->AddEdge(kp[i].first, kp[i].second, lpInfo.edge); 
           break;
      } else if(info.addPartialEdge) {
           Cfg &tmp = lpInfo.savedEdge.second;
	   if(!tmp.AlmostEqual(kp[i].first)) {
             _rm->m_pRoadmap->AddVertex(tmp);
             _rm->m_pRoadmap->AddEdge(kp[i].first, tmp, lpInfo.edge);
	   }
      }
  }
}

//
// used to "sort" Cfg's by obst generation number
//
bool
ConnectMapNodes::
info_Compare (const Cfg &_cc1, const Cfg &_cc2) {
        return (_cc1.info.obst < _cc2.info.obst ) ;
}

//
// used to sort cfgs by distance (CfgDistType is single cfg & distance)
//
bool
ConnectMapNodes::
CfgDist_Compare (const CfgDistType &_cc1, const CfgDistType &_cc2) {
        return (_cc1.second < _cc2.second ) ;
}

//
// used by "findKClosestPairs" for sort (DIST_TYPE is pair of cfgs & distance)
//
bool
ConnectMapNodes::
DIST_Compare (const DIST_TYPE &_cc1, const DIST_TYPE &_cc2) {
        return (_cc1.second < _cc2.second ) ;
}

//
// used to "sort" by distance
//
bool
ConnectMapNodes::
VE_DIST_Compare (const VE_DIST_TYPE &_cc1, const VE_DIST_TYPE &_cc2) {
        return (_cc1.second < _cc2.second ) ;
}

//----------------------------------------------------------------------
// SortByDistFromCfg
//
// Given: ONE cfg1  and ONE vector of cfgs
// Do:    sort cfgs by distance from cfg1 (modifies cfgs)
//----------------------------------------------------------------------
void
ConnectMapNodes::
SortByDistFromCfg(Environment *_env,DistanceMetric * dm, CNInfo& info,
   const Cfg& _cfg1, vector<Cfg>&  _cfgs) {

   // compute distances from _cfg1 for all cfgs in _cfgs
   vector<CfgDistType> distances;
   for (int i=0; i < _cfgs.size(); i++) {
        double dist = dm->Distance(_env, _cfg1,_cfgs[i],info.dmsetid);
        distances.push_back(CfgDistType(_cfgs[i],dist));
   }

   sort (distances.begin(), distances.end(), ptr_fun(CfgDist_Compare) );

   // now reconstruct _cfgs vector, sorted by distances 
   for (int j=0; j < _cfgs.size(); j++) 
        _cfgs[j] = distances[j].first;
  
   return;

}


//----------------------------------------------------------------------
// Given: k and ONE cfg and ONE vector
// Find : find k pairs of closest cfg from "cfg" to "vector"
//----------------------------------------------------------------------
vector< CfgPairType >
ConnectMapNodes::
FindKClosestPairs(Environment *_env,DistanceMetric * dm, CNInfo& info,
        Cfg& cfg1,vector<Cfg>& vec1, int k) {

  vector<Cfg> cfg;
  cfg.push_back(cfg1);

  // find kclosest cfgs to cfg in vertices
  return FindKClosestPairs( _env, dm, info, cfg, vec1, k);
}

//----------------------------------------------------------------------
// Given: k and ONE vector
// Find : find k pairs of closest cfg from vector to each cfg in vector
//----------------------------------------------------------------------
vector< CfgPairType >
ConnectMapNodes::
FindKClosestPairs(Environment *_env,DistanceMetric * dm, CNInfo& info,
        vector<Cfg>& vec1, int k) {
	
	vector< pair<Cfg,Cfg> > pairs;
	// if valid number of pairs requested
	if (k<=0) return pairs;

    vector<Cfg> vec_of_cfgs = vec1;

	//for each cfg in given vector
	for( int iV1=vec1.size()-1; iV1>1; iV1-- ){
		// find k closest cfgs between the two vectors 
		vector< pair<Cfg,Cfg> > kp = FindKClosestPairs(_env,dm,info,vec1[iV1],vec_of_cfgs,k);
		// save pairs
		pairs.insert(pairs.end(),kp.begin(),kp.end());
    }//endfor cfg

  return pairs;
}

//----------------------------------------------------------------------
// Given: k and a vector
// Find : k pairs of closest cfgs for each cfg in vector to all other cfgs in vector.
// -- if k don't exist, return as many as do
//----------------------------------------------------------------------
vector< pair<VID, VID> >
ConnectMapNodes::
FindKClosestPairs(Roadmap *rm, DistanceMetric * dm, CNInfo& info,
        vector<Cfg>& vec1, int k) {

	return FindKClosestPairs(rm, dm, info, vec1, vec1, k);
}

//----------------------------------------------------------------------
// Given: k and a TWO vectors
// Find : k pairs of closest cfgs for each cfg in vec1 to all cfgs in vec2.
//		  This means there will be k*n pairs returned. n in number of cfgs in 
//        vec1. k pair for each cfg in vec1.
//		  The differences between this function and FindKClosestPairs
//        (Environment *,DistanceMetric * , CNInfo& info, vector<Cfg>& , vector<Cfg>& , int )
//		  are type and size of return values.
// -- if k don't exist, return as many as do
//----------------------------------------------------------------------
vector< pair<VID, VID> > ConnectMapNodes::FindKClosestPairs
(Roadmap *rm,DistanceMetric * dm, CNInfo& info,vector<Cfg>& vec1,vector<Cfg>& vec2, int k)
{
	vector< pair<VID, VID> > pairs;
	if (k<=0) return pairs;

	Environment *_env = rm->GetEnvironment();
	RoadmapGraph<Cfg, WEIGHT> * pMap = rm->m_pRoadmap;

	//compute from the last to the second (ignore the first)
	for( int iV1=vec1.size()-1; iV1>0; iV1-- ){
		// find k closest cfgs
		vector< pair<Cfg,Cfg> > kpair = FindKClosestPairs(_env,dm,info,vec1[iV1],vec2,k);
		// save VID pairs (convert from Cfg to VID)
		for(int iKP=0; iKP<kpair.size(); ++iKP) {
			pairs.push_back(pair<VID, VID>(pMap->GetVID(kpair[iKP].first),pMap->GetVID(kpair[iKP].second)));
		}
	}

	return pairs;
}

//----------------------------------------------------------------------
// Given: k and a TWO vectors
// Find : k pairs of closest cfgs between the two input vectors of cfgs
// -- if k don't exist, return as many as do
//----------------------------------------------------------------------
vector< CfgPairType > 
ConnectMapNodes::
FindKClosestPairs(Environment *_env,DistanceMetric * dm, CNInfo& info, 
				  vector<Cfg>& vec1, vector<Cfg>& vec2, int k) 
{
	vector< CfgPairType > pairs;	
	// if valid number of pairs requested
	if (k<=0) return pairs;

	//Modified for aCC, replace vec1==vec2
	if( vec1.size()==vec2.size() && equal(vec1.begin(), vec1.end(), vec2.begin()) ){
		return FindKClosestPairs(_env, dm, info, vec1, k);
	} 
	else{
		// initialize w/ k elements each with huge distance...
		vector<DIST_TYPE> kp;
		for (int i=0; i < k; i++) {
			kp.push_back
			(DIST_TYPE(CfgPairType(Cfg::InvalidData(),Cfg::InvalidData()),MAX_DIST));
		}

		// now go through all kp and find closest k
		for (int c1 = 0; c1 < vec1.size(); c1++) {
			for (int c2 = 0; c2 < vec2.size(); c2++) {
				//if( vec1[c1]==vec2[c2] ) continue; //don't connect same
				double dist = dm->Distance(_env, vec1[c1],vec2[c2],info.dmsetid);
				if ( dist < kp[k-1].second) {
					kp[k-1] = DIST_TYPE(CfgPairType(vec1[c1],vec2[c2]),dist);
					sort (kp.begin(), kp.end(), ptr_fun(DIST_Compare) );
				}
			}//endfor c2
		}//endfor c1
		
		// now construct vector of k pairs to return (don't need distances...)
		for (int p=0; p < k && p<kp.size(); p++)
			if (kp[p].first.first != Cfg::InvalidData() && kp[p].first.second != Cfg::InvalidData())
				pairs.push_back( kp[p].first );		
	}//endif vec1 == vec2

	return pairs;
}

//----------------------------------------------------------------------
// Given: k, ONE Cfg and ONE vector of vertices and ONE vector of edges
// Find : find k pairs of closest cfg from "cfg" to "vector"
//----------------------------------------------------------------------
vector< Cfg_VE_Type > 
ConnectMapNodes::
FindKClosestPairs(Roadmap *_rm,DistanceMetric * dm, CNInfo& info,
				  Cfg& cfg, vector<Cfg>& verts, vector< pair<pair<VID,VID>,WEIGHT> >& edges, 
				  int k, bool midpoint) 
{

  vector< Cfg_VE_Type > pairs;

  // if valid number of pairs requested
  if (k<=0) return pairs;

  // initialize w/ k elements each with huge distance...
  vector<VE_DIST_TYPE> kp;
  for (int i=0; i < k; i++)
	  kp.push_back(  VE_DIST_TYPE( Cfg_VE_Type(), MAX_DIST)  );
  
  
  //-- VERTICES:
  //   Note: need to keep the distances so can't just call one of 
  //         the other versions of vertices compatable FindKClosestPairs

  // now go through all kp and find closest k
  for (int c1 = 0; c1 < verts.size(); c1++) {
	  if (cfg != verts[c1] ) { 
		  double dist = dm->Distance(_rm->GetEnvironment(), cfg, verts[c1], info.dmsetid);
		  if ( dist < kp[k-1].second) {
			  kp[k-1] = VE_DIST_TYPE(Cfg_VE_Type(cfg,verts[c1]),dist);
			  sort (kp.begin(), kp.end(), ptr_fun(VE_DIST_Compare) );
		  }
	  }// if (cfg != verts[c1])
  }//endfor c1
  
  //-- EDGES:
  for (int e1 = 0; e1 < edges.size(); e1++) {

	  Cfg endpt1 = _rm->m_pRoadmap->GetData(edges[e1].first.first);
	  Cfg endpt2 = _rm->m_pRoadmap->GetData(edges[e1].first.second);
	  Cfg tmp = cfg.ClosestPtOnLineSegment(endpt1,endpt2);
	  
	  if (tmp != endpt1 && tmp != endpt2){
		  double dist = dm->Distance(_rm->GetEnvironment(), 
			  cfg,
			  tmp,
			  info.dmsetid);
		  
		  if ( dist < kp[k-1].second) {
			  kp[k-1] = VE_DIST_TYPE(Cfg_VE_Type(cfg,
				  tmp,
				  endpt1,endpt2),
				  dist);
			  sort (kp.begin(), kp.end(), ptr_fun(VE_DIST_Compare) );
		  } //endif dist
		  
	  } // endif (tmp != endpt1 && tmp != endpt2)
  } //endfor e1
  
  // now construct vector of k pairs to return (don't need distances...)
  for (int p=0; p < k && p<kp.size(); p++)
	  if (kp[p].first.cfg1 != Cfg::InvalidData() && 
		  kp[p].first.cfg2 != Cfg::InvalidData())
		  pairs.push_back( kp[p].first );

   return pairs;
}

/*---------------------------------------------------------------
Vertices are stored as generated but they may have been originally
generated with respect to some obstacle.  This proceedure
will return a vector where every element is a vector of cfg's
corresponding to a unique (& valid) id value.
---------------------------------------------------------------*/
vec_vec_CfgType
ConnectMapNodes::
Get_Cfgs_By_Obst(Roadmap * _rm){

  //-- get all vertices
  vector<Cfg> vert= _rm->m_pRoadmap->GetVerticesData();

  //-- sort by info
  sort (vert.begin(), vert.end(), ptr_fun(info_Compare) );

  //-- declare value to return
  vec_vec_CfgType byInfo;

  //-- build up a separate vector for each unique & valid id
  vector < Cfg >  tmp;

  int  id, prev= -505;
  bool firstList=true;
  for (int i=0;i<vert.size();++i){
     id= vert[i].info.obst;
     if (id != prev)
        if (prev == -1) {
            tmp.erase(tmp.begin(),tmp.end());
        } else { // valid body
            if (firstList){
                firstList=false;
            } else {
                byInfo.push_back(tmp);
                tmp.erase(tmp.begin(),tmp.end());
            }
        }
     tmp.push_back(vert[i]);
     prev=id;
  }//endfor

  //-- last list wouldn't've been added in loop
  if (  prev != -1  &&  tmp.size()>0  )
     byInfo.push_back(tmp);

  //-- return vector of vectors of vertices
  return byInfo;

}

/*---------------------------------------------------------------
  Implements Rapidly-Exploring Random Tree (RRT) algm

     initial tree - "Roadmap", input parameter
     K iterations - input parameter
     delta T      - input parameter
     set U        - input "direction" set

     note: assumes HOLONOMIC robot
     note: add'l parameters are used in calls to various algobase 
           functions

  Algm explores starting from initial tree input and adds whatever 
  nodes and edges it explores as vertices and edges to m_pRoadmap->
---------------------------------------------------------------*/
void 
ConnectMapNodes::
RRT( Roadmap * rm,int K, double deltaT, vector<Cfg>&U,
        CollisionDetection* cd, LocalPlanners* lp, DistanceMetric * dm,
        CNInfo& info, LPInfo lpInfo){

  // get a local copy for brevity of notation
  Environment *env = rm->GetEnvironment();

  for (int k=0;k<K;++k){
      Cfg tmp = Cfg::GetRandomCfg(env);
      Cfg x_rand = Cfg(tmp);

      vector<Cfg> verticesData;
                  verticesData = rm->m_pRoadmap->GetVerticesData();

	  //find closest Cfg in map from random cfg.
      vector<  CfgPairType > kp = FindKClosestPairs(
                                           env,
                                           dm,
                                           info,
                                           x_rand,
                                           verticesData,
                                           1
                                          );
	  //if there is closet vertex.
      if (kp.size()>0) {

         Cfg x_near = kp[0].second;

         Cfg u;

		 //select direction
         if ( U.size() > 0 ) {
             // select u at random from input set U <---why?
             u = U[(int)(lrand48()%U.size())];
         } else {
             // holonomic robot assumption in purely random selection of u
             u = x_rand;
         }//endif U.size()

         Cfg x_new = Cfg::c1_towards_c2(x_near,u,deltaT);

         //if x_new in bbox AND x_new in freespace AND x_new connectable to x_near
         if (x_new.InBoundingBox(env)
          && !x_new.isCollision(env,cd,info.cdsetid,info.cdInfo)
          && lp->IsConnected(rm,cd,dm,x_near,x_new,info.lpsetid,&lpInfo)){

             // add x_new and connecting edge to x_near into roadmap
             Cfg t=Cfg(x_new);
             rm->m_pRoadmap->AddVertex(t);
             rm->m_pRoadmap->AddEdge(x_near, x_new, lpInfo.edge);
         }
      } //endif (kp.size()>0) 

  }//endfor k

}

/*---------------------------------------------------------------
Copy vertices and all incident edges associated with "vids" 
from one roadmap to another
---------------------------------------------------------------*/
void
ConnectMapNodes::
ModifyRoadMap(
        Roadmap *toMap,
        Roadmap *fromMap,
        vector<VID> vids){

  Cfg t;
  int i;

  //Add vertex
  for (i=0;i<vids.size();++i) {
    t=fromMap->m_pRoadmap->GetData(vids[i]);
    
    toMap->m_pRoadmap->AddVertex(t);
  } //endfor i


  //-- get edges from _rm connected component and add to submap
  for (i=0;i<vids.size();++i) {
     vector< pair<pair<VID,VID>,WEIGHT> > edges =
                           fromMap->m_pRoadmap->GetIncidentEdges(vids[i]);
     for (int j=0;j<edges.size();++j) {
       Cfg t1=fromMap->m_pRoadmap->GetData(edges[j].first.first),
           t2=fromMap->m_pRoadmap->GetData(edges[j].first.second);

       toMap->m_pRoadmap->AddEdge(t1,t2, edges[j].second);
     } //endfor j

  } //endfor i

}

/*---------------------------------------------------------------
Takes existing connected components of size less than or equal to
SmallCC and expands them randomly (or twds connected components).
---------------------------------------------------------------*/
void
ConnectMapNodes::
ConnectNodes_ExpandRRT(
        Roadmap * _rm,CollisionDetection* cd,
        LocalPlanners* lp,DistanceMetric * dm,
        CN& _cn, CNInfo& info){

  // display information specific to method
  #ifndef QUIET
  cout << "(iterations = "<<_cn.GetIterations()
       << ", stepFactor= "<<_cn.GetStepFactor()
       << ", smallcc   = "<<_cn.GetSmallCCSize()<<"): "<<flush;
  #endif

  // initialize information needed to check connection between cfg's
  LPInfo lpInfo=Initialize_LPinfo(_rm,info);

  
///Modified for VC
#if defined(_WIN32)
	using namespace std;
#endif

  // process components from smallest to biggest
  vector< pair<int,VID> > ccvec = _rm->m_pRoadmap->GetCCStats();
  for ( vector< pair<int,VID> >::reverse_iterator cc1=ccvec.rbegin();
       (*cc1).first <= _cn.GetSmallCCSize() && 
#if defined(__HP_aCC)
       (cc1!=ccvec.rend())
#else
       (cc1<ccvec.rend())
#endif
       ;++cc1){

      //-- submap = vertices & edges of current (cc1) connected component
      Roadmap submap;
      submap.environment = _rm->GetEnvironment();
      ModifyRoadMap(&submap,_rm,
                    _rm->m_pRoadmap->GetCC( (*cc1).second));

      if ( !strcmp(_cn.GetName(),"RRTexpand") ){
		  
          // use random U
          vector<Cfg> dummyU;
          RRT(&submap,
              _cn.GetIterations(),
              _cn.GetStepFactor() * lpInfo.positionRes,
              dummyU,
              cd, lp, dm, info, lpInfo);
          //-- map = map + submap
          ModifyRoadMap(_rm,&submap,
			  submap.m_pRoadmap->GetVerticesVID());
		  
      }else{  //"RRTcomponents"
		  
		  // use each connected component cfg's as U
          for (vector< pair<int,VID> >::iterator cc2=ccvec.begin();
		  cc2<ccvec.end();++cc2){
			  
			  VID cc1id = (*cc1).second;
			  VID cc2id = (*cc2).second;
			  
			  if ( !_rm->m_pRoadmap->IsSameCC(cc1id,cc2id) ) {
				  
				  //added tmp & separated decl from init for SUN compiler
				  Cfg tmp;
				  tmp = _rm->m_pRoadmap->GetData(cc2id);
				  vector<Cfg> U = _rm->m_pRoadmap->GetCC(tmp);
				  RRT(&submap,
					  _cn.GetIterations(),
					  _cn.GetStepFactor() * lpInfo.positionRes,
					  U,
					  cd, lp, dm, info, lpInfo);
				  
				  //-- map = map + submap
				  ModifyRoadMap(_rm,&submap,
					  submap.m_pRoadmap->GetVerticesVID());
				  
			  }// endif !_rm
			  
          } // endfor cc2
		  
      }
	  
	  
	   } //endfor cc1 


}

/*---------------------------------------------------------------
Obst to Obst connections are attempted for the "k" closest nodes.
Each "body" of a multibody is considered an obstacle.  Obstacles
have id's.  The k-closest cfg's, one of which was generated wrt the
"first" obstacle (body_i) and the other of which was generated wrt
the "second" (body_j), will have a connection attempted between them.
This continues for all unique id pairings.

For example:
  Given obstacle id's 2,3,4 & 5,
  connection is attempted for the k-closest in each obst-obst
  (i,j) pairing below:
        2-2  2-3  2-4  2-5
             3-3  3-4  3-5
                  4-4  4-5
                       5-5
  When i=j, the "k" value may be different than otherwise
---------------------------------------------------------------*/
void
ConnectMapNodes::
ConnectNodes_ObstBased(
        Roadmap * _rm,CollisionDetection* cd,
        LocalPlanners* lp,DistanceMetric * dm,
        CN& _cn, CNInfo& info){

  // display information specific to method 
  #ifndef QUIET
  cout << "(k_other="<<_cn.GetKOther()
       << ", k_self="<<_cn.GetKSelf()<<"): "<<flush;
  #endif

  // initialize information needed to check connection between cfg's
  LPInfo lpInfo=Initialize_LPinfo(_rm,info);


  //-- get cfg's
  vec_vec_CfgType body = Get_Cfgs_By_Obst(_rm);

  // Each "body" of a multibody is considered an obstacle.
  // for i, each body id
  for (int i=0; i<body.size(); i++){

    // for j, each body id starting with i
    for (int j=i; j<body.size(); j++) {

        int k;
        if (j==i) k = min(_cn.GetKSelf(),body[i].size());
        else      k = min(_cn.GetKOther(),min(body[i].size(),body[j].size()));

        // if no pairs to find, continue to next set of bodies
	if (k==0) continue;

        // get closest pairs of nodes on obst to (possibly) another obstacle
        vector< pair<Cfg,Cfg> > kp = FindKClosestPairs(
                                    _rm->GetEnvironment(),
                                    dm,
                                    info,
                                    body[i],
                                    body[j],
                                    k
                                    );

        //-- check connections between pairs
        for (int m=0;m<kp.size();++m){
                #if CHECKIFSAMECC
                if(_rm->m_pRoadmap->IsSameCC(kp[m].first,kp[m].second)) continue;
                #endif

                if (lp->IsConnected(_rm,cd,dm,
                        kp[m].first,kp[m].second,info.lpsetid,&lpInfo)){
                               _rm->m_pRoadmap->AddEdge(kp[m].first,kp[m].second,lpInfo.edge);
                }//endif IsConnected
        }//endfor m

      } //endfor j
  } //endfor i


}


//--------------------------------------------------------------------
//   "modifiedLM" -- modified Laumond's method. During connection 
//   phase, nodes are randomly generated, they are kept if they can 
//   be connected to no CCs or more than one CCs, otherwise it will 
//   be tossed away(only connected to one CC) if its 'distance' from 
//   the 'center' of that CC not larger than user-specified 'r' times 
//   the radius of that CC, i.e, it will be kept only when it 
//   'expand's that CC.
//
//   Parameters:
//
//   1) kclosest: num of closest nodes in each CC that this node is
//		going to try connection.
//   2) maxNum: the maximum numbers of nodes that are going to be
//		added into the roadmap during this.
//   3) rfactor: multiplier for 'radius' of CC, w/in which thrown out, 
//		outside of which kept.
// Pseudo-code
//------------
//   while (more than one CC remains *AND* added fewer new Cfgs than requested)
//      generate a random configuration, cfg
//      get current connected components from roadmap
//      for each connected component, CC
//           if possible to connect cfg to CC
//                 increment count of connections, #connections
//           endif
//      endfor
//      if (#connections is zero *OR* #connections greater than one )
//         *OR*
//         (#connections is one *AND* cfg distance to CCcenter > rfactor * CCradius)
//                increment count of new Cfgs added
//                add cfg & all edges
//      endif
//   endwhile
// ------------------------------------------------------------------
void
ConnectMapNodes::
ConnectNodes_modifiedLM(
        Roadmap * _rm,CollisionDetection* cd,
        LocalPlanners* lp,DistanceMetric * dm,
        CN& _cn, CNInfo& info){

  #ifndef QUIET
    // display information specific to method
    cout << "(kclosest=" << _cn.GetKClosest();
    cout << ", maxNum=" << _cn.GetMaxNum();
    cout << ", rfactor=" << _cn.GetRFactor()<< "): "<<flush;
  #endif

  const int    kclosest  = _cn.GetKClosest();
  const int    requested = _cn.GetMaxNum();
  const double rfactor   = _cn.GetRFactor();

  // initialize information needed to check connection between cfg's
  LPInfo lpInfo(_rm, info);

  // get local copies of necessary data
  Environment *env = _rm->GetEnvironment();
  int numMultiBody = env->GetMultiBodyCount();
  int robot        = env->GetRobotIndex();

  GNInfo gnInfo;
         gnInfo = info.gn.gnInfo;      // modify only local copy of gnInfo
         gnInfo.addNodes2Map = false;  // don't add new node to map (yet!)
	 int numNodes = 1;             // only need one new node at a time

  //-- unless user has specified otherwise, BasicPRM generation for 'cfg'
  if (!info.gn.generators.IsMember(gnInfo.gnsetid)) gnInfo.gnsetid = BASICPRM;

  // init counter
  int numCfgAdded  = 0;

  //-- while (more than one CC remains *AND* added fewer new Cfgs than requested)
  while(_rm->m_pRoadmap->GetCCStats().size()>1 && numCfgAdded<requested) {

     //init counter
     int numTries=0;
      
     Cfg cfg;
     while ( numTries++ < MAX_NUM ){

          gnInfo.nodes.erase(gnInfo.nodes.begin(), gnInfo.nodes.end());

          //-- generate new 'cfg'
          info.gn.GenerateNodes(_rm,cd,dm, gnInfo.gnsetid, gnInfo);

          // if a cfg node generated exit loop else keep trying...
          if ( gnInfo.nodes.size() > 0 ) 
               break;
          
     }//endwhile numTries

     // OBPRM-variants may return more than one node because of "shells"
     // and multiple obstacles to find the surface of so go thru all nodes returned
     for (int k=gnInfo.nodes.size()-1;k>=0;k--){

          // try to add this cfg
          cfg = gnInfo.nodes[k];

          // declare data structures necessary to return connection info
          vector< pair<Cfg, Cfg> > edges;
          vector< pair<WEIGHT,WEIGHT> > edgelpinfos;

          // init counter
          int numofConnection=0;

          //-- get current connected components from roadmap
          vector< pair<int,VID> > allCC = _rm->m_pRoadmap->GetCCStats();

          //-- for each connected component, CC
          for(int i=0; i<allCC.size(); ++i) {

             Cfg          tmp = _rm->m_pRoadmap->GetData(allCC[i].second);
             vector<Cfg>   CC = _rm->m_pRoadmap->GetCC(tmp);

             vector< pair<Cfg, Cfg> > kp = FindKClosestPairs(
                                      env,dm,info,cfg,CC,kclosest);
 
             //-- if possible to connect cfg to CC
             for(int j=0; j<kp.size(); ++j) {

                if (lp->IsConnected(_rm,cd,dm, kp[j].first,kp[j].second,
                               info.lpsetid,&lpInfo))  {
                     //-- increment count of connections, #connections
                     ++numofConnection;

                     // record edge in case we decide to add it later 
                     edges.push_back(kp[j]);
                     edgelpinfos.push_back(lpInfo.edge);

                     // only need one connection per CC so exit for j loop
                     break;

                } //endif (lp-> ...)

             } //endfor j

          } //endfor i

          // default decision is to not add cfg
          bool addExpandingCfg = false;

          // if only connected to one CC 
          if(numofConnection == 1) {

             // get all cfg's in CC
             vector<Cfg> CC = _rm->m_pRoadmap->GetCC(edges[0].second);

             // calculate CC's center (of mass), CCcenter
             Cfg CCcenter; // sum initialized to 0 by constructor.
             int i;
             for(i=0; i<CC.size(); ++i) {
                double centerWeight = float(i)/(i+1);
                CCcenter = Cfg::WeightedSum(CC[i], CCcenter, centerWeight);
             }

             // calculate CCradius 
             double CCradius = 0.0;
             for(i=0; i<CC.size(); ++i) {
                CCradius += dm->Distance(env, CCcenter, CC[i], info.dmsetid);
             }
             CCradius /= CC.size();

             // calculate distance of cfg to CCcenter 
             double distFromCenter = dm->Distance(env, 
                                                  CCcenter, 
                                                  cfg,
                                                  info.dmsetid);

             // if cfg distance to CCcenter > rfactor * CCradius
             if(distFromCenter > rfactor * CCradius) {
                     // adding cfg 'expands' CC sufficiently so decide to add cfg
                     addExpandingCfg = true;
             }

          }//endif(numofConnection == 1)


          //-- if (#connections is zero *OR* #connections greater than one )
             //-- *OR*
             //-- (#connections is one *AND* cfg distance to CCcenter > rfactor * CCradius)
          if ( numofConnection != 1  ||  addExpandingCfg ) {

                    //-- increment count of new Cfgs added
                    ++numCfgAdded;
     
                    //-- add cfg & all edges
                    _rm->m_pRoadmap->WeightedGraph<Cfg,WEIGHT>::AddVertex(cfg);
                    for(int i=0; i<edges.size(); ++i) {
                              _rm->m_pRoadmap->AddEdge(edges[i].first,  // always 'cfg'
                                                   edges[i].second, // cfg in CC
                                                   edgelpinfos[i]); 
                    }//endfor i

          } //endif (numofConnection != 1 || addExpandingCfg)

     } //endfor k

  } //endwhile

}//end method

/////////////////////////////////////////////////////////////////////
//
//  METHODS for class CN
//      
/////////////////////////////////////////////////////////////////////

CN::
CN() {
  strcpy(name,"");
  connector = 0; 
  cnid = INVALID_EID;
}

CN::
~CN() {
}

bool 
CN::
operator==(const CN& _cn) const
{
  if(strcmp(name,_cn.name) != 0 ) {
     return false;
  } else if ( !strcmp(name,"random") ) {
    return (numEdges == _cn.numEdges ); 
  } else if ( !strcmp(name,"closest") ) {
     return (kclosest == _cn.kclosest );
  } else if ( !strcmp(name,"obstBased") ) {
     return ( (k_other == _cn.k_other) && (k_self == _cn.k_self ) );
  } else if ( !strcmp(name,"RRTexpand") || !strcmp(name,"RRTcomponents") ) {
     return ( (iterations == _cn.iterations) 
             && (stepFactor == _cn.stepFactor)
             && (smallcc == _cn.smallcc) );
  } else if ( !strcmp(name,"components") ) {
     return ( (kpairs == _cn.kpairs) && (smallcc == _cn.smallcc) );
  } else if ( !strcmp(name,"modifiedLM") ) {
     return ( (kclosest == _cn.kclosest) 
             && (maxNum == _cn.maxNum) 
             && (rfactor == _cn.rfactor) );
  } else {  
     return false;
  }
}


char* CN::GetName() const {
  return const_cast<char*>(name);
}

CNF & CN::GetConnector(){
  return connector;
}

EID CN::GetID() const {
  return cnid;
}

int CN::GetNumEdges() const{
    return numEdges;
}

void CN::SetNumEdges(int cEdge) {
    numEdges = cEdge;
}

int CN::GetKClosest() const {
    return kclosest;
}

int CN::GetSmallCCSize() const {
    return smallcc;
}

int CN::GetKPairs() const {
    return kpairs;
}

int CN::GetKOther() const {
    return k_other;
}

int CN::GetKSelf() const {
    return k_self;
}

int CN::GetIterations() const {
    return iterations;
}

int CN::GetStepFactor() const {
    return stepFactor;
}

int CN::GetMaxNum() const {
    return maxNum;
}
double CN::GetRFactor() const {
    return rfactor;
}

ostream& operator<< (ostream& _os, const CN& cn) {
        _os<< cn.GetName();
        if ( !strcmp(cn.GetName(),"random") ){
           _os<< ", numEdges = " << cn.GetNumEdges(); 
        }
        if ( strstr(cn.GetName(),"closest") ){
           _os<< ", kclosest = " << cn.GetKClosest();  
        }
        if ( strstr(cn.GetName(),"obstBased") ){
           _os<< ", k_other = " << cn.GetKOther() << ", k_self = " << cn.GetKSelf();  
        }
        if ( strstr(cn.GetName(),"RRTexpand") || strstr(cn.GetName(),"RRTcomponents") ){
           _os<< ", iterations = " << cn.GetIterations() 
              << ", stepFactor = " << cn.GetStepFactor()
              << ", smallcc = " << cn.GetSmallCCSize();  
        }
        if ( strstr(cn.GetName(),"modifiedLM") ){
           _os<< ", kclosest = " << cn.GetKClosest()
              << ", maxNum = " << cn.GetMaxNum()
              << ", rfactor = " << cn.GetRFactor();
        }
        if ( strstr(cn.GetName(),"components") ){
           _os<< ", kpairs = " << cn.GetKPairs() << ", smallcc = " << cn.GetSmallCCSize();  
        }
        return _os;
}

/////////////////////////////////////////////////////////////////////
//
//  METHODS for class CNSets
//      
/////////////////////////////////////////////////////////////////////

  //==================================
  // CNSets class Methods: Constructors and Destructor
  //==================================

CNSets::
CNSets(){
}

CNSets::
~CNSets(){
}

  //===================================================================
  // CNSets class Methods: Adding CNs, Making & Modifying CN sets
  //===================================================================

int 
CNSets::
AddCN(const char* _cninfo) {
  SID sid = MakeCNSet(_cninfo); 
  SetIDs--;
  return DeleteOSet(sid);        // delete the set, but not elements
}


int 
CNSets::
AddCNToSet(const SID _sid, const EID _cnid) {
  return AddElementToOSet(_sid,_cnid);
}

int 
CNSets::
DeleteCNFromSet(const SID _sid, const EID _cnid) {
  return DeleteElementFromOSet(_sid,_cnid);
}

SID 
CNSets::
MakeCNSet(const char* _cnlist){

  ///Modified for VC
  istrstream  is((char *)_cnlist);
  if (!is) {
         cout << endl << "In MakeCNSet: can't open instring: " << _cnlist ;
         return INVALID_SID;
  }

  return MakeCNSet(is); 
}

SID
CNSets::
MakeCNSet(const EID _eid) {
  return MakeOSet(_eid);
}

SID
CNSets::
MakeCNSet(const vector<EID> _eidvector) {
  return MakeOSet(_eidvector);
}

int
CNSets::
DeleteCNSet(const SID _sid) { 
  return DeleteOSet(_sid);
}


SID 
CNSets:: 
MakeCNSet(istream& _myistream) { 
  char cnname[100]; 
  int kclosest;
  int numEdges;                // parameter for random alg
  int smallcc, kpairs;         // parameters for ConnectCCs
  int k_other, k_self;         // parameters for ObstBased
  int iterations, stepFactor;  // parameters for RRTexpand,RRTcomponents
  int maxNum;                  // parameter  for modifiedLM
  double rfactor;              // parameter  for modifiedLM

  vector<EID> cnvec;  // vector of cnids for this set 

  while ( _myistream >> cnname ) { 			// while cns to process...  
    if (!strcmp(cnname,"random")) {              // Random 
       CN cn1; 
       strcpy(cn1.name,cnname);
       cn1.connector = &ConnectMapNodes::ConnectNodes_Random;
       cn1.numEdges = 0;
        while( _myistream >> numEdges) { //get numEdges value
          if ( numEdges < 0 ) {
            cout << endl << "INVALID: numEdges = " << numEdges;
            exit(-1);
          } else {
            cn1.numEdges = numEdges;
            cn1.cnid = AddElementToUniverse(cn1);
            if( ChangeElementInfo(cn1.cnid,cn1) != OK ) {
                cout << endl << "In MakeSet: couldn't change element info";
                exit(-1);
            }
            cnvec.push_back( cn1.cnid );
          }
        }
       if(cn1.numEdges == 0) {  //if no numEdges value given, use default 5
            cn1.numEdges = DEFAULT_numEdges;
            cn1.cnid = AddElementToUniverse(cn1);
            if( ChangeElementInfo(cn1.cnid,cn1) != OK ) {
                cout << endl << "In MakeSet: couldn't change element info";
                exit(-1);
            }
            cnvec.push_back( cn1.cnid );
       }
       _myistream.clear(); // clear failure to read in last s value

    } else if ( !strcmp(cnname,"closest") || !strcmp(cnname,"closestVE") ) { // closest 
       CN cn1;
       strcpy(cn1.name,cnname);
       if (!strcmp(cnname,"closest"))
           cn1.connector = &ConnectMapNodes::ConnectNodes_Closest;
       else
           cn1.connector = &ConnectMapNodes::ConnectNodes_ClosestVE;
       cn1.kclosest = 0;
        while( _myistream >> kclosest) { //get kclosest value
          if ( kclosest < 0 ) {
            cout << endl << "INVALID: kclosest = " << kclosest;
            exit(-1);
          } else {
            cn1.kclosest = kclosest;
            cn1.cnid = AddElementToUniverse(cn1);
            if( ChangeElementInfo(cn1.cnid,cn1) != OK ) {
                cout << endl << "In MakeSet: couldn't change element info";
                exit(-1);
            }
            cnvec.push_back( cn1.cnid );
          }
        }
       if(cn1.kclosest == 0) {  //if no kclosest value given, use default 5
            cn1.kclosest = 5;
            cn1.cnid = AddElementToUniverse(cn1);
            if( ChangeElementInfo(cn1.cnid,cn1) != OK ) {
                cout << endl << "In MakeSet: couldn't change element info";
                exit(-1);
            }
            cnvec.push_back( cn1.cnid );
       }
       _myistream.clear(); // clear failure to read in last s value

    } else if (!strcmp(cnname,"components")) {          // components
       CN cn1;
       strcpy(cn1.name,cnname);
       cn1.connector = &ConnectMapNodes::ConnectNodes_ConnectCCs;
       cn1.kpairs = 0;
       if ( _myistream >> kpairs) { //get kpairs value
          if ( kpairs < 0 ) {
            cout << endl << "INVALID: kpairs = " << kpairs;
            exit(-1);
          } else {
            if (_myistream >> smallcc) { //get smallcc value, if any
               if ( smallcc < 0 ) { 
                  cout << endl << "INVALID: smallcc = " << smallcc;
                  exit(-1);
               } 
            } else {
               smallcc = SMALL_CC;  // default
            }
            cn1.smallcc = smallcc;
            cn1.kpairs = kpairs;
            cn1.cnid = AddElementToUniverse(cn1);
            if( ChangeElementInfo(cn1.cnid,cn1) != OK ) {
                cout << endl << "In MakeSet: couldn't change element info";
                exit(-1);
            }
            cnvec.push_back( cn1.cnid );
          }
       }
       if(cn1.kpairs == 0) {  //use defaults if no kpairs: kpairs=4,smallcc=SMALL_CC
            cn1.smallcc = SMALL_CC;
            cn1.kpairs = 4;
            cn1.cnid = AddElementToUniverse(cn1);
            if( ChangeElementInfo(cn1.cnid,cn1) != OK ) {
                cout << endl << "In MakeSet: couldn't change element info";
                exit(-1);
            }
            cnvec.push_back( cn1.cnid );
       }
       _myistream.clear(); // clear failure to read in last value

    } else if (!strcmp(cnname,"obstBased")) {          // obstBased
       CN cn1;
       strcpy(cn1.name,cnname);
       cn1.connector = &ConnectMapNodes::ConnectNodes_ObstBased;
       cn1.k_other = 0;
       if ( _myistream >> k_other) { //get value, if any
          if ( k_other < 0 ) {
            cout << endl << "INVALID: k_other = " << k_other;
            exit(-1);
          } else {
            if (_myistream >> k_self) { //get value, if any
               if ( k_self < 0 ) {
                  cout << endl << "INVALID: k_self = " << k_self;
                  exit(-1);
               }
            } else {
               k_self = K_SELF;  // default
            }
            cn1.k_self = k_self;
            cn1.k_other = k_other;
            cn1.cnid = AddElementToUniverse(cn1);
            if( ChangeElementInfo(cn1.cnid,cn1) != OK ) {
                cout << endl << "In MakeSet: couldn't change element info";
                exit(-1);
            }
            cnvec.push_back( cn1.cnid );
          }
       }
       if(cn1.k_other == 0) {  //use defaults if no k_other
            cn1.k_other = K_OTHER;
            cn1.k_self  = K_SELF;
            cn1.cnid = AddElementToUniverse(cn1);
            if( ChangeElementInfo(cn1.cnid,cn1) != OK ) {
                cout << endl << "In MakeSet: couldn't change element info";
                exit(-1);
            }
            cnvec.push_back( cn1.cnid );
       }
       _myistream.clear(); // clear failure to read in last value

    } else if (!strcmp(cnname,"RRTexpand") || !strcmp(cnname,"RRTcomponents") ) {
       CN cn1;
       strcpy(cn1.name,cnname);
       cn1.connector = &ConnectMapNodes::ConnectNodes_ExpandRRT;

       if ( _myistream >> iterations) { //get value, if any
          if ( iterations < 0 ) {
            cout << endl << "INVALID: iterations = " << iterations;
            exit(-1);
          } else {
            if (_myistream >> stepFactor) { //get value, if any
               if ( stepFactor < 0 ) {
                  cout << endl << "INVALID: stepFactor = " << stepFactor;
                  exit(-1);
               } else {
                 if (_myistream >> smallcc) { //get value, if any
                    if ( smallcc < 0 ) {
                       cout << endl << "INVALID: smallcc = " << smallcc;
                       exit(-1);
                    }
                 } else {
                       smallcc = SMALL_CC;  // default
                 }
               }
            } else {
               stepFactor = STEP_FACTOR;  // default
               smallcc    = SMALL_CC;  // default
            }
          }
       } else {
          iterations = ITERATIONS;  // default
          stepFactor = STEP_FACTOR;  // default
          smallcc    = SMALL_CC;  // default
       }
       cn1.iterations = iterations;
       cn1.stepFactor = stepFactor;
       cn1.smallcc    = smallcc;

       cn1.cnid = AddElementToUniverse(cn1);
       if( ChangeElementInfo(cn1.cnid,cn1) != OK ) {
          cout << endl << "In MakeSet: couldn't change element info";
          exit(-1);
       }
       cnvec.push_back( cn1.cnid );

       _myistream.clear(); // clear failure to read in last value


    } else if (!strcmp(cnname,"modifiedLM")) {          // modifiedLM
       CN cn1;
       strcpy(cn1.name,cnname);
       cn1.connector = &ConnectMapNodes::ConnectNodes_modifiedLM;
       if ( _myistream >> kclosest) { //get value, if any
          if (  kclosest < 0 ) {
            cout << endl << "INVALID: kclosest = " << kclosest;
            exit(-1);
          } else {
            if (_myistream >> maxNum) { //get value, if any
               if ( maxNum < 0 ) {
                  cout << endl << "INVALID: maxNum = " << maxNum;
                  exit(-1);
               } else {
                  if (_myistream >> rfactor) { //get value, if any
                     if ( rfactor < 0 ) { 
                        cout << endl << "INVALID: rfactor = " << rfactor;
                        exit(-1);
                     }
		  } else {
               		rfactor = RFACTOR;  // default
                  }
	       }
            } else {
               maxNum = MAX_NUM;  // default
               rfactor = RFACTOR;  // default
            }
          }
       } else {
          kclosest = 5;  // default
          maxNum = MAX_NUM;  // default
          rfactor = RFACTOR;  // default
       }
       cn1.kclosest = kclosest;
       cn1.maxNum = maxNum;
       cn1.rfactor = rfactor;

       cn1.cnid = AddElementToUniverse(cn1);
       if( ChangeElementInfo(cn1.cnid,cn1) != OK ) {
          cout << endl << "In MakeSet: couldn't change element info";
          exit(-1);
       }
       cnvec.push_back( cn1.cnid );

       _myistream.clear(); // clear failure to read in last value


    } else {
       cout << "INVALID: connection method name = " << cnname;
       exit(-1);
    }
  } // end while 

  return MakeOSet(cnvec);
}

  //===================================================================
  // CNSets class Methods: Getting Data & Statistics
  //===================================================================

CN
CNSets::
GetCN(const EID _cnid) const {
   return GetElement(_cnid);
}

vector<CN> 
CNSets::
GetCNs() const {
  vector<CN> elts2; 
  vector<pair<EID,CN> > elts1 = GetElements(); 
  for (int i=0; i < elts1.size(); i++) 
     elts2.push_back( elts1[i].second );
  return elts2; 
}

vector<CN> 
CNSets::
GetCNSet(const SID _sid) const {
  vector<CN> elts2; 
  vector<pair<EID,CN> > elts1 = GetOSet(_sid); 
  for (int i=0; i < elts1.size(); i++) 
     elts2.push_back( elts1[i].second );
  return elts2; 
}


vector<pair<SID,vector<CN> > > 
CNSets::
GetCNSets() const {

  vector<pair<SID,vector<CN> > > s2;
  vector<CN> thesecns;

  vector<pair<SID,vector<pair<EID,CN> > > > s1 = GetOSets(); 

  for (int i=0; i < s1.size(); i++)  {
    thesecns.erase(thesecns.begin(),thesecns.end());
    for (int j=0; j < s1[i].second.size(); j++ ) 
       thesecns.push_back (s1[i].second[j].second);
    s2.push_back( pair<SID,vector<CN> > (s1[i].first,thesecns) );
  }
  return s2; 
}



  //===================================================================
  // CNSets class Methods: Display, Input, Output
  //===================================================================


void 
CNSets::
DisplayCNs() const{
   DisplayElements();
}

void 
CNSets::
DisplayCN(const EID _cnid) const{
   DisplayElement(_cnid);
}

void 
CNSets::
DisplayCNSets() const{
   DisplayOSets();
}

void 
CNSets::
DisplayCNSet(const SID _sid) const{
   DisplayOSet(_sid);
}

void
CNSets::
WriteCNs(const char* _fname) const {

      ofstream  myofstream(_fname);
      if (!myofstream) {
         cout << endl << "In WriteCNS: can't open outfile: " << _fname ;
      }
      WriteCNs(myofstream);
      myofstream.close();
}

void
CNSets::
WriteCNs(ostream& _myostream) const {

      vector<CN> cns = GetCNs();

      _myostream << endl << "#####CNSTART#####";
      _myostream << endl << cns.size();  // number of cns

      //format: CN_NAME (a string) CN_PARMS (double, int, etc)
      for (int i = 0; i < cns.size() ; i++) {
          _myostream << endl;
          _myostream << cns[i].name << " ";
          if ( !strcmp(cns[i].name,"random") ) {
             _myostream << " ";
          }
          if ( strstr(cns[i].name,"closest") ) {
             _myostream << " ";
          }
      }
      _myostream << endl << "#####CNSTOP#####";
}

void
CNSets::
ReadCNs(const char* _fname) {

      ifstream  myifstream(_fname);
      if (!myifstream) {
         cout << endl << "In ReadCNs: can't open infile: " << _fname ;
         return;
      }
      ReadCNs(myifstream);
      myifstream.close();
}

void
CNSets::
ReadCNs(istream& _myistream) {

      char tagstring[100];
      char cndesc[100];
      int  numCNs;

      _myistream >> tagstring;
      if ( !strstr(tagstring,"CNSTART") ) {
         cout << endl << "In ReadCNs: didn't read CNSTART tag right";
         return;
      }

      _myistream >> numCNs;
      _myistream.getline(cndesc,100,'\n');  // throw out rest of this line
      for (int i = 0; i < numCNs; i++) {
        _myistream.getline(cndesc,100,'\n');
        AddCN(cndesc);
      }

      _myistream >> tagstring;
      if ( !strstr(tagstring,"CNSTOP") ) {
         cout << endl << "In ReadCNs: didn't read CNSTOP tag right";
         return;
      }
}

