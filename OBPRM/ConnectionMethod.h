#ifndef ConnectionMethod_h
#define ConnectionMethod_h
#include "DistanceMetrics.h"
#include "LocalPlanners.h"

#ifdef __K2
  #include <strstream.h>
#else
  #include <strstream>
#endif

// Abstract Interface Class for connection methods
template <class CFG, class WEIGHT>
class ConnectionMethod { 
 public:

  //////////////////////
  // Constructors and Destructor
  ConnectionMethod();
  ~ConnectionMethod();

  //////////////////////
  // Access
  //virtual string GetName();
  virtual char* GetName();
  virtual void SetDefault() = 0;

  //////////////////////
  // I/O methods

  virtual void ParseCommandLine(istrstream& is) = 0;
  virtual void PrintUsage(ostream& _os) = 0;
  virtual void PrintValues(ostream& _os) = 0;   
  virtual ConnectionMethod<CFG, WEIGHT>* CreateCopy() =0;

  /////////////////////
  // Useful functions

  /**Compare two distances in DIST_TYPE instances.
   *return (_cc1.second < _cc2.second)
   */
  static bool DIST_Compare(const pair<pair<CFG,CFG>,double>& _cc1, 
			   const pair<pair<CFG,CFG>,double>& _cc2);
   /**Compare distances in 2 CfgDistType instances.
    *@note used to sort cfgs by distance
    */
  static bool CfgDist_Compare(const pair<CFG,double>&, 
			       const pair<CFG,double>&);
   /**Sort given list accroding to the distance from given
    *Cfg to every Cfg in the list.
    *
    *@param _cfgs A list of Cfgs. this list will be sorted at the end of this
    *function.
    *@param _cfg1 Distances will be calculated from this Cfg to every Cfgs in
    *_cfgs.
    *
    *@note Distance is calculated depends on distance metric set defined in info.
    *@note used by Query::CanConnectToCC.
    */
   void SortByDistFromCfg(Environment* _env, DistanceMetric* dm, 
			  SID dmsetid,
			  const CFG& _cfg1, 
			  vector<CFG>&  _cfgs);
  /**Find k pairs of closest Cfgs from a given Cfg to 
   *all Cfgs in a given vector.
   *
   *@param vec1 A list of Cfgs.
   *@param k Find k closest Cfgs in vec1 for a given Cfg.
   *
   *@return a list of pairs, <cfg1, n1>, <cfg1, n2>, ... , <cfg1, nk>
   *Here n1,..,nk are k Cfgs in vec1 and are k closest neighbors of cfg1.
   */
  //static 
  vector<pair<CFG,CFG> > FindKClosestPairs(Environment* _env, 
						  DistanceMetric* dm, 
						  CFG& cfg1,
						  vector<CFG>& vec1, int k);
  /**Find k of closest Cfgs in given vector of Cfgs for each Cfg
   *in same vector.
   *Therefore the return list will contains k*n pairs.
   *Here n is number of Cgfs in vec1.
   *
   *Following is short Alg for calculating result list:
   *   -# for every Cfg, c1, in vector
   *       -# find k closest Cfg in vec1 for c1.
   *   -# end for
   *
   *@param vec1 A list of Cfgs.
   *@param k Find k pairs for each Cfg in vec1.
   *
   *@return A (k*n-elemet) list of pair of Cfgs, and each pair represents a path from
   *the frist Cfg to the second Cfg which has first-k-small distance between
   *the frist Cfg to all other Cfgs in vec1.
   *@see FindKClosestPairs(Roadmap *, DistanceMetric *, CNInfo& , vector<Cfg>& , int )
   *use same alogithm, but returns differnt format of list.
   */
  //static 
  vector<pair<CFG,CFG> > FindKClosestPairs(Environment* _env, 
						  DistanceMetric* dm,  
						  vector<CFG>& vec1, int k);
  /**Find k of closest Cfgs in given vector of Cfgs for each Cfg
   *in same vector.
   *Therefore the return list will contains k*n pairs.
   *Here n is number of Cgfs in vec1.
   *
   *Following is short Alg for calculating result list:
   *   -# for every Cfg, c1, in vector
   *       -# find k closest Cfg in vec1 for c1.
   *   -# end for
   *
   *@param vec1 A list of Cfgs.
   *@param k Find k pairs for each Cfg in vec1.
   *
   *@return A (k*n-elemet) list of pair of VIDs, and each pair represents a path from 
   *the frist VID to the second VID which has first-k-small distance between 
   *the frist VID to all other elements in vec1.
   */
  //static 
  vector<pair<VID,VID> > FindKClosestPairs(Roadmap<CFG, WEIGHT>* rm, 
						  DistanceMetric* dm,
						  vector<CFG>& vec1, int k);
  /**
   *k pairs of closest cfgs for each cfg in vec1 to all cfgs in vec2.
   *This means there will be k*n pairs returned. n in number of cfgs in 
   *vec1. k pair for each cfg in vec1.
   *The differences between this function and FindKClosestPairs
   *(Environment *,DistanceMetric * , CNInfo& info, vector<Cfg>& , vector<Cfg>& , int )
   *are type and size of return values.
   *Following is short Alg for calculating result list:
   *   -# for every Cfg, c1, in first vector
   *		-# find k first closest cfg in second vector from c1
   *   -# end for
   *
   *@param vec1 A list of Cfgs.
   *@param vec2 A list of Cfgs.
   *@param k Find k pairs with first k shortest distance between vec1 and vec2.
   *
   *@return A (k*n-elemet) list of pair of Cfgs, and each pair represents a path from 
   *the frist Cfg to the second Cfg which has k-small distance between all 
   *possilbe paths.
   */
  //static 
  vector<pair<VID,VID> > FindKClosestPairs(Roadmap<CFG, WEIGHT>* rm,
						  DistanceMetric* dm, 
						  vector<CFG>& vec1, 
						  vector<CFG>& vec2, int k);
  /**Find k pairs of closest Cfgs between the two input vectors of Cfgs.
   *This method check distance from every Cfg in vec1 to every Cfg in vec2.
   *The first k shorst path will be returned among these pathes.
   *Following is short Alg for calculating result list:
   *   -# for every Cfg, c1, in first vector
   *       -# for every Cfg, c2, in second vector
   *           -# if distance(c1,c2)< largest distance in return list.
   *           -# then replace largest distance by this path (c1->c2)
   *           -# sort return list.
   *           -# end if
   *       -# end for
   *   -# end for
   *
   *@param vec1 A list of Cfgs.
   *@param vec2 A list of Cfgs.
   *@param k Find k pairs with first k shortest distance between vec1 and vec2.
   *
   *@return A (k-elemet) list of pair of Cfgs, and each pair represents a path from 
   *the frist Cfg to the second  Cfg which has k-small distance between all 
   *possilbe paths.   */
  //static 
  vector<pair<CFG,CFG> > FindKClosestPairs(Environment* _env, 
					   DistanceMetric* dm,
					   vector<CFG>& vec1,
					   vector<CFG>& vec2, int k);

  //////////////////////
  // Core: Connection methods 
  virtual void ConnectComponents() = 0;
  virtual void ConnectComponents(Roadmap<CFG, WEIGHT>*, 
				 CollisionDetection*, 
				 DistanceMetric *,
				 LocalPlanners<CFG,WEIGHT>*,
				 bool addPartialEdge,
				 bool addAllEdges) = 0;
 protected:
  //////////////////////
  // Data
  char* element_name; //Method name in the command line

   
  //////////////////////////////////////////////////////
  // 
  // Public Data
  //
 public:
  SID* cdsetid;
  SID* dmsetid;
  CDInfo* cdInfo;
  double connectionPosRes, ///< Position resolution for node connection
         connectionOriRes; ///< Orientation resolution for node connection

};


template <class CFG, class WEIGHT>
ConnectionMethod<CFG,WEIGHT>::ConnectionMethod() {
}


template <class CFG, class WEIGHT>
ConnectionMethod<CFG,WEIGHT>::~ConnectionMethod(){
  //rdmp = NULL;
  //cd = NULL;
  //dm = NULL;
  //lp = NULL;

}

template <class CFG, class WEIGHT>
char* ConnectionMethod<CFG,WEIGHT>::GetName() { 
  return element_name; 
}
//
// used by "findKClosestPairs" for sort (DIST_TYPE is pair of cfgs & distance)
//
template <class CFG, class WEIGHT>
bool
ConnectionMethod<CFG, WEIGHT>::
DIST_Compare (const pair<pair<CFG,CFG>,double>& _cc1, const pair<pair<CFG,CFG>,double>& _cc2) {
  return (_cc1.second < _cc2.second ) ;
}

//
// used to sort cfgs by distance (CfgDistType is single cfg & distance)
//
template <class CFG, class WEIGHT>
bool
ConnectionMethod<CFG, WEIGHT>::
CfgDist_Compare (const pair<CFG,double>& _cc1, const pair<CFG,double>& _cc2) {
  return (_cc1.second < _cc2.second ) ;
}
//----------------------------------------------------------------------
// SortByDistFromCfg
//
// Given: ONE cfg1  and ONE vector of cfgs
// Do:    sort cfgs by distance from cfg1 (modifies cfgs)
//----------------------------------------------------------------------
template <class CFG, class WEIGHT>
void
ConnectionMethod<CFG, WEIGHT>::
SortByDistFromCfg(Environment* _env, DistanceMetric* dm, SID dmsetid,
		  const CFG& _cfg1, 
		  vector<CFG>& _cfgs) {
  // compute distances from _cfg1 for all cfgs in _cfgs
  vector<pair<CFG,double> > distances;
  pair<CFG,double> tmp;
  for (int i=0; i < _cfgs.size(); i++) {
    double dist = dm->Distance(_env, (CFG*)&_cfg1, (CFG*)&_cfgs[i], dmsetid);
    tmp.first = _cfgs[i];
    tmp.second = dist;
    distances.push_back(tmp);
  }

  sort (distances.begin(), distances.end(), ptr_fun(CfgDist_Compare) );
  
  // now reconstruct _cfgs vector, sorted by distances 
  for (int j=0; j < _cfgs.size(); j++) 
    _cfgs[j] = distances[j].first;
  
  return ;
}
//----------------------------------------------------------------------
// Given: k and ONE cfg and ONE vector
// Find : find k pairs of closest cfg from "cfg" to "vector"
//----------------------------------------------------------------------
template <class CFG, class WEIGHT>
vector<pair<CFG,CFG> >
ConnectionMethod<CFG, WEIGHT>::
FindKClosestPairs(Environment* _env, DistanceMetric* dm, 
		  CFG& cfg1, vector<CFG>& vec1, int k) {
  vector<CFG> cfg;
  cfg.push_back(cfg1);
  
  // find kclosest cfgs to cfg in vertices
  return FindKClosestPairs( _env, dm, cfg, vec1, k);
}

//----------------------------------------------------------------------
// Given: k and ONE vector
// Find : find k pairs of closest cfg from vector to each cfg in vector
//----------------------------------------------------------------------
template <class CFG, class WEIGHT>
vector<pair<CFG,CFG> >
ConnectionMethod<CFG, WEIGHT>::
FindKClosestPairs(Environment* _env, DistanceMetric* dm, 
		  vector<CFG>& vec1, int k) {
  vector< pair<CFG,CFG> > pairs;
  // if valid number of pairs requested
  if (k<=0) return pairs;
  
  vector<CFG> vec_of_cfgs = vec1;
  
  //for each cfg in given vector
  for( int iV1=vec1.size()-1; iV1>1; iV1-- ){
    // find k closest cfgs between the two vectors 
    vector< pair<CFG,CFG> > kp = FindKClosestPairs(_env,dm,vec1[iV1],vec_of_cfgs,k);
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
template <class CFG, class WEIGHT>
vector< pair<VID, VID> >
ConnectionMethod<CFG, WEIGHT>::
FindKClosestPairs(Roadmap<CFG, WEIGHT> *rm, 
		  DistanceMetric* dm, 
		  vector<CFG>& vec1, int k) {
  return FindKClosestPairs(rm, dm, vec1, vec1, k);
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
template <class CFG, class WEIGHT>
vector< pair<VID, VID> > 
ConnectionMethod<CFG, WEIGHT>::
FindKClosestPairs(Roadmap<CFG, WEIGHT>* rm, 
		  DistanceMetric* dm, 
		  vector<CFG>& vec1,vector<CFG>& vec2, int k) {
  vector< pair<VID, VID> > pairs;
  if (k<=0) return pairs;
  
  Environment *_env = rm->GetEnvironment();
  RoadmapGraph<CFG, WEIGHT> * pMap = rm->m_pRoadmap;
  
  //compute from the last to the second (ignore the first)
  for( int iV1=vec1.size()-1; iV1>0; iV1-- ){
    // find k closest cfgs
    vector< pair<CFG,CFG> > kpair = FindKClosestPairs(_env,dm,vec1[iV1],vec2,k);
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
template <class CFG, class WEIGHT>
vector<pair<CFG,CFG> > 
ConnectionMethod<CFG, WEIGHT>::
FindKClosestPairs(Environment* _env, DistanceMetric* dm, 
		  vector<CFG>& vec1, vector<CFG>& vec2, int k) {
  vector<pair<CFG,CFG> > pairs;	
  CFG invalid;
  invalid.InvalidData();
  pair<pair<CFG,CFG>,double> tmp;
  // if valid number of pairs requested
  if (k<=0) return pairs;
  
  //Modified for aCC, replace vec1==vec2
  if( vec1.size()==vec2.size() && equal(vec1.begin(), vec1.end(), vec2.begin()) ){
    return FindKClosestPairs(_env, dm, vec1, k);
  } else {
    // initialize w/ k elements each with huge distance...
    vector<pair<pair<CFG,CFG>,double> > kp;
    for (int i=0; i < k; i++) {
      tmp.first.first = invalid;
      tmp.first.second = invalid;
      tmp.second = MAX_DIST;
      kp.push_back(tmp);
    }
    
    // now go through all kp and find closest k
    for (int c1 = 0; c1 < vec1.size(); c1++) {
      for (int c2 = 0; c2 < vec2.size(); c2++) {
	//if( vec1[c1]==vec2[c2] ) continue; //don't connect same

	double dist = dm->Distance(_env, vec1[c1],vec2[c2],*dmsetid);
	if ( dist < kp[k-1].second) {
	  tmp.first.first = vec1[c1];
	  tmp.first.second = vec2[c2];
	  tmp.second = dist;
	  kp[k-1] = tmp;
	  // sort (kp.begin(), kp.end(), ptr_fun(ConnectionMethod<CFG, WEIGHT>::DIST_Compare) );
	  sort (kp.begin(), kp.end(), ptr_fun(DIST_Compare) );
	}
      }//endfor c2
    }//endfor c1
    
		// now construct vector of k pairs to return (don't need distances...)
    for (int p=0; p < k && p<kp.size(); p++)
      if (kp[p].first.first != invalid && kp[p].first.second != invalid)
	pairs.push_back( kp[p].first );		
  }//endif vec1 == vec2
  
  return pairs;
}


#endif
