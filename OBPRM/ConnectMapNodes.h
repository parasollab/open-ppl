// $Id$
/////////////////////////////////////////////////////////////////////
//
//  ConnectMapNodes.h
//
//  General Description
//      This set of classes supports a "RoadMap Node Connection Algobase".
//     	Generate roadmap edges and enter each as a graph edge. 
//
//      The classes in the set are:
//        o CN            -- info related to an individual method of 
//                           node connection 
//
//        o CNSets        -- contains 'master' list of all connection methods
//                           and maintains sets of connection methods
//                           (derived from BasicSets<CN>)
//                           
//        o ConnectMapNodes -- 	has CNSets as data element, and its methods
//                           	include the actual connection algorithms.
//                           	A 'wrapper' method cycles thru sets of cn's. 
//
//  Created
//      8/27/98  Daniel Vallejo
//
//  Last Modified By:
//      8/21/99  Lucia K. Dale   add ObstBased connection & aux methods
//                               changed to work w/ InfoCfg
//
/////////////////////////////////////////////////////////////////////

#ifndef ConnectMapNodes_h
#define ConnectMapNodes_h

#include "OBPRM.h"
#include "Sets.h"
#include "RoadmapGraph.h"

#include "util.h"
#include "CollisionDetection.h"
#include "LocalPlanners.h"

#include <stdlib.h>
#include <stdio.h>

#include <iostream.h>
#include <fstream.h>
#include <strstream.h>
#include <iomanip.h>

#include <vector.h>


//---------------------------------------------------------------
//  Pre-defined Algobase Sets
//  CAUTION:  DO NOT CHANGE THE ENUMERATION ORDERS w/o CHANGING
//              SET DEFN's in "DefaultInit" method of 
//		GenerateMapNodes class
//---------------------------------------------------------------
enum cn_predefined {    //----------------
        RANDOM,         // Random
        CLOSEST10,      // Try closest 10 on every other object
        CLOSEST20,      //  "     "    20  "   "     "      "
        CN_USER1};      // first user defined cn set, if any

//---------------------------------------------------------------
// Algo base information data structures
//---------------------------------------------------------------
struct CNInfo {
    SID cnsetid;        // connector set id
    SID lpsetid;        // local planner set id
    SID cdsetid;        // collision detection set id
    SID dmsetid;        // distance metric set id
    int numEdges;
    vector <EdgeInfo<WEIGHT> > edges;
    bool addPartialEdge;
    int dupeNodes,dupeEdges;  // used for acct'ing w/ closestVE

};

#define SMALL_CC    3                   // default cut-off for small CCs

class CN;
class CNSets;
class ConnectNodes;

class Roadmap;
class Environment;

const double MAX_DIST =  1e10;

// pointer to cn function
typedef void (*CNF) (Roadmap* rm, CollisionDetection *,
			 LocalPlanners* ,DistanceMetric *, CN&, CNInfo&); 



/////////////////////////////////////////////////////////////////////
//  class CN
//
//  General Description
//     This class contains information relevant to a particular 
//     Connect Map Nodes strategy (e.g., name, ptr to function, etc).
//     
//
/////////////////////////////////////////////////////////////////////
class CN {
  friend class CNSets;
public:

  //===================================================================
  // Constructors and Destructor
  //===================================================================
  CN();
  ~CN();

  //===================================================================
  // Operators
  //===================================================================
  bool operator==(const CN & _cn) const;

  //===================================================================
  // Other Methods
  //===================================================================
  char*  GetName() const;
  CNF    GetConnector();
  EID    GetID() const;
  int  	 GetNumEdges   () const;
  int    GetKClosest   () const;
  int 	 GetSmallCCSize() const;
  int 	 GetKPairs     () const;
  int 	 GetKOther     () const;
  int 	 GetKSelf      () const;
  int 	 GetIterations () const;
  int 	 GetStepFactor () const;
  int    GetMaxNum     () const;

protected:
  //===================================================================
  // Data
  //===================================================================
  char  name[80];
  CNF   connector;          // ptr to connection strategy code
  EID   cnid;
  int 	numEdges;          // used by random algm 
  int   kclosest;          // used by closest algm & modifiedLM
  int   maxNum;            // used by modifiedLM
  int 	kpairs;            // used by components algm
  int 	smallcc;           // used by components & rrt algm
  int 	stepFactor;        // used by rrt algm
  int 	iterations;        // used by rrt algm
  int 	k_other;           // used by obstBased algm
  int 	k_self;            // used by obstBased algm
 
private:
};

ostream& operator<< (ostream& _os, const CN& cn); 



/////////////////////////////////////////////////////////////////////
//  class CNSets
//
//  General Description
//     This class is derived from BasicSets<CN>.
//
//     This class manages/contains:
//        * a 'master' list of all connection strategies, and
//        * sets of connection strategies
//
//     Each connection strategy and set is given a unique id. The cn ids (EIDs)
//     are used to compose labels encoding connection strategies sets. 
//
/////////////////////////////////////////////////////////////////////
class CNSets : public BasicSets<CN> {
public:
  //===================================================================
  //  Constructors and Destructor
  //===================================================================
    CNSets();
    ~CNSets();

  //===================================================================
  //  Other Methods
  //===================================================================

        // Adding CNs, Making & Modifying CN sets
   int AddCN(const char* _cninfo);     // add cn(s) to universe
   int AddCNToSet(const SID _sid, const EID _cnid);
   int DeleteCNFromSet(const SID _sid, const EID _cnid);

   SID MakeCNSet(const char* cnlist);  // make an ordered set of cns,
   SID MakeCNSet(istream& _myistream); //  - add cn to universe if not there
   SID MakeCNSet(const EID _eid);
   SID MakeCNSet(const vector<EID> _eidvector);

   int DeleteCNSet(const SID _sid);

   // Getting Data & Statistics
   CN GetCN(const EID _cnid) const;
   vector<CN> GetCNs() const;
   vector<CN> GetCNSet(const SID _sid) const;
   vector<pair<SID,vector<CN> > > GetCNSets() const;

   // Display, Input, Output
   void DisplayCNs() const;
   void DisplayCN(const EID _cnid) const;
   void DisplayCNSets() const;
   void DisplayCNSet(const SID _sid) const;

   void WriteCNs(const char* _fname) const;
   void WriteCNs(ostream& _myostream) const;
   void ReadCNs(const char* _fname);
   void ReadCNs(istream& _myistream);

  //===================================================================
  //  Data
  //===================================================================
protected:
private:
};

/////////////////////////////////////////////////////////////////////
//  class ConnectMapNodes
//
//  General Description
//     This is the main connection strategy class. It has an CNSets object
//     as a data element (list of cns and cn sets), and its methods
//     include the actual connection strategy algorithms:
//        * "random" -- random connections
//        * "closest" --  try to connect each node with the k closest nodes
//        * "obstBased" -- try to connect different obstacles 
//           note1: parameters are 1) k_other(#pairs to try between different obst)
//                                 2) k_self (#pairs to try from each cfg to 
//                                            obst's own cfg list)
//
//           sample command line "-cNodes obstBased [k_other] [k_self]"
//
//        * "components" -- try to connect different connected components
//           note1: should be used after "closest"
//           note2: parameters are 1) kpairs (#pairs to try between big ccs) 
//                                 2) smallcc (size of CCs if try all 
//					connections)
//
//           sample command line "-cNodes closest components [kpairs] [smallcc]"
//
//        * "expandRRT" -- try to grow small connected components into larger ones
//           note1: should be used after some other method
//           note2: parameters are 1) iterations (of RRT algm)
//                                 2) stepFactor (mult of environment determined 
//                                               "posres" to use)
//                                 3) smallcc (grow CCs smaller/equal this size)
//        * "modifiedLM" -- modified Laumond's method. During connection phase,
//           nodes are randomly generated, they are kept if they can be connected
//           to no CCs or more than one CCs, otherwise it will be tossed away(only
//           connected to one CC) if its 'distance' from the 'center' of that CC
//           not larger than 2 times the radius of that CC, i.e, it will be kept
//           only if it 'expand' that CC.
//           note:  parameters are 1) kclosest: num of closest nodes in each CC
//                                    that this node is going to try connection.
//                                 2) maxNum: the maximum numbers of nodes that
//                                    are going to be added into the roadmap during this.
//
//     'Wrapper' methods cycle thru sets of cns, stopping upon 1st
//     success or trying all.
//
/////////////////////////////////////////////////////////////////////
//brc moved this from ConnectMapNodes:private
  class Cfg_VE_Type {
  public:
    // destructor
    ~Cfg_VE_Type();

    // constructors
    Cfg_VE_Type();
    Cfg_VE_Type(Cfg& _cfg1,Cfg& _cfg2);
    Cfg_VE_Type(Cfg& _cfg1,Cfg& _cfg2, Cfg& _endpt1,Cfg& _endpt2);
    // data
    Cfg         cfg1,cfg2;
    bool            cfg2_IsOnEdge;
    vector<Cfg> endpt;
  }; //Cfg_VE_Type
  typedef vector < vector<Cfg> >  vec_vec_CfgType;
  typedef pair   <Cfg,Cfg>    CfgPairType;
  typedef pair   <CfgPairType,double> DIST_TYPE;
  typedef pair   <Cfg_VE_Type,double> VE_DIST_TYPE;
// end of brc mvoed part

class ConnectMapNodes {
   friend SID MakeCNSet(istream&); 
public:
  //===================================================================
  // Constructors and Destructor
  //===================================================================
  ConnectMapNodes();
  ~ConnectMapNodes();

  //===================================================================
  // Other Methods
  //===================================================================

  void DefaultInit();
  void UserInit(Input * input);

  void ConnectNodes(Roadmap*, CollisionDetection *,LocalPlanners* ,
		DistanceMetric *,SID, CNInfo&); 

  void ConnectNodes(Environment*, RoadmapGraph<Cfg,WEIGHT> & roadmap,
		CollisionDetection *, LocalPlanners* ,DistanceMetric *,
		SID, CNInfo&);

  static void ConnectNodes_Random(Roadmap*, CollisionDetection *,
				LocalPlanners* ,DistanceMetric *, 
				CN&, CNInfo&); 

  static void ConnectNodes_Closest(Roadmap*, CollisionDetection *,
				LocalPlanners* ,DistanceMetric *, 
				CN&, CNInfo&); 

  static void ConnectNodes_ClosestVE(Roadmap*, CollisionDetection *,
				LocalPlanners* ,DistanceMetric *, 
				CN&, CNInfo&); 

  static void ConnectNodes_ConnectCCs(Roadmap*, CollisionDetection *,
				LocalPlanners* ,DistanceMetric *, 
				CN&, CNInfo&); 

  static void ConnectNodes_ObstBased(Roadmap*, CollisionDetection *,
                                LocalPlanners* ,DistanceMetric *,
                                CN&, CNInfo&);

  static void ConnectNodes_ExpandRRT(Roadmap*, CollisionDetection *,
                                LocalPlanners* ,DistanceMetric *,
                                CN&, CNInfo&);

  static void ConnectNodes_modifiedLM(Roadmap*, CollisionDetection *,
                                LocalPlanners* ,DistanceMetric *,
                                CN&, CNInfo&);

  static void ConnectNodes_RRTcomponents(Roadmap*, CollisionDetection *,
                                LocalPlanners* ,DistanceMetric *,
                                CN&, CNInfo&);

  typedef pair <Cfg,double> CfgDistType;
  static bool CfgDist_Compare (const CfgDistType&, const CfgDistType&);
  static void SortByDistFromCfg (Environment *_env,DistanceMetric *dm, 
				CNInfo& info, const Cfg& _cfg1, vector<Cfg>&  _cfgs);

private:



  static void ModifyRoadMap(Roadmap *toMap, Roadmap *fromMap, vector<VID> vids);

  static void RRT(Roadmap* rm, int K, double deltaT, vector<Cfg>& U,
                                CollisionDetection*,LocalPlanners*,
                                DistanceMetric *, CNInfo&, LPInfo);

  static void ConnectSmallCCs(Roadmap*,CollisionDetection*,LocalPlanners*,
				DistanceMetric *, CN&, CNInfo&, VID, VID);
  static void ConnectBigCCs(Roadmap*,CollisionDetection*,LocalPlanners*,
				DistanceMetric *, CN&, CNInfo&, VID, VID);

  static vector< CfgPairType > FindKClosestPairs(
                Environment*, DistanceMetric *, CNInfo&, 
                vector<Cfg>&, vector<Cfg>&, int);

  static vector< CfgPairType > FindKClosestPairs(
                Environment*, DistanceMetric *, CNInfo&, 
                vector<Cfg>&, int);

  static vector< CfgPairType > FindKClosestPairs(
                Environment*, DistanceMetric *, CNInfo&, 
                Cfg&, vector<Cfg>&, int);

  static vector< Cfg_VE_Type > FindKClosestPairs(
                Roadmap*, DistanceMetric *, CNInfo&,
                Cfg&                              cfg,
                vector<Cfg>&                      verts,
                vector< pair<pair<VID,VID>,WEIGHT> >& edges,
                int                                   k,
                bool                              midpoint);

  static LPInfo Initialize_LPinfo(Roadmap * _rm,CNInfo& info);

  static vec_vec_CfgType Get_Cfgs_By_Obst(Roadmap * _rm);

  static bool DIST_Compare    (const DIST_TYPE&,    const DIST_TYPE&);
  static bool VE_DIST_Compare (const VE_DIST_TYPE&, const VE_DIST_TYPE&);
  static bool info_Compare    (const Cfg&,      const Cfg&);

  //===================================================================
  // Data
  //===================================================================
public:
  CNSets connectors;
  CNInfo cnInfo;

protected:
private:
};

#endif
