// $Id$

/**@file ConnectMapNodes.h
  *This set of classes supports a "RoadMap Node Connection Algobase".
  *Generate roadmap edges and enter each as a graph edge. 
  *
  *The classes in the set are:
  *	- CN  info related to an individual method of node connection 
  *	- CNSets contains 'master' list of all connection methods
  *       and maintains sets of connection methods
  *       (derived from BasicSets<CN>)
  *                           
  *	- ConnectMapNodes has CNSets as data element, and its methods
  *       include the actual connection algorithms.
  *       A 'wrapper' method cycles thru sets of cn's. 
  *
  *@author Daniel Vallejo
  *@date 8/27/98
  */

#ifndef ConnectMapNodes_h
#define ConnectMapNodes_h

#include "OBPRM.h"
#include "Sets.h"
#include "RoadmapGraph.h"

#include "util.h"
#include "CollisionDetection.h"
#include "LocalPlanners.h"

#include "GenerateMapNodes.h"
class GenerateMapNodes;

#include <stdlib.h>
#include <stdio.h>

#include <iostream.h>
#include <fstream.h>
#include <strstream.h>
#include <iomanip.h>

#include <vector.h>


//---------------------------------------------------------------
/**Pre-defined Algobase Sets.
  *@warning  DO NOT CHANGE THE ENUMERATION ORDERS w/o CHANGING
  *          SET DEFN's in "DefaultInit" method of 
  *          GenerateMapNodes class
  */
//---------------------------------------------------------------
enum cn_predefined {    //----------------
        RANDOM,         ///< Randomly find object
        CLOSEST10,      ///< Try closest 10 on every other object
        CLOSEST20,      ///< Try closest 20 on every other object
        CN_USER1        ///< First user defined cn set, if any
};

//---------------------------------------------------------------
/// Algo base information data structures
//---------------------------------------------------------------
struct CNInfo {
    SID cnsetid;                        ///< Connector set id
    SID lpsetid;                        ///< Local planner set id
    SID cdsetid;                        ///< Collision detection set id
    SID dmsetid;                        ///< Distance metric set id
    CDInfo cdInfo;
    vector <EdgeInfo<WEIGHT> > edges;
    bool addPartialEdge;
    int tag;                            ///< Nodes can be marked by user
    int dupeNodes,dupeEdges;            ///< used for acct'ing w/ closestVE
    GenerateMapNodes gn;                ///< how new cfg's should be generated

};

/// default cut-off for small CCs
#define SMALL_CC    3

class CN;
class CNSets;
class ConnectNodes;

class Roadmap;
class Environment;

const double MAX_DIST =  1e10;

# define DEFAULT_numEdges 5

///Pointer to cn function
typedef void (*CNF) (Roadmap* rm, CollisionDetection *,
			 LocalPlanners* ,DistanceMetric *, CN&, CNInfo&); 



/////////////////////////////////////////////////////////////////////
/**Store information for connecting nodes.
  *This class contains information relevant to a particular 
  *Connect Map Nodes strategy (for example, name, ptr to function, etc).
  */    
/////////////////////////////////////////////////////////////////////
class CN {
  friend class CNSets;
public:

  //===================================================================
  /**@name Constructors and Destructor*/
  //===================================================================
  //@{
  ///Default constructor. Initialize its data member to invalid values.
  CN();
  ///Destructor. Currently do nothing.
  ~CN();
  //@}

  //===================================================================
  /**@name Operator overlaoding*/
  //===================================================================
  //@{
  /**Compare name of given CN with anme of this instance.
    *@param _cn the name of this CN is compared with name of this instance.
    *@return True, if names of these two instance are the same. Otherwise False.
    */
  bool operator==(const CN & _cn) const;
  //@}
  
  //===================================================================
  /**@name Access Methods
    *All of these functions used for getting values of datamembers. 
    */
  //===================================================================
  //@{

  char*  GetName() const;	///<Retrive pointer for #name
  CNF    GetConnector();	///<Get #connector
  EID    GetID() const;		///<Get #cnid
  int  	 GetNumEdges   () const;///<Get #numEdges
  int    GetKClosest   () const;///<Get #kclosest
  int 	 GetSmallCCSize() const;///<Get #smallcc
  int 	 GetKPairs     () const;///<Get #kpairs
  int 	 GetKOther     () const;///<Get #k_other
  int 	 GetKSelf      () const;///<Get #k_self
  int 	 GetIterations () const;///<Get #iterations
  int 	 GetStepFactor () const;///<Get #stepFactor
  int    GetMaxNum     () const;///<Get #maxNum
  double GetRFactor    () const;///<Get #rfactor
  //@}

protected:
  //===================================================================
  // Data
  //===================================================================
  char   name[80];
  CNF    connector;         ///< ptr to connection strategy code
  EID    cnid;
  int 	 numEdges;          ///< used by random algm 
  int    kclosest;          ///< used by closest algm & modifiedLM
  int    maxNum;            ///< used by modifiedLM
  double rfactor;           ///< used by modifiedLM
  int 	 kpairs;            ///< used by components algm
  int 	 smallcc;           ///< used by components & rrt algm
  int 	 stepFactor;        ///< used by rrt algm
  int 	 iterations;        ///< used by rrt algm
  int 	 k_other;           ///< used by obstBased algm
  int 	 k_self;            ///< used by obstBased algm
 
private:
};

ostream& operator<< (ostream& _os, const CN& cn); 



/////////////////////////////////////////////////////////////////////
/**This class is derived from BasicSets<CN>.
  *This class manages/contains:
  *	- a 'master' list of all connection strategies, and
  *	- sets of connection strategies
  *
  *     Each connection strategy and set is given a unique id. The cn ids (EIDs)
  *     are used to compose labels encoding connection strategies sets. 
  */
/////////////////////////////////////////////////////////////////////
class CNSets : public BasicSets<CN> {
public:
  //===================================================================
  /**@name  Constructors and Destructor*/
  //===================================================================
  //@{
    CNSets();	///< Default constructor, do nothing currently.
    ~CNSets();  ///< Destructor, do nothing currently.
   //@}

  //===================================================================
  //  Other Methods
  //===================================================================

   /**@name Adding CNs, and CN sets.*/
   //@{
   int AddCN(const char* _cninfo);     ///< Add cn(s) to universe
   int AddCNToSet(const SID _sid, const EID _cnid);
   //@}
   
   /**@name Make CNSET
     *Make an ordered set of cns, add cn to universe if not there
     */
   //@{
   SID MakeCNSet(const char* cnlist);
   SID MakeCNSet(istream& _myistream);
   SID MakeCNSet(const EID _eid);
   SID MakeCNSet(const vector<EID> _eidvector);
   //@}

   /**@name Delete CN and CNSET*/
   //@{
   int DeleteCNFromSet(const SID _sid, const EID _cnid);
   int DeleteCNSet(const SID _sid);
   //@}

   /**@name Getting Data & Statistics.*/
   //@{
   CN GetCN(const EID _cnid) const;
   vector<CN> GetCNs() const;
   vector<CN> GetCNSet(const SID _sid) const;
   vector<pair<SID,vector<CN> > > GetCNSets() const;
   //@}

   /**@name Display CN and CNSet.*/
   //@{
   void DisplayCNs() const;
   void DisplayCN(const EID _cnid) const;
   void DisplayCNSets() const;
   void DisplayCNSet(const SID _sid) const;
   //@}
   
   /**@name Read/Write CN and CNSet.*/
   //@{
   void WriteCNs(const char* _fname) const;
   void WriteCNs(ostream& _myostream) const;
   void ReadCNs(const char* _fname);
   void ReadCNs(istream& _myistream);
   //@}

  //===================================================================
  //  Data
  //===================================================================
protected:
private:
};


/**Cfg VE Tyep.
  *@todo Not well documented. I don't know what this class for.
  */
class Cfg_VE_Type {
  public:
  
  /**@name Constructor and Destructor*/
  //@{
  // constructors
  Cfg_VE_Type();
  Cfg_VE_Type(Cfg& _cfg1,Cfg& _cfg2);
  Cfg_VE_Type(Cfg& _cfg1,Cfg& _cfg2, Cfg& _endpt1,Cfg& _endpt2);
    
  ///Destructor
  ~Cfg_VE_Type();
  //@}
    
  // data
  Cfg         cfg1,cfg2;
  bool        cfg2_IsOnEdge;
  vector<Cfg> endpt;
}; //End of Cfg_VE_Type
  

typedef vector < vector<Cfg> >  vec_vec_CfgType;
typedef pair   <Cfg,Cfg>    CfgPairType;
typedef pair   <CfgPairType,double> DIST_TYPE;
typedef pair   <Cfg_VE_Type,double> VE_DIST_TYPE;

/**ConnectMapNodes
  *This is the main connection strategy class. 
  *It has an CNSets object as a data element (list of cns and cn sets), 
  *and its methods include the actual connection strategy algorithms:
  *	- "random" -- random connections
  *
  *	- "closest" --  try to connect each node with the k closest nodes
  *
  *	- "obstBased" -- try to connect different obstacles 
  *		- note1: parameters are 
  *			-# k_other(#pairs to try between different obst)
  *			-# k_self (#pairs to try from each cfg to obst's own cfg list)
  *		- sample command line "-cNodes obstBased [k_other] [k_self]"
  *
  *	- "components" -- try to connect different connected components
  *		- note1: should be used after "closest"
  *		- note2: parameters are 
  *			-# kpairs (#pairs to try between big ccs) .
  *			-# smallcc (size of CCs if try all connections).
  *		- sample command line "-cNodes closest components [kpairs] [smallcc]".
  *
  *	- "RRTexpand" -- try to grow small connected components into larger ones
  *		- note1: should be used after some other method
  *		- note2: parameters are 
  *			-# iterations (of RRT algm).
  *			-# stepFactor (mult of environment determined "posres" to use).
  *			-# smallcc (grow CCs smaller/equal this size).
  *
  *	- "modifiedLM" -- modified Laumond's method. During connection phase,
  *           nodes are randomly generated, they are kept if they can be 
  *           connected to no CCs or more than one CCs, otherwise it will 
  *           be tossed away(only connected to one CC) if its 'distance' 
  *   	     from the 'center' of that CC not larger than a 'radial factor' 
  *           times the radius of that CC, 
  *           i.e, it will be kept only if it 'expand' that CC.
  *		- note:  parameters are 
  *			-# kclosest: num of closest nodes in each CC 
  *                                  that this node is going to try connection.
  *			-# maxNum:  the maximum numbers of nodes that are going 
  *                                 to be added into the roadmap during this.
  *			-# rfactor: given that every CC has a 'radius', here's 
  *                                 what the method does. if cfg.dist > r*radial_factor 
  *                                 then add cfg to map
  *
  *	- 'Wrapper' methods cycle thru sets of cns, stopping upon 1st success or trying all.
  ******************************************************************************************/

class ConnectMapNodes {
   friend SID MakeCNSet(istream&); 
public:
  //===================================================================
  /**@name Constructors and Destructor.*/
  //@{ 
  ConnectMapNodes();	///< Default constructor, which calls DefaultInit.
  ~ConnectMapNodes();	///< Destructor, do nothing currently.
  //@}

  //===================================================================
  /**@name Initialization Methods.*/
  //@{
  void DefaultInit();	///< Initialize members in #cninfo.
  void UserInit(Input * input, Environment * env); ///< Get more information from Input.
  //@}
  
  //===================================================================
  /**@name Helper Methods*/
  //@{
  typedef pair <Cfg,double> CfgDistType;
  static bool CfgDist_Compare (const CfgDistType&, const CfgDistType&);
  static void SortByDistFromCfg (Environment *_env,DistanceMetric *dm, 
				CNInfo& info, const Cfg& _cfg1, vector<Cfg>&  _cfgs);
  static void setConnectionResolution(double _posRes, double _oriRes);
  //@}	
  
  
  //===================================================================
  /**@name Connection Methods.*/
  //@{
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
  //@}

public:
  CNSets connectors;
  CNInfo cnInfo;

protected:
  static double connectionPosRes, connectionOriRes;
  
///////////////////////////////////////////////////////////////////////////////////////
// PRIVATE METHOD and DATA MEMBERS
// NO DOCUMENTATION YET
///////////////////////////////////////////////////////////////////////////////////////
private:

  static void ClosestVE(
        Roadmap * _rm,CollisionDetection* cd,
        LocalPlanners* lp,DistanceMetric * dm,
        CN& _cn, CNInfo& info, vector<Cfg>& oldV, vector<Cfg>& newV);

  static void ModifyRoadMap(Roadmap *toMap, Roadmap *fromMap, vector<VID> vids);

  static void RRT(Roadmap* rm, int K, double deltaT, vector<Cfg>& U,
                                CollisionDetection*,LocalPlanners*,
                                DistanceMetric *, CNInfo&, LPInfo);

  static void ConnectSmallCCs(Roadmap*,CollisionDetection*,LocalPlanners*,
				DistanceMetric *, CN&, CNInfo&, VID, VID);
  static void ConnectBigCCs(Roadmap*,CollisionDetection*,LocalPlanners*,
				DistanceMetric *, CN&, CNInfo&, VID, VID);

  static vector< pair<VID, VID> > FindKClosestPairs(
		Roadmap *rm, DistanceMetric * dm, CNInfo& info,
        	vector<Cfg>& vec1, int k);
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

private:
};

#endif
