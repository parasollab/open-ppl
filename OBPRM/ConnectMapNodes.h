// $Id$

//////////////////////////////////////////////////////////////////////////////////////////
/**@file ConnectMapNodes.h
  *This set of classes supports a "RoadMap Node Connection Algobase".
  *Generate roadmap edges and enter each as a graph edge. 
  *
  *The classes in the set are:
  * - CN  info related to an individual method of node connection 
  * - CNSets contains 'master' list of all connection methods
  *       and maintains sets of connection methods
  *       (derived from BasicSets<CN>)
  *                           
  * - ConnectMapNodes has CNSets as data element, and its methods
  *       include the actual connection algorithms.
  *       A 'wrapper' method cycles thru sets of cn's. 
  *
  *@author Daniel Vallejo
  *@date 8/27/98
  */
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef ConnectMapNodes_h
#define ConnectMapNodes_h


////////////////////////////////////////////////////////////////////////////////////////////
// Include OBPRM headers
#include "RoadmapGraph.h"
#include "LocalPlanners.h"
#include "GenerateMapNodes.h"


//---------------------------------------------------------------
/**Pre-defined Algobase Sets.
  *@warning  DO NOT CHANGE THE ENUMERATION ORDERS w/o CHANGING
  *          SET DEFN's in "DefaultInit" method of 
  *          GenerateMapNodes class
  *
  *@see ConnectMapNodes::DefaultInit. The order for set creations
  *is the same as the order here. (i.e. the enumeration values
  *here is set id!!). (ex: RANDOM (0) is CN set id for "random"
  *in ConnectMapNodes::DefaultInit)
  */
//---------------------------------------------------------------
enum cn_predefined {
        RANDOM,         ///< Randomly find object
        CLOSEST10,      ///< Try closest 10 on every other object
        CLOSEST20,      ///< Try closest 20 on every other object
        CN_USER1        ///< First user defined cn set, if any
};

//////////////////////////////////////////////////////////////////////////////////////////
//
//
// Algo base information data structures
//
//
//////////////////////////////////////////////////////////////////////////////////////////

struct CNInfo {

    SID cnsetid;                        ///< Connector set id
    SID lpsetid;                        ///< Local planner set id
    SID cdsetid;                        ///< Collision detection set id
    SID dmsetid;                        ///< Distance metric set id
    CDInfo cdInfo;

    vector <EdgeInfo<WEIGHT> > edges;   ///< This seems deprecated?!
    bool addPartialEdge;                ///< Add failed path to roadmap or not.
    int tag;                            ///< Nodes can be marked by user
    int dupeNodes,dupeEdges;            ///< used for acct'ing w/ closestVE
    GenerateMapNodes gn;                ///< how new cfg's should be generated

};

/////////////////////////////////////////////////////////////////////
/**@class CN
  *Store information for connecting nodes.
  *This class contains information relevant to a particular 
  *Connect Map Nodes strategy (for example, name, ptr to function, etc).
  */    
/////////////////////////////////////////////////////////////////////

/// default cut-off for small CCs
#define SMALL_CC    3

class CN;
class CNSets;
class ConnectNodes;

class Roadmap;
class Environment;

const double MAX_DIST =  1e10;

# define DEFAULT_numEdges 5

///Pointer to map-node connection function
typedef void (*CNF) (Roadmap* rm, CollisionDetection *,
             LocalPlanners* ,DistanceMetric *, CN&, CNInfo&); 

class CN {

  friend class CNSets;

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

      ///Default constructor. Initialize its data member to invalid values.
      CN();
      ///Destructor. Currently do nothing.
      ~CN();

   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Operator Overloading
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Operator Overloading*/
   //@{

      /**Compare name of given CN with name of this instance.
        *If the names of two CN instances are different, false will be returned.
        *Otherwise following rules will be applied:
        *   -# random return true if #numEdges are the same.
        *   -# closest return true if #kclosest are the same.
        *   -# obstBased return true if #k_other are the same 
        *      and #k_self are the same.
        *   -# RRTexpand return true if #iterations , #stepFactor,
        *      and #smallcc between two CN instances are the same.
        *   -# RRTcomponents return true if #iterations ,
        *      #stepFactor, and #smallcc between two CN instances
        *      are the same.
        *   -# components return true if #kpairs are the same and 
        *      #smallcc are the same.
        *   -# modifiedLM return true if #kclosest, #maxNum, and
        *      #rfactor between two CN instances are the same.
        *
        *@param _cn the name of this CN is compared with name of this instance.
        *@@return true if above rules returns true, otherwise false will bereturned.
        */
      bool operator==(const CN & _cn) const;

  //@}
  
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Access Methods.
    *All of these functions used for getting values of datamembers.
    */
  //@{

      inline char*  GetName() const;   ///<Retrive pointer for #name
      inline CNF &  GetConnector();    ///<Get #connector
      inline EID    GetID() const;     ///<Get #cnid
      inline int    GetNumEdges   () const;///<Get #numEdges
      inline int    GetKClosest   () const;///<Get #kclosest
      inline int    GetSmallCCSize() const;///<Get #smallcc
      inline int    GetKPairs     () const;///<Get #kpairs
      inline int    GetKOther     () const;///<Get #k_other
      inline int    GetKSelf      () const;///<Get #k_self
      inline int    GetIterations () const;///<Get #iterations
      inline int    GetStepFactor () const;///<Get #stepFactor
      inline int    GetMaxNum     () const;///<Get #maxNum
      inline double GetRFactor    () const;///<Get #rfactor

  	  void SetNumEdges(int cEdge){numEdges=cEdge;} ///<Set #numEdges
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

  char   name[80];
  CNF    connector;         ///< ptr to connection strategy code
  EID    cnid;

  int    numEdges;          ///< used by random algm 

  int    kclosest;          ///< used by closest algm & modifiedLM

  int    maxNum;            ///< used by modifiedLM
  double rfactor;           ///< used by modifiedLM

  int    kpairs;            ///< used by components algm
  int    smallcc;           ///< used by components & rrt algm

  int    stepFactor;        ///< used by rrt algm
  int    iterations;        ///< used by rrt algm

  int    k_other;           ///< used by obstBased algm
  int    k_self;            ///< used by obstBased algm
 
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:
};

/**Output information depends on the name of CN instance.
  *Following rules will be applied:
  * -# if name=random, then print CN::numEdges.
  * -# if name=kclosest, then print CN::kclosest.
  * -# if name=obstBased, then print CN::k_other and CN::k_self.
  * -# if name=RRTexpand, then print CN::stepFactor, CN::iterations,
  *       and CN::smallcc.
  * -# if name=modifiedLM, then print CN::maxNum, CN::rfactor
  *       and CN::kclosest
  * -# if name=components, then print CN::kpairs, and CN::smallcc
  */
ostream& operator<< (ostream& _os, const CN& cn); 

//////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//
//  class CNSets
//
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
/**This class is derived from BasicSets<CN>.
  *This class manages/contains:
  * - a 'master' list of all connection strategies, and
  * - sets of connection strategies
  *
  *     Each connection strategy and set is given a unique id. The cn ids (EIDs)
  *     are used to compose labels encoding connection strategies sets. 
  */
/////////////////////////////////////////////////////////////////////
class CNSets : public BasicSets<CN> {
public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destrcutor.
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Constructors and Destructor */
  //@{

    CNSets();   ///< Default constructor, do nothing currently.
    ~CNSets();  ///< Destructor, do nothing currently.

  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Adding CNs, Making & Modifying CN sets
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Adding CNs Making & Modifying CN sets */
  //@{

       /**Add node connection info to universe.
         *@note this method just adds CN instances to universe 
         *, and no set will be created.
         *@see MakeCNSet(const char*)
         */
       int AddCN(const char* _cninfo);

       /**Add an element to (ordered) CN set.
         *@see BasicSets::AddElementToOSet
         */
       int AddCNToSet(const SID _sid, const EID _cnid);

       /**Remove element from (ordered) CN set.
         *@see BasicSets::DeleteElementFromOSet
         */
       int DeleteCNFromSet(const SID _sid, const EID _cnid);

       /**Read node generation info from a given string.
         *@return INVALID_SID if istrstream for this given
         *string could not be created.
         *@see MakeCNSet(istream& _myistream)
         */
       SID MakeCNSet(const char* cnlist);

       /**Read node generation info from input stream,
         *create CN instances for these info, and make an (ordered) CN set for these.
         *Accroding to read-in CN "names", following rules are applied.
         *
         *  -# random
         *      -# CN::connector = ConnectMapNodes::ConnectNodes_Random 
         *      -# CN::numEdges = user spcified value (should larger than zero)
         *         (if no user spcified value, default value is DEFAULT_numEdges )
         *      -# EX: random 5 10 15 
         *         (create 3 CNs, random 5, random 10, and random 15)
         *
         *  -# closest
         *      -# CN::connector = ConnectMapNodes::ConnectNodes_Closest 
         *      -# CN::kclosest = user spcified value (should larger than zero)
         *         (if no user spcified value, default value is 5 )
         *      -# EX: closest 2 6
         *         (create 3 CNs, closest 2 and closest 6)
         *
         *  -# closestVE
         *      -# CN::connector = ConnectMapNodes::ConnectNodes_ClosestVE 
         *      -# CN::kclosest = user spcified value (should larger than zero)
         *         (if no user spcified value, default value is 5 )
         *      -# EX: closestVE 2 6
         *         (create 3 CNs, closestVE 2 and closestVE 6)
         *
         *  -# components
         *      -# CN::connector = ConnectMapNodes::ConnectNodes_ConnectCCs 
         *      -# CN::kpairs = user spcified value (should larger than zero)
         *         (if no user spcified value, default value is 4 )
         *      -# CN::smallcc = user spcified value (should larger than zero)
         *         (if no user spcified value, default value is SMALL_CC )
         *      -# EX: components 4 3 (create 1 CNs, kpairs=4 and smallcc=3 )
         *
         *  -# obstBased
         *      -# CN::connector = ConnectMapNodes::ConnectNodes_ObstBased 
         *      -# CN::k_other = user spcified value (should larger than zero)
         *         (if no user spcified value, default value is K_OTHER )
         *      -# CN::k_self = user spcified value (should larger than zero)
         *         (if no user spcified value, default value is K_SELF )
         *      -# EX: obstBased 10 3 (create 1 CNs, k_other=10, k_self=3)
         *
         *  -# RRTexpand
         *      -# CN::connector = ConnectMapNodes::ConnectNodes_ExpandRRT 
         *      -# CN::iterations = user spcified value (should larger than zero)
         *         (if no user spcified value, default value is ITERATIONS )
         *      -# CN::stepFactor = user spcified value (should larger than zero)
         *         (if no user spcified value, default value is STEP_FACTOR )
         *      -# CN::smallcc = user spcified value (should larger than zero)
         *         (if no user spcified value, default value is SMALL_CC )
         *      -# EX: RRTexpand 10 3 3
         *             (create 1 CNs, iterations=10, stepFactor=3, smallcc=3)
         *
         *  -# modifiedLM
         *      -# CN::connector = ConnectMapNodes::ConnectNodes_modifiedLM 
         *      -# CN::kclosest = user spcified value (should larger than zero)
         *         (if no user spcified value, default value is 5 )
         *      -# CN::maxNum = user spcified value (should larger than zero)
         *         (if no user spcified value, default value is MAX_NUM )
         *      -# CN::rfactor = user spcified value (should larger than zero)
         *         (if no user spcified value, default value is RFACTOR )
         *      -# EX: modifiedLM 5 20 2
         *             (create 1 CNs, kclosest=5, maxNum=20, rfactor=2)
         *
         *@return SID of new set if every thing is OK. Otherwise, process will be terminiated.
         *@see BasicSets::MakeOSet 
         */
       SID MakeCNSet(istream& _myistream);

       /**Make a new (ordered) CN set with element _eid.
         *@see BasicSets::MakeOSet(const EID _eid)
         */
       SID MakeCNSet(const EID _eid);

       /**Make a new (ordered) CN cd set with a list of elements in _eidvector.
         *@see BasicSets::MakeOSet 
         */
       SID MakeCNSet(const vector<EID> _eidvector);

       /**Remove a (ordered) CN set from universe.
         *@see BasicSets::DeleteOSet
         */
       int DeleteCNSet(const SID _sid);

  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Method (Getting Data & Statistics)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Access Methods.
    *Getting Data & Statistics 
    */
   //@{

       /**Get a CN instance from universe.
         *@note _cnid should be added to universe before.
         *@see BasicSets::GetElement
         */
       CN GetCN(const EID _cnid) const;

       /**Get all CN instances in universe.
         *@return a list of CN instances.
         *@see BasicSets::GetElements
         */
       vector<CN> GetCNs() const;

       /**Get a (ordered) CN set from universe.
         *@note _sid should be created and added to universe before.
         *@see BasicSets::GetOSet(const SID _sid)
         */
       vector<CN> GetCNSet(const SID _sid) const;

       /**Get all (ordered) CN set in universe.
         *@return a list of (ordered) CN sets and their SIDs.
         *Each (ordered) CN set contains a list of CN instances.
         *@see BasicSets::GetOSets
         */
       vector<pair<SID,vector<CN> > > GetCNSets() const;

   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O Method (Display, Input, Output)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name I/O Methods.
     *Display, Input, Output 
     */
   //@{ 

       /**Output all CN instances info in universe
         *to standard output.
         *@see BasicSets::DisplayElements.
         */
       void DisplayCNs() const;

       /**Output CN instance info to standard output.
         *@param _lpid Specify which CN instance should be printed.
         *@see BasicSets::DisplayElement.
         */
       void DisplayCN(const EID _cnid) const;

       /**Output information of all (ordered) CN sets in universe.
         *@see BasicSets::DisplayOSets
         */
       void DisplayCNSets() const;

       /**Output information of (ordered) CN set with _sid.
         *@see BasicSets::DisplayOSet
         */
       void DisplayCNSet(const SID _sid) const;

       /**Ouput information about all CN instances to file.
         *@param _fname filename for data file.
         *@see WriteCNs(ostream& _myostream)
         */
       void WriteCNs(const char* _fname) const;

       /**Ouput information about all CN instances to output stream.
         *@note format: CN_NAME (a string) CN_PARMS (double, int, etc) 
         *for each CN instance.
         *
         *@note Acturally, this method outputs name of CN instances only.
         *@see GetCNs 
         */
       void WriteCNs(ostream& _myostream) const;

       /**Read information about CN instances from file.
         *@param _fname filename for data file.
         *@see ReadCNs(istream& _myistream)
         */
       void ReadCNs(const char* _fname);

       /**Read information about CN instances from input stream.
         *This method reads and adds CN instances to universe.
         *@see AddCN for add CN instances to universe and WriteCNs 
         *for data format
         *@note if this method could not be able to understand
         *input file format, then error message will be post to 
         *standard output.
         */
       void ReadCNs(istream& _myistream);

   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:
    
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private Data members and Member methods
  //
  //
  /////////////////////////////////////////////////////////////////////////////////////////
private:
};

//////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//
//  class ConnectMapNodes & Cfg_VE_Type
//
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

/**@class ConnectMapNodes
  *This is the main connection strategy class. 
  *It has an CNSets object as a data element (list of cns and cn sets), 
  *and its methods include the actual connection strategy algorithms:
  * - "random" -- random connections
  *
  * - "closest" --  try to connect each node with the k closest nodes
  *
  * - "obstBased" -- try to connect different obstacles 
  *     - note1: parameters are 
  *         -# k_other(#pairs to try between different obst)
  *         -# k_self (#pairs to try from each cfg to obst's own cfg list)
  *     - sample command line "-cNodes obstBased [k_other] [k_self]"
  *
  * - "components" -- try to connect different connected components
  *     - note1: should be used after "closest"
  *     - note2: parameters are 
  *         -# kpairs (#pairs to try between big ccs) .
  *         -# smallcc (size of CCs if try all connections).
  *     - sample command line "-cNodes closest components [kpairs] [smallcc]".
  *
  * - "RRTexpand" -- try to grow small connected components into larger ones
  *     - note1: should be used after some other method
  *     - note2: parameters are 
  *         -# iterations (of RRT algm).
  *         -# stepFactor (mult of environment determined "posres" to use).
  *         -# smallcc (grow CCs smaller/equal this size).
  *
  * - "modifiedLM" -- modified Laumond's method. During connection phase,
  *           nodes are randomly generated, they are kept if they can be 
  *           connected to no CCs or more than one CCs, otherwise it will 
  *           be tossed away(only connected to one CC) if its 'distance' 
  *          from the 'center' of that CC not larger than a 'radial factor' 
  *           times the radius of that CC, 
  *           i.e, it will be kept only if it 'expand' that CC.
  *     - note:  parameters are 
  *         -# kclosest: num of closest nodes in each CC 
  *                                  that this node is going to try connection.
  *         -# maxNum:  the maximum numbers of nodes that are going 
  *                                 to be added into the roadmap during this.
  *         -# rfactor: given that every CC has a 'radius', here's 
  *                                 what the method does. if cfg.dist > r*radial_factor 
  *                                 then add cfg to map
  *
  * - 'Wrapper' methods cycle thru sets of cns, stopping upon 1st success or trying all.
  ******************************************************************************************/

/**Cfg VE Tyep.
  *@todo Not well documented. I don't know what this class for.
  */
class Cfg_VE_Type {
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
      Cfg_VE_Type();

      /**Constructor.
        *   -# #cfg1 = _cfg1
        *   -# #cfg2 = _cfg2
        *   -# #cfg2_IsOnEdge = false
        */
      Cfg_VE_Type(Cfg& _cfg1,Cfg& _cfg2);

      /**Constructor.
        *   -# #cfg1 = _cfg1
        *   -# #cfg2 = _cfg2
        *   -# #cfg2_IsOnEdge = true
        *   -# #endpt = (_endpt1,_endpt2)
        */
      Cfg_VE_Type(Cfg& _cfg1,Cfg& _cfg2, Cfg& _endpt1,Cfg& _endpt2);
    
      ///Destructor. Do nothing.
      ~Cfg_VE_Type();

  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  Cfg         cfg1, ///< Start of an edge
              cfg2; ///< End of an edge
  ///True if cfg2 is on line segment defined by #endpt.
  bool        cfg2_IsOnEdge;
  ///Defines a line segment where cfg2 is. (Defined only if #cfg2_IsOnEdge is true.)
  vector<Cfg> endpt;

}; //End of Cfg_VE_Type
  

typedef vector < vector<Cfg> >      vec_vec_CfgType;    ///< 2D vecoter Cfg
typedef pair   <Cfg,Cfg>            CfgPairType;        ///< A pair if Cfgs
typedef pair   <CfgPairType,double> DIST_TYPE;          ///< Distance between 2 Cfgs
typedef pair   <Cfg_VE_Type,double> VE_DIST_TYPE;       ///< Distance between 2 Cfgs defined in Cfg_VE_Type

class ConnectMapNodes {

   friend SID MakeCNSet(istream&); 

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

      ConnectMapNodes();    ///< Default constructor, which calls DefaultInit.
      ~ConnectMapNodes();   ///< Destructor, do nothing currently.

  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Initialization functions
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Initialization Methods*/
  //@{

      /**Initialize members in #cninfo.
        *Default values for #cninfo are:
        *   -# CNInfo::cnsetid = CLOSEST10
        *   -# CNInfo::lpsetid = SL_R5
        *   -# CNInfo::dmsetid = S_EUCLID9
        *   -# CNInfo::cdsetid = CSTK or RAPID
        *      (depends on USE_CSTK defined or not)
        *   -# CNInfo::dupeEdges = 0
        *   -# CNInfo::dupeNodes = 0
        */
      void DefaultInit();

      /**Initialize this instance accroding to user input.
        *Following CN Sets are created.
        *   -# random
        *   -# closest 10
        *   -# closest 20
        *In addition, Sets will be created according to user 
        *input strings. (one string for one set).
        *
        *CNInfo instance are also initailized here as following:
        *   -# CNInfo::addPartialEdge = Input::addPartialEdge
        *
        *Data member initialized as following:
        *   -# #connectionPosRes= Environment::GetPositionRes
        *   -# #connectionOriRes= Environment::GetOrientationRes
        *
        *@note if user input strings are not empty, 
        *#cnInfo.cnsetid will be set to CN_USER1
        *
        *@see CN::MakeCNSet(istream&) and ValidateParameters
        */
      virtual void UserInit(Input * input, Environment * env); ///< Get more information from Input.

  //@}
  
  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Staic Methods: Helper Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Helper Methods
    */
  //@{

      /**A distance associated with a Cfg.
        *@note used in SortByDistFromCfg and CfgDist_Compare only.
        * This looks like kind of redundancy!?
        *@see DIST_TYPE for ANOTHER distance type.
        */
      typedef pair <Cfg,double> CfgDistType;

      /**Compare distances in 2 CfgDistType instances.
        *@note used to sort cfgs by distance (CfgDistType is single cfg & distance)
        */
      static bool CfgDist_Compare(const CfgDistType&, const CfgDistType&);

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
      static void 
      SortByDistFromCfg(Environment *_env,
                        DistanceMetric *dm, 
                        CNInfo& info, 
                        const Cfg& _cfg1, 
                        vector<Cfg>&  _cfgs);

      /**Set/Update Position and Orientation Resolution of this instance.
        *@param _posRes Resoultion for Position
        *@param _oriRes Resoultion for Resolution
        *
        *The data member #connectionPosRes and #connectionOriRes
        *will be changed to given values.
        */
      static void setConnectionResolution(double _posRes, double _oriRes);

  //@}  
  
  
  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    High Level Node Connection functions
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name High Level Node Connection.
    *High level node connection methods.
    *may call more than one "Actual Node Connection Heuristic"
    *in sets 
    */
  //@{

      /**Generate edges for roadmap according to all CNs in the set.
        *This is a high level view of ConnectNodes, which 
        *retrives actural node connection funtions from speicified
        *CNSet.
        *In this speicified CNSet, CN(s) contains pointer to 
        *a low level node connection funtion and parameters for
        *these low level funtions.
        *The job of this method is to:
        *   -# Get speicified CNSet
        *   -# Call low level node connection funtions in
        *      CNs in this speicified GNSet.
        *
        *@param rdmp New created edges will be added to this roadmap.
        *@param _cnsetid Which set of CN will be used in node connection time.
        *@param info Record node connection process infomation.
        *@see RoadmapGraph::AddEdges
        */
      virtual void ConnectNodes(Roadmap * rdmp,
             CollisionDetection *cd,LocalPlanners* lp, DistanceMetric * dm, 
             SID _cnsetid, CNInfo &info);

      /**Generate edges for roadmap according to all CNs in the set.
        *This method creates a Roadmap instance using given Environment
        *and RoadmapGraph, and then 
        *ConnectNodes(Roadmap *,CollisionDetection *,LocalPlanners*, DistanceMetric *, 
        *SID, CNInfo&) is called.
        *
        *@note New created edges are inserted into given RoadmapGraph, roadmap.
        */
      virtual void ConnectNodes(Environment * environment, RoadmapGraph<Cfg,WEIGHT>& roadmap,
             CollisionDetection *cd,LocalPlanners* lp,
             DistanceMetric * dm, SID _cnsetid, CNInfo &info);

  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Staic Methods: CORE Node Connection functions
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name CORE Node Connection functions.
    *Actual node gernation methods.
    */
  //@{
      /**Connect nodes in map Totally Randomly.
        *Following Algorithm is used:
        *   -# loop following CN::GetNumEdges times
        *       -# node1 = random node in map, _rm
        *       -# node2 = random node in map, _rm
        *       -# lp_set is a random local planner set
        *       -# using local planning functions in lp_set
        *          to connect node1 and node2
        *       -# if connected, add this edge to map, _rm.
        *
        *@param info provides inforamtion other than connection, like
        *collision dection, local planner, and distance metrics.
        *@param _cn provides information for specific node connection 
        *paramters.
        *@param lp Local planner for connecting given 2 Cfgs.
        *@see RoadmapGraph::AddEdge and LocalPlanners::IsConnected
        */
      static void ConnectNodes_Random(Roadmap * _rm,
                                      CollisionDetection* cd,
                                      LocalPlanners* lp,
                                      DistanceMetric * dm,
                                      CN& _cn, CNInfo& info);

      /**Connect nodes in map to their k closest neighbors.
        *Following Algorithm is used:
        *   -# for evry node, cfg1, in roadmap
        *       -# find k closet neighbors for cfg1
        *       -# lp_set is a local planner set defined in info.lpsetid
        *       -# for every node, cfg2, in k-closest neighbor list for cfg1
        *           -# using local planning functions in lp_set
        *              to connect cfg1 and cfg2
        *           -# if connected, add this edge to map, _rm.
        *       -#end for
        *   -# end for
        *
        *@param info provides inforamtion other than connection, like
        *collision dection, local planner, and distance metrics.
        *@param _cn provides information for specific node connection 
        *paramters.
        *@param lp Local planner for connecting given 2 Cfgs.
        *
        *@see RoadmapGraph::AddEdge and LocalPlanners::IsConnected
        */
      static void ConnectNodes_Closest(Roadmap * _rm,
                                       CollisionDetection* cd,
                                       LocalPlanners* lp,
                                       DistanceMetric * dm,
                                       CN& _cn, CNInfo& info);

      /**Connect nodes in map to their k closest neighbors, which
        *could be vertex or point on edges in roadmap.
        *
        *This method not only creates edges, but creates new verteices 
        *also. New verteices are always on the existing edges.
        *
        *@param info provides inforamtion other than connection, like
        *collision dection, local planner, and distance metrics.
        *@param _cn provides information for specific node connection 
        *paramters.
        *@param lp Local planner for connecting given 2 Cfgs.
        *
        *@see ClosestVE for more information.
        */
      static void ConnectNodes_ClosestVE(Roadmap * _rm,
                                         CollisionDetection* cd,
                                         LocalPlanners* lp,
                                         DistanceMetric * dm,
                                         CN& _cn, CNInfo& info);

      /**Try to connect different connected components of the roadmap.
        *We try to connect all pairs of connected components. If both
        *components are small (less than "smallcc" nodes), then we try to 
        *connect all pairs of nodes.  If at least one of the components is 
        *large, we try to connect the "kpairs" closest pairs of nodes.
        *
        *@see WeightedGraph ::GetCCStats, WeightedGraph ::GetCC, 
        *WeightedGraph ::IsSameCC for information about connected 
        *component in graph,and ConnectSmallCCs, ConnectBigCCs for 
        *connecting between connected component.
        */
      static void ConnectNodes_ConnectCCs(Roadmap * _rm,
                                          CollisionDetection* cd,
                                          LocalPlanners* lp,
                                          DistanceMetric * dm,
                                          CN& _cn, CNInfo& info);

      /**Connect nodes depends on Obstacle information.
        *Obstacle to Obstacle connections are attempted for 
        *the "k" closest nodes.
        *Each "body" of a multibody is considered an obstacle.
        *Obstacles have id's. The k-closest cfg's, one of which 
        *was generated wrt the "first" obstacle (body_i) and 
        *the other of which was generated wrt the "second" (body_j), 
        *will have a connection attempted between them.
        *This continues for all unique id pairings.
        *For example:
        *Given obstacle id's 2,3,4 & 5,
        *connection is attempted for the k-closest in each obst-obst
        *(i,j) pairing below:
        *   - 2-2  2-3  2-4  2-5
        *   - 3-3  3-4  3-5
        *   - 4-4  4-5
        *   - 5-5
        *When i=j, the "k" value may be different than otherwise
        *
        *Following Algorithm is used:
        *   -# classify all nodes in roadmap by their obstacle id
        *      (obstacle id is assignment in node generation time
        *       to InfoCfg::obst in every Cfg)
        *   -# for each obstacle id, i
        *       -# for each obstacle id, j ,here j>=i
        *       -# find k closest pairs between Cfgs haveing obs_id=i
        *          and Cfgs haveing obs_id=j
        *       -# lp_set is a local planner set defined in info.lpsetid
        *       -# for every pair, i
        *           -# using local planning functions in lp_set
        *              to connect pair.first and pair.second
        *              (pair.first has obs_id=i and pair.second has obs_id=j)
        *           -# if connected, add this edge to map, _rm.
        *       -#end for
        *   -# end for
        *
        *For k value used in above algorithm:
        *if i!=j, then k=CN::GetKOther. Otherwise, k=GetKSelf.
        *
        *@param info provides inforamtion other than connection, like
        *collision dection, local planner, and distance metrics.
        *@param _cn provides information for specific node connection 
        *paramters.
        *@param lp Local planner for connecting given 2 Cfgs.
        *
        *@see RoadmapGraph::AddEdge and LocalPlanners::IsConnected,
        *and Get_Cfgs_By_Obst for classifing Cfgs.
        */
      static void ConnectNodes_ObstBased(Roadmap * _rm,
                                         CollisionDetection* cd,
                                         LocalPlanners* lp,
                                         DistanceMetric * dm,
                                         CN& _cn, CNInfo& info);

      /**Using RRT to expand or connect connected compnents.
        *This method handles both RRTexpand and RRTcomponents.
        *Follow algorithm is for RRTexpand:
        *   -# for every connected components, cc, in roadmap.
        *       -# if cc's size < CN::GetSmallCCSize
        *           -# then using RRT to expand cc randomly
        *              (i.e. parameter U in RRT is empty)
        *       -# end if
        *   -# end for
        *
        *Follow algorithm is for RRTcomponents:
        *   -# for every connected components, cc1, in roadmap, which
        *      has size smaller than CN::GetSmallCCSize
        *       -# for every connected components, cc2,in roadmap. Here cc1!=cc2.
        *           -# then using RRT to expand cc1 using Cfgs in cc2 as directions.
        *              (i.e. parameter U in RRT is Cfgs in cc2)
        *       -# end for
        *   -# end for
        *
        *@param info provides inforamtion other than connection, like
        *collision dection, local planner, and distance metrics.
        *@param _cn provides information for specific node connection 
        *paramters.
        *@param lp Local planner for connecting given 2 Cfgs.
        *
        *@see RRT.
        */
      static void ConnectNodes_ExpandRRT(Roadmap * _rm,
                                         CollisionDetection* cd,
                                         LocalPlanners* lp,
                                         DistanceMetric * dm,
                                         CN& _cn, CNInfo& info);

      /**"modifiedLM" -- modified Laumond's method. 
        *During connection 
        *phase, nodes are randomly generated, they are kept if they can 
        *be connected to no CCs or more than one CCs, otherwise it will 
        *be tossed away(only connected to one CC) if its 'distance' from 
        *the 'center' of that CC not larger than user-specified 'r' times 
        *the radius of that CC, i.e, it will be kept only when it 
        *'expand's that CC.
        *
        * Parameters for this connector:
        *
        *   -# kclosest: num of closest nodes in each CC that this node is
        *                going to try connection.
        *   -# maxNum: the maximum numbers of nodes that are going to be
        *              added into the roadmap during this.
        *   -# rfactor: multiplier for 'radius' of CC, w/in which thrown out, 
        *               outside of which kept.
        *
        *Alogorithm:
        *
        *   -# while (more than one CC remains *AND* added fewer new Cfgs than requested)
        *       -# generate a random configuration, cfg
        *       -# get current connected components from roadmap
        *       -# for each connected component, CC
        *           -# if possible to connect cfg to CC
        *               -# increment count of connections, # of connections
        *           -# endif
        *       -# endfor
        *       -# if(# of connections is zero *OR* # of connections greater than one )
        *          *OR*
        *          (#connections is one *AND* cfg distance to CCcenter > rfactor * CCradius)
        *           -# increment count of new Cfgs added 
        *           -# add cfg & all edges
        *       -# endif
        *   -# end while
        *
        *@param info provides inforamtion other than connection, like
        *collision dection, local planner, and distance metrics.
        *@param _cn provides information for specific node connection 
        *paramters.
        *@param lp Local planner for connecting given 2 Cfgs.
        *
        *@see RoadmapGraph::AddEdge and LocalPlanners::IsConnected,
        */
      static void ConnectNodes_modifiedLM(Roadmap * _rm,
                                          CollisionDetection* cd,
                                          LocalPlanners* lp,
                                          DistanceMetric * dm,
                                          CN& _cn, CNInfo& info);
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
public:
  CNSets connectors;
  CNInfo cnInfo;

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Static: Closest Vertex Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Closest Edge Methods*/
  //@{
	/**
	 *Connect all cfgs in vec1 to cfgs in vec2.
     *Following Algorithm is used:
     *   -# for evry node, cfg1, in vec1
     *       -# find k closet neighbors in vec2 for cfg1
     *       -# lp_set is a local planner set defined in info.lpsetid
     *       -# for every node, cfg2, in k-closest neighbor list for cfg1
     *           -# using local planning functions in lp_set
     *              to connect cfg1 and cfg2
     *           -# if connected, add this edge to map, _rm.
     *       -#end for
     *   -# end for
     *
	 */
	static void ConnectNodes_Closest(Roadmap * _rm,
		                             CollisionDetection* cd,
									 LocalPlanners* lp,
									 DistanceMetric * dm,
									 CN& _cn, CNInfo& info,
									 vector<Cfg>& vec1,
									 vector<Cfg>& vec2,
									 const int kclosest);
  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Static: Closest Vertex & Edge Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Closest Vertex & Edge Methods*/
  //@{
  
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
      static void ClosestVE(
            Roadmap * _rm,CollisionDetection* cd,
            LocalPlanners* lp,DistanceMetric * dm,
            CN& _cn, CNInfo& info, vector<Cfg>& oldV, vector<Cfg>& newV);

  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Static: Connect CC Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Connect CC Methods*/
  //@{
	  /**
	   *Try to connect connected components in ccs1 to connected components
	   *in ccs2.
       *@see WeightedGraph ::GetCCStats, WeightedGraph ::GetCC, 
       *WeightedGraph ::IsSameCC for information about connected 
       *component in graph,and ConnectSmallCCs, ConnectBigCCs for 
       *connecting between connected component.
       */
      static void ConnectNodes_ConnectCCs(Roadmap * _rm,
                                          CollisionDetection* cd,
                                          LocalPlanners* lp,
                                          DistanceMetric * dm,
                                          CN& _cn, CNInfo& info,
										  vector< pair<int,VID> > & ccs1,
										  vector< pair<int,VID> >  ccs2);
  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Static: RRT Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Rapidly-Exploring Random Tree Methods*/
  //@{

      /**Copy vertices and all incident edges associated with "vids" 
        *from one roadmap to another.
        *@param toMap Target, Cfgs in vids and incident edges in fromMap  
        *will be copied to this submap.
        *@param fromMap Source, edge information will be retrived from here.
        *@param vids Source, vertex information will be retrived from here.
        *Usually, in this list, elements are Cfgs in same connected component.
        */
      static void ModifyRoadMap(Roadmap *toMap, Roadmap *fromMap, vector<VID> vids);

      /**Implements Rapidly-Exploring Random Tree (RRT) algm.
        *initial tree - "Roadmap", input parameter
        *
        *Algm explores starting from initial tree input and adds whatever 
        *nodes and edges it explores as vertices and edges to roadmap.
        *
        *Following is algorithm for RRT:
        *   -# loop following K times.
        *       -# Randomly generate a Cfg, randCfg
        *       -# Find randCfg's cloest Cfg in tree, nearCfg
        *       -# Pick a random diection, u, in U. If U is empty
        *          use randCfg as a direction.
        *       -# Create a new Cfg, newCfg, in direction, u, having
        *          deltaT distance away from nearCfg.
        *       -# if newCfg and nearCfg are connected and newCfg is free.
        *           -# Add newCfg to tree
        *           -# Add new edge nearCfg->newCfg to tree.
        *       -# end if
        *   -# end loop
        *
        *@param K This variable determines how many times will be tried to
        *         generate new nodes.
        *@param deltaT This variable determines how far a new node will be from
        *       its parent in the tree.
        *@param U A "direction" set.
        *
        *@note assumes HOLONOMIC robot.
        *@note add'l parameters are used in calls to various algobase 
        *       functions.
        */
      static void RRT(Roadmap* rm, int K, double deltaT, vector<Cfg>& U,
                                    CollisionDetection*,LocalPlanners*,
                                    DistanceMetric *, CNInfo&, LPInfo);

  //}@

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Static: Connect Connected Components Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Connect Connected Components Methods*/
  //@{

      /**Connect two small connected components.
        *Algorithm:
        *   -# for every Cfg, cfg1, in cc1
        *       -# for every Cfg. cgf2, in cc2
        *           -# if cfg1 and cfg2 could be connected.
        *           -# then add this edge to roadmap, _rm.
        *              and return.
        *           -# else if info.addPartialEdge
        *               -# then add failed path to roadmap
        *                  (cfg1->failed_point)
        *           -# end if
        *       -# end for
        *   -# end for
        *
        *@param _cc1id a node id which defines the first connected component.
        *@param _cc2id a node id which defines the second connected component.
        */
      static void 
      ConnectSmallCCs
      (Roadmap* _rm,CollisionDetection *cd, LocalPlanners* lp,DistanceMetric * dm,CN& _cn, 
       CNInfo& info, vector<Cfg> & cc1vec, vector<Cfg> & cc2vec);

      /**Connect two big connected components.
        *Algorithm:
        *   -# find k pairs of closest Cfgs from cc1 to cc2.
        *   -# for every pair, p, in k pairs.
        *       -# if p.first and p.second could be connected.
        *          (p.first is from cc1 and p.second is from cc2)
        *       -# then add this edge to roadmap, _rm.
        *          and return.
        *       -# else if info.addPartialEdge
        *           -# then add failed path to roadmap
        *              (cfg1->failed_point)
        *       -# end if
        *   -# end for
        *
        *In step 1, FindKClosestPairs is used to find first k
        *closest pairs from the first connected component to 
        *the second connected component.
        *
        *@param _cc1id a node id which defines the first connected component.
        *@param _cc2id a node id which defines the second connected component.
        *@param _cn _cn.GetKPairs define k above.
        */
      static void 
      ConnectBigCCs
      (Roadmap* _rm,CollisionDetection *cd, LocalPlanners* lp,DistanceMetric * dm,CN& _cn, 
       CNInfo& info, vector<Cfg> & cc1vec, vector<Cfg> & cc2vec);

  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Static: Find Closest Pairs
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Find Closest Pairs.*/
  //@{

      /**Find k pairs of closest Cfgs from a given Cfg to 
        *all Cfgs in a given vector.
        *
        *@param vec1 A list of Cfgs.
        *@param k Find k closest Cfgs in vec1 for a given Cfg.
        *
        *@return a list of pairs, <cfg1, n1>, <cfg1, n2>, ... , <cfg1, nk>
        *Here n1,..,nk are k Cfgs in vec1 and are k closest neighbors of cfg1.
        */
      static vector< CfgPairType >
      FindKClosestPairs
      (Environment *_env,DistanceMetric * dm, CNInfo& info,Cfg& cfg1,vector<Cfg>& vec1, int k);

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
      static vector< CfgPairType >
      FindKClosestPairs
      (Environment *_env, DistanceMetric * dm, CNInfo& info,vector<Cfg>& vec1, int k);

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
      static vector< pair<VID, VID> > 
      FindKClosestPairs
      (Roadmap *rm, DistanceMetric * dm, CNInfo& info, vector<Cfg>& vec1, int k);

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
	  static vector< pair<VID, VID> > 
	  FindKClosestPairs
	  (Roadmap *rm,DistanceMetric * dm, CNInfo& info,vector<Cfg>& vec1,vector<Cfg>& vec2, int k);

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
        *possilbe paths.
        */
      static vector< CfgPairType > 
      FindKClosestPairs
      (Environment *_env,DistanceMetric * dm,CNInfo& info,vector<Cfg>& vec1 ,vector<Cfg>& vec2, int k);

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
        *@return A (k-elemet) list of Cfg_VE_Type.
        *@see VE_DIST_Compare for sorting.
        */
      static vector< Cfg_VE_Type >
      FindKClosestPairs
      (Roadmap*, DistanceMetric *, CNInfo&, Cfg& cfg, vector<Cfg>& verts,
       vector< pair<pair<VID,VID>,WEIGHT> >& edges, int k, bool midpoint);

  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Static:: Helper methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Helper Methods for Node Connection*/
  //@{

      /**Get an initialized LPInfo.
        *This method creates a new LPInfo and init its value
        *accroding to given parameters.
        *
        *Initialization Rules are :
        *   -# LPInfo::positionRes = #connectionPosRes
        *   -# LPInfo::orientationRes = #connectionOriRes
        *   -# LPInfo::checkCollision = true
        *   -# LPInfo::savePath = false
        *   -# LPInfo::cdsetid = info.cdsetid
        *   -# LPInfo::dmsetid = info.dmsetid
        */
      static LPInfo Initialize_LPinfo(Roadmap * _rm,CNInfo& info);

      /**Classify Cfgs by from which obstacles these Cfg are generated.
        *Vertices are stored as generated but they may have been originally
        *generated with respect to some obstacle. This proceedure
        *will return a vector where every element is a vector of cfg's
        *corresponding to a unique (& valid) id value.
        *
        *@return Return a 2D "array", each row contains Cfgs generated 
        *"from" same obstacle.
        */
      static vec_vec_CfgType Get_Cfgs_By_Obst(Roadmap * _rm);

      /**Compare two distances in DIST_TYPE instances.
        *return (_cc1.second < _cc2.second)
        */
      static bool DIST_Compare    (const DIST_TYPE &_cc1, const DIST_TYPE &_cc2);

      /**Compare two distances in VE_DIST_TYPE instances.
        *return (_cc1.second < _cc2.second)
        */
      static bool VE_DIST_Compare (const VE_DIST_TYPE &_cc1, const VE_DIST_TYPE &_cc2);

      /**Compare InfoCfg::obst between two given Cfgs.
        *@return (_cc1.info.obst < _cc2.info.obst )
        *@note this is used to "sort" Cfg's by obst generation number
        *@see Get_Cfgs_By_Obst
        */
      static bool info_Compare    (const Cfg &_cc1, const Cfg &_cc2);

  //@}


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  static double connectionPosRes, ///< Position resolution for node connection
                connectionOriRes; ///< Orientation resolution for node connection

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:
};

#endif
