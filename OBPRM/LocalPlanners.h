// $Id$
/////////////////////////////////////////////////////////////////////
//
//  LocalPlanners.h
//
//  General Description
//      This set of classes supports a "Local Planning Algobase".
//
//      The classes in the set are:
//        o LP            -- info related to an individual local planner 
//        o LPSets        -- contains 'master' list of all local planners
//                           and maintains sets of local planners
//                           (derived from BasicSets<LP>)
//        o LocalPlanners -- has LPSets as data element, and its methods
//                           include the actual local planning algorithms.
//                           A 'wrapper' method cycles thru sets of lps.
//
//       Each LP element is given a unique id, which is used to compose
//       labels encoding which local planners succeeded in connection.A
//       (e.g., for storing as edges in a roadmap).
//
//       NOTE: Assumes that the configuration type is known (e.g, typedef'd)
//             as "Cfg" -- this must be provided.
//
//  Created
//      8/7/98  Nancy Amato
//  Last Modified By:
//      xx/xx/xx  <Name>
/////////////////////////////////////////////////////////////////////

#ifndef LocalPlanners_h
#define LocalPlanners_h

#include "OBPRM.h"
#include "Sets.h"

#include "util.h"
#include "CollisionDetection.h"

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
//              SET DEFN's in "Roadmap.c"
//---------------------------------------------------------------
enum lp_predefined {    //---------------
        SL,             // straightline
        R5,             // rotate at s=0.5
        SL_R5,          // SL and R5
        AD69,           // a* distance w/ 6 tries and 9 neighbors
        SL_R5_AD69,     // SL & R5 & AD69
        LP_USER1};      // first user defined lp set, if any

//---------------------------------------------------------------
// Algo base information data structures
//---------------------------------------------------------------                        
struct CNInfo;
class Roadmap;
class DistanceMetric;

class LPInfo {
public:
    LPInfo() {}
    LPInfo(Roadmap *rm, const CNInfo& cnInfo);

    bool checkCollision;
    bool savePath;
    bool saveFailedPath;
    pair<WEIGHT,WEIGHT> edge;
    double positionRes;
    double orientationRes;
    int cd_cntr;
    SID cdsetid;
    SID dmsetid;
    vector<Cfg> path;
    pair<Cfg,Cfg> savedEdge;
    int nTicks;
    
};
    
class LP;
class LPSets;
class LocalPlanners;

class Roadmap;

typedef bool (*LPF) (Environment *env,CollisionDetection *,DistanceMetric*,
				Cfg&,Cfg&,LP&,LPInfo*); 	// pointer to lp function
                                         		// *NOTE* need to update
                                         		//  when params known


/////////////////////////////////////////////////////////////////////
//  class LP
//
//  General Description
//     This class contains information relevant to a particular 
//     local planner (e.g., name, ptr to function, etc).
//     
//
/////////////////////////////////////////////////////////////////////
class LP {
  friend class LPSets;
public:

  //===================================================================
  // Constructors and Destructor
  //===================================================================
  LP();
  ~LP();

  //===================================================================
  // Operators
  //===================================================================
  bool operator==(const LP & _lp) const;

  //===================================================================
  // Other Methods
  //===================================================================
  char*  GetName() const;
  LPF    GetPlanner();
  double GetS() const;
  int    GetTries() const;
  int    GetNeighbors() const;
  EID    GetID() const;
  int    GetFEdgeMask() const;
  int    GetBEdgeMask() const;

protected:
  //===================================================================
  // Data
  //===================================================================
  char   name[80];
  LPF    planner;          // ptr to local planner code
  double sValue;
  int    tries;
  int    neighbors;
  EID    lpid;
  int    forwardEdge;
  int    backEdge;
private:
};

ostream& operator<< (ostream& _os, const LP& lp); 



/////////////////////////////////////////////////////////////////////
//  class LPSets
//
//  General Description
//     This class is derived from BasicSets<LP>.
//
//     This class manages/contains:
//        o a 'master' list of all local planners, and
//        o sets of local planners
//
//     Each local planner and set is given a unique id. The lp ids (EIDs)
//     are used to compose labels encoding local planner sets (used, e.g.,
//     to record which local planners succeed when connecting a pair of
//     configurations).
//
/////////////////////////////////////////////////////////////////////
class LPSets : public BasicSets<LP> {
public:
  //===================================================================
  //  Constructors and Destructor
  //===================================================================
    LPSets();
    ~LPSets();

  //===================================================================
  //  Other Methods
  //===================================================================

        // Adding LPs, Making & Modifying LP sets
   int AddLP(const char* _lpinfo);     // add lp(s) to universe
   int AddLPToSet(const SID _sid, const EID _lpid);
   int DeleteLPFromSet(const SID _sid, const EID _lpid);

   SID MakeLPSet(const char* lplist);  // make an ordered set of lps,
   SID MakeLPSet(istream& _myistream); //  - add lp to universe if not there
   SID MakeLPSet(const EID _eid);
   SID MakeLPSet(const vector<EID> _eidvector);

   int DeleteLPSet(const SID _sid);

        // Getting Data & Statistics
   LP GetLP(const EID _lpid) const;
   vector<LP> GetLPs() const;
   vector<LP> GetLPSet(const SID _sid) const;
   //vector<pair<SID,vector<pair<EID,LP> > > > GetLPSets() const;
   vector<pair<SID,vector<LP> > > GetLPSets() const;

        // Display, Input, Output
   void DisplayLPs() const;
   void DisplayLP(const EID _lpid) const;
   void DisplayLPSets() const;
   void DisplayLPSet(const SID _sid) const;

   void WriteLPs(const char* _fname) const;
   void WriteLPs(ostream& _myostream) const;
   void ReadLPs(const char* _fname);
   void ReadLPs(istream& _myistream);

  //===================================================================
  //  Data
  //===================================================================
protected:
private:
};

/////////////////////////////////////////////////////////////////////
//  class LocalPlanners
//
//  General Description
//     This is the main local planner class. It has an LPSets object
//     as a data element (list of lps and lp sets), and its methods
//     include the actual local planning algorithms:
//        o straightline
//        o rotate_at_s, 0 <= s <= 1
//        o a_star_distance and a_star_clearance, 
//            1 <= tries <= 20
//            neighbors = 3, 9, or 15
//
//     'Wrapper' methods cycle thru sets of lps, stopping upon 1st
//     success or trying all.
//
/////////////////////////////////////////////////////////////////////
class LocalPlanners {
   friend SID MakeLPSet(istream&); 
public:
  //===================================================================
  // Constructors and Destructor
  //===================================================================
  LocalPlanners();
  ~LocalPlanners();

  //===================================================================
  // Other Methods
  //===================================================================
  void DefaultInit();
  void UserInit(Input *input,  ConnectMapNodes*);
  bool IsConnected(Environment *env,CollisionDetection *,DistanceMetric *,
				Cfg _c1, Cfg _c2, SID _lpsetid, LPInfo *info);
  bool IsConnected(Roadmap * rm,CollisionDetection *,DistanceMetric *,
                                Cfg _c1, Cfg _c2, SID _lpsetid, LPInfo *info);
  bool IsConnectedFindAll(Environment *env,CollisionDetection *,DistanceMetric *,
				Cfg _c1, Cfg _c2, SID _lpsetid, LPInfo *info);

  static bool IsConnected_straightline_simple(Environment *env,CollisionDetection *,DistanceMetric *,
				Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);
  static bool IsConnected_SLclearance(Environment *env,CollisionDetection *,DistanceMetric *,
                                Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);
  static bool lineSegmentInCollision(Environment *env,CollisionDetection *,DistanceMetric *,
                                Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);
  static bool IsConnected_straightline(Environment *env,CollisionDetection *,DistanceMetric *,
				Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);
  static bool IsConnected_rotate_at_s(Environment *env,CollisionDetection *,DistanceMetric *,
				Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);
  static bool IsConnected_astar(Environment *env,CollisionDetection *,DistanceMetric *,
				Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);

  bool UsesPlannerOtherThan(char plannerName[], SID lpsetid=0);

  //===================================================================
  // Data
  //===================================================================
  LPSets planners;
  static cd_predefined cdtype; // used for building line segment.
  static int lineSegmentLength;
  static bool usingClearance;
protected:
private:
};


#endif
