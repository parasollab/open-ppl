// $Id$
/////////////////////////////////////////////////////////////////////
//
//   Query.h
//
//   General Description
//      This is the main OBPRM class which contains data and methods
//      to manipulate the environment with specified moving bodies
//      (ie, robot(s)) and the corresponding roadmap.
//
//      This file contains the prototype declarations for the class. 
//      Definitions are in the file "Roadmap.c".
//
//  Created
//      08/18/98  Nancy Amato
//  Last Modified By:
//      11/23/98  Lucia K. Dale
/////////////////////////////////////////////////////////////////////

#ifndef Query_h
#define Query_h

#include "QueryCmds.h"     
#include "Roadmap.h"     
#include "util.h"     

#include "util.h"

class Query {
public:
  //===================================================================
  //  Constructors and Destructor
  //===================================================================
   Query();

   // *preferred*, fills input & inits 
   Query(Input *, QueryCmds*, 
	CollisionDetection*, DistanceMetric*, LocalPlanners*,ConnectMapNodes*);  

   ~Query();

  //===================================================================
  // Other Methods
  //===================================================================
      // default initializations
   void initDefaultSetIDs(ConnectMapNodes    *);
 
      // actually do the query
   bool PerformQuery(CollisionDetection*, ConnectMapNodes*, LocalPlanners*, DistanceMetric*);
   bool PerformQuery(Cfg _start, Cfg _goal, CollisionDetection*,
        ConnectMapNodes*, LocalPlanners*, DistanceMetric*, SID _lpsid, vector<Cfg>* _path);
   bool CanConnectToCC(Cfg, CollisionDetection*, ConnectMapNodes*,
        LocalPlanners*,DistanceMetric*, vector<Cfg>, SID, VID*, LPInfo*);
   bool GetPathSegment(Cfg, Cfg, CollisionDetection*, LocalPlanners*, 
        DistanceMetric*, WEIGHT, LPInfo*);

      // Display, Input, Output
   void ReadQuery(const char* _filename);
   void WritePath();
   void WritePath(char* _filename);

  //===============================================================
  //  Data
  //===============================================================

   Roadmap rdmp;

   vector<Cfg> query;                  // start,[intermediate nodes],goal 
   vector<Cfg> path;                   // output paths

   char *outputPathFile;

   SID lpsetid;
   SID cdsetid;
   SID dmsetid;
};

#endif
