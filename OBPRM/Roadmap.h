// $Id$
/////////////////////////////////////////////////////////////////////
//
//   Roadmap.h
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
//      07/16/98  Lucia K. Dale
/////////////////////////////////////////////////////////////////////

#ifndef Roadmap_h
#define Roadmap_h

#include "Input.h"              // init info (command line, env initfile)
#include "OBPRM.h"              // Cfg type defined here
#include "Environment.h"     // Environment classes
#include "RoadmapGraph.h"     	// graph class

#include "LocalPlanners.h"      // Local Planner       Algobase
#include "GenerateMapNodes.h" 	// Map Node Generators Algobase
#include "ConnectMapNodes.h" 	// Map Node Connectors Algobase
#include "DistanceMetrics.h"    // Distance Metrics    Algobase
#include "CollisionDetection.h" // Collision Detection Algobase

// Format version for roadmap (*.map) files
//      The number breaks down as MonthDayYear (mmddyy).
//      For future versions, name them according to
//      YearMonthDay(yyyymmdd) so numerical
//      comparisons can be more easily made.
// Warning: Be consistent.  It should be YYYYMMDD
//      Inconsistent conversions can be misleading.
//      For example, comparing 200083  to 20000604.
#define RDMPVER_LEGACY                      0
#define RDMPVER_LEGACY_CFG_FIELDS           0            // none
#define RDMPVER_LEGACY_EDGEWT_FIELDS        1            // lp

#define RDMPVER_62000                      62000
#define RDMPVER_62000_CFG_FIELDS           2            // obst, tag
#define RDMPVER_62000_EDGEWT_FIELDS        2            // lp, ticks (or weight for DBL)

#define RDMPVER_061100                      61100
#define RDMPVER_061100_CFG_FIELDS           2            // obst, tag
#define RDMPVER_061100_EDGEWT_FIELDS        2            // lp, ticks (or weight for DBL)

#define RDMPVER_061300                      61300
#define RDMPVER_061300_CFG_FIELDS           3            // obst, tag, clearance
#define RDMPVER_061300_EDGEWT_FIELDS        2            // lp, ticks (or weight for DBL)

#define RDMPVER_CURRENT                     61300 
#define RDMPVER_CURRENT_STR                "061300" 
#define RDMPVER_CURRENT_CFG_FIELDS          3            // obst, tag, clearance
#define RDMPVER_CURRENT_EDGEWT_FIELDS       2            // lp, ticks (or weight for DBL)

class Roadmap {
public:
  //===================================================================
  //  Constructors and Destructor
  //===================================================================
        //------------------------------------------------------------
        // Init Roadmap.
        //------------------------------------------------------------

   Roadmap();


   // *preferred*, fills input & inits 
   Roadmap(Input*, CollisionDetection*, 
	   DistanceMetric*, LocalPlanners*, 
	   Environment* env = NULL);

        //------------------------------------------------------------
        // Delete all the elements of RoadMap. 
        //------------------------------------------------------------
   ~Roadmap();

  //=======================================================================
  // Initialialization
  //=======================================================================
        //------------------------------------------------------------
        // Initialize Roadmap (parse command line & read in data)
        //------------------------------------------------------------
   void InitRoadmap(Input*, CollisionDetection*,
		    DistanceMetric*,LocalPlanners*, char *ExistingMap=NULL, 
		    Environment* env = NULL); 

        //------------------------------------------------------------
        // Read roadmap's environment from a file into environment field
        //------------------------------------------------------------
   void InitEnvironment(Input*);

   void InitEnvironment(Environment*);

  //=======================================================================
  // Getting Statistics
  //=======================================================================
        //------------------------------------------------------------
        // Get a pointer to roadmap's environment 
        //------------------------------------------------------------
    Environment * GetEnvironment();

  //=======================================================================
  // Display, Input, Output
  //=======================================================================
        //------------------------------------------------------------
        // Read/Write roadmap from/to a file
        //------------------------------------------------------------
    void ReadRoadmap(Input*, CollisionDetection *cd,
                             DistanceMetric *dm,
                             LocalPlanners *lp,
                             const char* _filename);
    void WriteRoadmap(Input*, CollisionDetection *cd,
                             DistanceMetric *dm,
                             LocalPlanners *lp,
                             const char* _filename = 0);
    void ReadRoadmapGRAPHONLY(const char* _filename);

    // utilities to automatically convert to current roadmap version
    bool CheckVersion(const char* _fname);
    bool ConvertToCurrentVersion(const char* _fname, int thisVersion);
    void ConvertGraph(istream&  myifstream, ostream& myofstream,
		      int presentCfgFields, int presentEdgeWtFields);
    void SaveCurrentVersion(const char* _fname, int thisVersion, char* infile, char* outfile);


        //------------------------------------------------------------
        // Write roadmap's environment to an output stream 
        //------------------------------------------------------------
    void WriteEnvironment(ostream& _ostream);

  //=======================================================================
  // Data
  //=======================================================================
public:

   Environment * environment;

   RoadmapGraph<Cfg,WEIGHT> roadmap;   // Roadmap built for environment

   int RoadmapVersionNumber;

protected:
private:
};

#endif
