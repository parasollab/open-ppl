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
//  Last Modified By:
//      xx/xx/xx  <Name>
/////////////////////////////////////////////////////////////////////

#ifndef Roadmap_h
#define Roadmap_h

#include "Input.h"              // init info (command line, env initfile)
#include "OBPRM.h"              // Cfg type defined here
#include "AttEnvironment.h"     // Environment classes
#include "RoadmapGraph.h"     	// graph class

#include "LocalPlanners.h"      // Local Planner       Algobase
#include "GenerateMapNodes.h" 	// Map Node Generators Algobase
#include "ConnectMapNodes.h" 	// Map Node Connectors Algobase
#include "DistanceMetrics.h"    // Distance Metrics    Algobase
#include "CollisionDetection.h" // Collision Detection Algobase


//typedef int WEIGHT;             // weights for roadmap graph (encode lps)

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
	   AttEnvironment* env = NULL);

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
		    AttEnvironment* env = NULL); 

        //------------------------------------------------------------
        // Read roadmap's environment from a file into environment field
        //------------------------------------------------------------
   void InitEnvironment(Input*);

   void InitEnvironment(AttEnvironment*);

  //=======================================================================
  // Getting Statistics
  //=======================================================================
        //------------------------------------------------------------
        // Get a pointer to roadmap's environment 
        //------------------------------------------------------------
    AttEnvironment * GetAttEnvironment();

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
    void ReadExtraCfgInfo(istream& _myistream);



        //------------------------------------------------------------
        // Write roadmap's environment to an output stream 
        //------------------------------------------------------------
    void WriteEnvironment(ostream& _ostream);

  //=======================================================================
  // Data
  //=======================================================================
public:

   AttEnvironment * environment;

   RoadmapGraph<Cfg,WEIGHT> roadmap;   // Roadmap built for environment

protected:
private:
};

#endif
