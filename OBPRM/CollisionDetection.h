// $Id$
/////////////////////////////////////////////////////////////////////
//
//  CollisionDetection.h
//
//  General Description
//
//  Created
//      8/11/98  Daniel Vallejo 
/////////////////////////////////////////////////////////////////////

#ifndef CollisionDetection_h
#define CollisionDetection_h

#include "OBPRM.h"
#include "Vectors.h"
#include "Sets.h"
#include "Transformation.h"
#include "Input.h"

#include <stdlib.h>
#include <stdio.h>

#include <iostream.h>
#include <fstream.h>
#include <strstream.h>
#include <iomanip.h>

#include <vector.h>

//CD libraries
#ifdef USE_CSTK
#include <cstkSmallAPI.h>
#include <cstk_global.h>
#endif
#ifdef USE_VCLIP
#include <vclip.h>
#endif
#ifdef USE_RAPID
#include <RAPID.H>
#endif

const double MaxDist =  1e10;

class Environment;
class GenerateMapNodes;
class ConnectMapNodes;

class Roadmap;
class MultiBody;

class CD;
class CDSets;
class CollisionDetection;

//---------------------------------------------------------------
// Algo base information data structures
//
// This was made into a class so I knew everything was
// initialized properly. I got tired of trying to track
// down where all the CDInfo variables were created - BD July 2000
//---------------------------------------------------------------
class CDInfo {

public:
   CDInfo();
   ~CDInfo();
   void ResetVars();

    int    colliding_obst_index;

    bool     ret_all_info;
    int      nearest_obst_index;
    double   min_dist;
    Vector3D robot_point;  // robot_point and object_point
    Vector3D object_point; // should by closest points in world coords

private:

};

// pointer to cd function
typedef bool (*CDF) (MultiBody*,MultiBody*,CD&,CDInfo&);     

const int Out = 0;	// Type Out: no collision sure; collision unsure.
const int In = 1;	// Type In: no collision unsure; collision sure.
const int Exact = 2;	// Type Exact: no collision sure; collision sure.


/////////////////////////////////////////////////////////////////////
//  class CD
//
//  General Description
//
/////////////////////////////////////////////////////////////////////
class CD {
  friend class CDSets;
public:
  //===================================================================
  // Constructors and Destructor
  //===================================================================
  CD();
  ~CD();

  //===================================================================
  // Operators
  //===================================================================
  CD&  operator=(const CD & _cd);
  bool operator==(const CD & _cd) const;

  //===================================================================
  // Other Methods
  //===================================================================
  char* GetName() const;
  CDF   GetCollisionDetection();
  int 	GetType() const; 

protected:

  //===================================================================
  // Data
  //===================================================================
  char  name[80];
  CDF   collision_detection;          // ptr to collision detection code
  EID	cdid;
  int 	type; 

private:
};

ostream& operator<< (ostream& _os, const CD& cd);


/////////////////////////////////////////////////////////////////////
//  class CDSets
//
//  General Description
//
/////////////////////////////////////////////////////////////////////
class CDSets : public BasicSets<CD> {
public:
  //===================================================================
  //  Constructors and Destructor
  //===================================================================
    CDSets();
    ~CDSets();

  //===================================================================
  //  Other Methods
  //===================================================================

   // Adding LPs, Making & Modifying LP sets

   int AddCD(const char* _cdinfo);     // add cd(s) to universe
   int AddCDToSet(const SID _sid, const EID _cdid);
   int DeleteCDFromSet(const SID _sid, const EID _cdid);

   SID MakeCDSet(const char* cdlist);  // make an ordered set of cds,
   SID MakeCDSet(istream& _myistream); //  - add cd to universe if not there
   SID MakeCDSet(const EID _eid);
   SID MakeCDSet(const vector<EID> _eidvector);

   int DeleteCDSet(const SID _sid);

   // Getting Data & Statistics

   CD GetCD(const EID _cdid) const;
   vector<CD> GetCDs() const;
   vector<CD> GetCDSet(const SID _sid) const;
   vector<pair<SID,vector<CD> > > GetCDSets() const;



   // Display, Input, Output

   void DisplayCDs() const;
   void DisplayCD(const EID _cdid) const;
   void DisplayCDSets() const;
   void DisplayCDSet(const SID _sid) const;

   void WriteCDs(const char* _fname) const;
   void WriteCDs(ostream& _myostream) const;
   void ReadCDs(const char* _fname);
   void ReadCDs(istream& _myistream);


  //===================================================================
  //  Data
  //===================================================================
protected:
private:
};

/////////////////////////////////////////////////////////////////////
//  class CollisionDetection
//
//  General Description
//
/////////////////////////////////////////////////////////////////////
class CollisionDetection {
public:
  //===================================================================
  // Constructors and Destructor
  //===================================================================
  CollisionDetection();
  ~CollisionDetection();

  //===================================================================
  // Other Methods
  //===================================================================

  void DefaultInit();
  void UserInit(Input * input,  GenerateMapNodes*, ConnectMapNodes*);

#ifdef USE_CSTK
  double cstkDistance(MultiBody* robot, MultiBody* obstacle);
#endif
  double Clearance(Environment * env);

  // Drivers
  bool IsInCollision
	(Environment* env, SID _cdsetid, CDInfo& _cdInfo, MultiBody* lineRobot = NULL);
  bool IsInCollision
	(Environment* env, SID _cdsetid, CDInfo& _cdInfo, MultiBody* rob, MultiBody* obstacle);
  bool IsInCollision
	(Environment* env, SID _cdsetid, CDInfo& _cdInfo, int robot, int obstacle);

  // Individual Static Methods
  static bool IsInCollision_boundingSpheres
	(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);
  static bool IsInCollision_insideSpheres
	(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);
  static bool IsInCollision_naive
	(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);
  static bool IsInCollision_quinlan
	(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);
#ifdef USE_CSTK
  static bool IsInCollision_cstk
	(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);
#endif

#ifdef USE_VCLIP
  static bool IsInCollision_vclip
	(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);
  static VclipPose GetVclipPose(const Transformation&, const Transformation&);
  static bool IsInColl_AllInfo_vclip
  (MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);
#endif

#ifdef USE_RAPID
  static bool IsInCollision_RAPID
	(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);
#endif

  // for cstk
  static void SetLineTransformation(const Transformation&, double linTrans[12]); 

  //===================================================================
  // Data
  //===================================================================
  CDSets collisionCheckers;

public:
  CDInfo cdInfo;
protected:
private:
};

#endif
