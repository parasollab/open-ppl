// $Id$
/////////////////////////////////////////////////////////////////////
//
//  CollisionDetection.h
//
//  General Description
//
//  Created
//      8/11/98  Daniel Vallejo 
//  Last Modified By:
//      1/13/98 Guang Song, cfg is no longer an argument for many methods here.
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

typedef bool (*CDF) (MultiBody*,MultiBody*,CD&);     // pointer to cd function
                                    // *NOTE* need to update
                                    //  when params known

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

  bool IsInCollision(Environment * env, SID _cdsetid, MultiBody * lineRobot = NULL);
  bool IsInCollision(Environment * env, MultiBody * rob, MultiBody * obstacle, SID _cdsetid);
  bool IsInCollision(Environment * env, int robot, int obstacle, SID _cdsetid);

  static bool IsInCollision_boundingSpheres(MultiBody* robot, MultiBody* obstacle,  CD& _cd);
  static bool IsInCollision_insideSpheres(MultiBody* robot, MultiBody* obstacle,  CD& _cd);
  static bool IsInCollision_naive(MultiBody* robot, MultiBody* obstacle, CD& _cd);
  static bool IsInCollision_quinlan(MultiBody* robot, MultiBody* obstacle,  CD& _cd);
#ifdef USE_CSTK
  static bool IsInCollision_cstk(MultiBody* robot, MultiBody* obstacle,  CD& _cd);
#endif
#ifdef USE_VCLIP
  static bool IsInCollision_vclip(
	MultiBody* robot, MultiBody* obstacle,  CD& _cd);
  static VclipPose GetVclipPose(const Transformation&, const Transformation&);
#endif
#ifdef USE_RAPID
  static bool IsInCollision_RAPID(MultiBody* robot, MultiBody* obstacle,  CD& _cd);
#endif

  static void SetLineTransformation(const Transformation&, double linTrans[12]); // for cstk

  //===================================================================
  // Data
  //===================================================================
  CDSets collisionCheckers;

protected:
private:
};

#endif

