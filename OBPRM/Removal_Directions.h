///////////////////////////////////////////////////////////////////////////////
//  Removal_Directions.h
// 
//  Class containing heuristics for removal directions for the parts
//  of an assembly
//
//  10/26/00  Sujay
//
///////////////////////////////////////////////////////////////////////////////
#ifndef Removal_Directions_h
#define Removal_Directions_h

#include <iostream.h>
#include <string.h>
#include <stdlib.h>

#include "CollisionDetection.h"
#include "ConnectMapNodes.h"
#include "OBPRM.h"
#include "Roadmap.h"

#define REMOVAL_DISTANCE       10
#define REMOVAL_INCREMENT      0.2
#define MINIMUM_CLEARANCE      1
#define CLEARANCE              5


class Removal_Directions {
public:

  //===================================================================
  //  Constructors and Destructor
  //===================================================================

  Removal_Directions() {};

  ~Removal_Directions();

  //===================================================================
  //  Other Methods
  //===================================================================

  // Main method that returns cfgs that are closer to disassembly
  static vector<Cfg> expand(Cfg * _cfg, Roadmap *rmap, CollisionDetection cd,  	  
                     ConnectMapNodes cn, int HEURISTIC);


// methods called internally within the class

private:

  // Method returning cfgs that have been moved along normals to part faces
  static vector<Cfg> Normals(Cfg * _cfg, Roadmap * _rm, CollisionDetection cd, ConnectMapNodes cn);

  // Method to check is a body is beyond a particular distance from all other bodies
  static bool IsBeyond(Environment* env, int currentbody, double distance);

}; 
#endif
