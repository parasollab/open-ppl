// $Id:

/**@file Haptic.h
   This class reads in a 'haptic' path as input and calls the
   appropriate pushing method located in Push.cpp. This class
   is based on the HapticInput class.
   @date 09/29/98
   @author O.B. Bayazit
*/

#ifndef Haptic_h
#define Haptic_h

#define ADD_FREE 1
#define USE_AS_SEED 2
#define USE_SURFACE 4
#define CLOSESTWORKSPACEPOINTS 8
#define USE_SURFACE2 16

#include "Roadmap.h"
#include "Push.h"

class Haptic {

public:

  //===================================================================
  // Constructors and Destructor
  //===================================================================
   Haptic();
   ~Haptic();

  //===================================================================
  // methods
  //===================================================================
   void init(Roadmap *rm, char * tmp[5], CollisionDetection *_cd, LocalPlanners *_lp,
    DistanceMetric *_dm, GNInfo gnInfo, CNInfo cnInfo);

   /**
   This method adds the free nodes from the Phantom input to the roadmap.
   Should not be used on existing nodes/paths in the roadmap.
   */
   void AddFreeNodes(vector <Cfg>cfgs,bool checkConnection);

   /**
   This method takes surface nodes (typically generated by Phantom)
   and uses as seed nodes for obprm.
   Should eventually be moved to GenerateMapNodes.cpp
   */
   void AddUsingSeed(vector <Cfg> seeds,int nodesPerSeed);

protected:
private:
   Roadmap *rdmp;
   Environment *env;
   CollisionDetection *cd;
   LocalPlanners *lp;
   DistanceMetric *dm;
   GNInfo generationInfo;
   CNInfo connectionInfo;

};
#endif
