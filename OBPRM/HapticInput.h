/////////////////////////////////////////////////////////////////////
//
//   HapticInput.h
//
//   General Description
//	Based on HRoadmap class written by O.B. Bayazit, this class
//	read in a 'haptic' path as input and process it and return 
// 	free nodes in the roadmap.
//  Created
//      09/29/98  O.B. Bayazit (HRoadmap class)
//      07/23/99  Guang Song (add some methods, rename it HapticInput)
//  Last Modified By:
/////////////////////////////////////////////////////////////////////
#ifndef HapticInput_h
#define HapticInput_h

#define ADD_FREE 1
#define USE_AS_SEED 2
#define USE_SURFACE 4
#define CLOSESTWORKSPACEPOINTS 8
#define USE_SURFACE2 16

#include "Roadmap.h"

class HapticInput {
    
public:

  //===================================================================
  // Constructors and Destructor
  //===================================================================
   HapticInput();
   ~HapticInput();

  //===================================================================
  // methods
  //===================================================================
   void init(Roadmap *rm, char * tmp[5], CollisionDetection *_cd, LocalPlanners *_lp,
	DistanceMetric *_dm, GNInfo gnInfo, CNInfo cnInfo);
   void AddHapticPath(vector <Cfg> cfgs,int mask, int nodesPerSeed,
    			int intermediate, int closest,
			bool checkConnection);
   bool CheckConnection(Cfg c1,Cfg c2);

   // written by Burchan (method 1 & 3)
   void AddFreeNodes(vector <Cfg>cfgs,bool checkConnection);
   void AddUsingSeed(vector <Cfg> seeds,int nodesPerSeed);
   void AddUsingSurface(vector <Cfg> seeds,int numIntermediate);
   void AddUsingClosestWorkspacePoints(vector <Cfg> seeds,int totalNodes);

   // by Guang (method 2)
   void AddUsingSurface2(vector <Cfg> seeds);
   vector<Cfg> GenerateClosestOutsideNode(bool&, Vector3D&, double&, Cfg inter, double incrCoord);

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


   //----------------------------------------------------------------------------
   // these three methods are from GenerateMapNodes class, so go back there later.
   Cfg GenerateSurfaceCfg(Environment *, CollisionDetection *cd, DistanceMetric *, 
                          Cfg , Cfg ,  GNInfo& );
   Cfg GenerateOutsideCfg(Environment *, CollisionDetection *cd, Cfg , Cfg , GNInfo &);
   vector <Cfg> GenerateOBPRMNodes(Environment *env, CollisionDetection *cd, 
				   DistanceMetric *, vector <Cfg> ,int , GNInfo &);

