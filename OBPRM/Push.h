/**@file Push.h
   General Description
      This class contains pushing methods based on the 
      HapticInput class written by O.B. Bayazit and Guang Song.
      This class takes a bad path and attempts to push it to the 
      free space.
   @date 09/29/98
   @author O.B. Bayazit (HRoadmap class)
*/

#ifndef Push_h
#define Push_h

#include "Roadmap.h"

class Push {
    
public:

  //===================================================================
  // Constructors and Destructor
  //===================================================================
   Push(Roadmap *rm, CollisionDetection *_cd, 
	LocalPlanners *_lp, DistanceMetric *_dm, GNInfo gnInfo, CNInfo cnInfo);
   ~Push();

  //===================================================================
  // methods
  //===================================================================
   bool CheckConnection(Cfg c1,Cfg c2);

   // written by Burchan (method 1 & 3)
   void SimplePush(vector <Cfg> seeds,int numIntermediate);
   //was AddUsingSurface();

   void WorkspaceAssistedPush(vector <Cfg> seeds,int totalNodes);
   //was AddUsingClosestWorkspacePoints();

   // by Guang (method 2)
   void ShortestPush(vector <Cfg> seeds);
   //was AddUsingSurface2();

   vector<Cfg> GenerateClosestOutsideNode(bool&, Vector3D&, double&, Cfg inter, double incrCoord);

   //written by Sujay and Shawna
   vector <Cfg> GenerateIntermediateCfgs(Cfg cfg_start, Cfg cfg_end, double stepSize);
   vector <Cfg> findCollidedCfgs(vector<Cfg> cfgs);
   bool isPathGood(vector <Cfg> cfgs);   

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

