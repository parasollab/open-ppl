// $Id$
/////////////////////////////////////////////////////////////////////
//
//  Cfg_fixed_tree.h
//
//  General Description
//      A derived template class from CfgManager. It provides some 
//      specific implementation directly related to fixed-base 
//	tree structure robots.
//      
//
//  Created
//      10/11/99        Guang Song
//
//  Last Modified By:
//
/////////////////////////////////////////////////////////////////////

#ifndef Cfg_fixed_tree_h
#define Cfg_fixed_tree_h

#include "CfgManager.h"

class Cfg_fixed_tree : public CfgManager {
public:

  //===================================================================
  //  Constructors and Destructor
  //===================================================================

  Cfg_fixed_tree(int _numofJoints);
  ~Cfg_fixed_tree();


  //===================================================================
  //  Other Methods
  //===================================================================
  virtual Vector3D GetRobotCenterPosition(const Cfg & c) const;
  virtual Cfg GetRandomCfg(double R, double rStep);
  virtual Cfg GetRandomCfg_CenterOfMass(double *boundingBox);
  virtual Cfg GetRandomRay(double incr);

  // for rotate-at-s Local Planner.
  virtual vector<Cfg> GetMovingSequenceNodes(const Cfg& c1, const Cfg& c2, double s);

  // methods for Cfg generation and collision checking.
  virtual bool ConfigEnvironment(const Cfg &c, Environment *env);

  // Node Generation methods
  virtual bool GenerateOverlapCfg(Environment *env, int robot, 
         Vector3D robot_start, Vector3D robot_goal, Cfg *resultCfg); // OBPRM 
  virtual vector<Cfg> GenSurfaceCfgs4ObstNORMAL(Environment * env,
         CollisionDetection *,int obstacle, int nCfgs, SID _cdsetid, CDInfo& _cdInfo); // NORMAL

  
  virtual bool isInRange(const Cfg &c); 


protected:
  int NumofJoints;
  
private:



}; 
#endif
