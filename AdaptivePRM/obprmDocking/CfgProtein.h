/////////////////////////////////////////////////////////////////////
//
//  CfgProtein.h
//
//  General Description
//      A derived template class from Cfg_fixed_tree. It provides some 
//      specific implementation directly related to polypeptide chain.
//
//      
//
//  Created
//      04/22/2000	Guang Song
//	
//
//  Last Modified By:
//
/////////////////////////////////////////////////////////////////////

#ifndef CfgProtein_h
#define CfgProtein_h

#include "Cfg_fixed_tree.h"

class CfgProtein : public Cfg_fixed_tree {
public:

  //===================================================================
  //  Constructors and Destructor
  //===================================================================

  CfgProtein(int _numofJoints);
  ~CfgProtein();


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

  
  virtual bool isInRange(const Cfg &c); 

  // a few methods exclusively for protein folding.
  virtual bool isCollision(Cfg &c, Environment *env, CollisionDetection *cd,
			   SID _cdsetid, CDInfo& _cdInfo,
			   bool enablePenetration=true);
  //virtual double GetPotential(const Cfg &c, Environment * env);
  //virtual vector<Vector3D> GetCoordinates(Environment * env);

  ///////////////////////////////////////////////////
  //  Data ..
  ///////////////////////////////////////////////////
  static double std; // both parameters are for node sampling.
  static double range;

protected:
private:



}; 
#endif
