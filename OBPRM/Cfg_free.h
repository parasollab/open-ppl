// $Id$
/////////////////////////////////////////////////////////////////////
//
//  Cfg_free.h
//
//  General Description
//      A derived class from CfgManager. It provides some specific
//      implementation for a 6-dof rigid-body moving in a 3-D
//      work space.
//
//  Created
//      08/31/99        Guang Song
//
/////////////////////////////////////////////////////////////////////

#ifndef Cfg_free_h
#define Cfg_free_h

#include "CfgManager.h"

class Cfg_free : public CfgManager {
public:

  //===================================================================
  //  Constructors and Destructor
  //===================================================================

  Cfg_free();
  ~Cfg_free();


  //===================================================================
  //  Other Methods
  //===================================================================
  virtual Vector3D GetRobotCenterPosition(const Cfg & c) const;
  virtual Cfg GetRandomCfg(double R, double rStep);
  virtual Cfg GetRandomCfg_CenterOfMass(double *boundingBox);
  virtual Cfg GetRandomRay(double incr);

  // methods for Cfg generation and collision checking.
  virtual bool ConfigEnvironment(const Cfg &c, Environment *env);

  // Node Generation methods
  virtual bool GenerateOverlapCfg(Environment *env, int robot,
         Vector3D robot_start, Vector3D robot_goal, Cfg *resultCfg); // OBPRM
  virtual vector<Cfg> GenSurfaceCfgs4ObstNORMAL(Environment * env,
         CollisionDetection *,int obstacle, int nCfgs, 
	SID _cdsetid,CDInfo& _cdInfo); // NORMAL

  virtual vector<Cfg> GetCfgByOverlappingNormal(
	Environment * env, CollisionDetection* cd, 
	const GMSPolyhedron &polyRobot, const GMSPolyhedron &polyObst, 
	int robTri, int obsTri, 
	SID _cdsetid, CDInfo& _cdInfo,
	MultiBody *);

  virtual bool InNarrowPassage(
	const Cfg& c, Environment * env,CollisionDetection* cd,
	SID _cdsetid, CDInfo& _cdInfo,
	MultiBody * onflyRobot);

  protected:

  private:



} ;
#endif
