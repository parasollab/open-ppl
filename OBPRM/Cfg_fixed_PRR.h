// $Id$

/**@file Cfg_fixed_PRR.h
   A derived class from CfgManager. It provides some specific
   implementation closely related to a PRR robot.
   Degree of Freedom: 3
   @author Guang Song
   @date 08/31/99
*/


#ifndef Cfg_fixed_PRR_h
#define Cfg_fixed_PRR_h


#include "CfgManager.h"

class Cfg_fixed_PRR : public CfgManager {
public:

  //===================================================================
  //  Constructors and Destructor
  //===================================================================

  Cfg_fixed_PRR();
  ~Cfg_fixed_PRR();


  //===================================================================
  //  Other Methods
  //===================================================================


  virtual Vector3D GetRobotCenterPosition(const Cfg & c) const;
  virtual Cfg GetRandomCfg(double R, double rStep);
  virtual Cfg GetRandomCfg_CenterOfMass(double *boundingBox);
  virtual Cfg GetRandomRay(double incr);

  /// methods for Cfg generation and collision checking.
  virtual bool ConfigEnvironment(const Cfg &c, Environment *env);

  /// Node Generation methods: OBPRM
  virtual bool GenerateOverlapCfg(Environment *env, int robot,
         Vector3D robot_start, Vector3D robot_goal, Cfg *resultCfg);
  /// Node Generation methods: NORMAL
  virtual vector<Cfg> GenSurfaceCfgs4ObstNORMAL(Environment * env,
         CollisionDetection *,int obstacle, int nCfgs,
    SID _cdsetid, CDInfo& _cdInfo);

  protected:

  private:



} ;
#endif
