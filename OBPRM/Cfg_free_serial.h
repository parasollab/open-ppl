// $Id$
/////////////////////////////////////////////////////////////////////
//
//  Cfg_free_serial.h
//
//  General Description
//      A derived template class from CfgManager. It provides some
//      specific implementation directly related to a multiple joints
//      serial robot.
//
//  Created
//      08/31/99        Guang Song
//
//  Last Modified By:
//
/////////////////////////////////////////////////////////////////////

#ifndef Cfg_free_serial_h
#define Cfg_free_serial_h

#include "CfgManager.h"

class Cfg_free_serial : public CfgManager {
public:

  //===================================================================
  //  Constructors and Destructor
  //===================================================================

  Cfg_free_serial(int _numofJoints);
  ~Cfg_free_serial();


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
         CollisionDetection *,int obstacle, int nCfgs, SID _cdsetid); // NORMAL


  protected:
     int NumofJoints;

  private:



} ;
#endif
