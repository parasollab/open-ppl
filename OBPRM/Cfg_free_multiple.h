/////////////////////////////////////////////////////////////////////
//
//  Cfg_free_multiple.h
//
//  General Description
//      A derived class from CfgManager. It provides some 
//      specific implementation directly related to a set of free
//      flying objects.
//
//  Created
//      06/29/00        Sujay
//
//  Last Modified By:
//	09/07/00	Ian
//
/////////////////////////////////////////////////////////////////////

#ifndef Cfg_free_multiple_h
#define Cfg_free_multiple_h

#include "CfgManager.h"

class Body;
class Environment;
class Cfg;

class Cfg_free_multiple : public CfgManager {
public:

  //===================================================================
  //  Constructors and Destructor
  //===================================================================

  Cfg_free_multiple(int robots);

  ~Cfg_free_multiple();


  //===================================================================
  //  Other Methods
  //===================================================================
  

  virtual Cfg FindIncrement(const Cfg& c, const Cfg& _goal, int * n_ticks,
                            double positionRes, double orientationRes);
  virtual Cfg FindIncrement(const Cfg& c, const Cfg& _goal, int  n_ticks);
  
  virtual void IncrementTowardsGoal(Cfg& c, const Cfg &goal, const Cfg &increment);

  virtual Cfg GetRandomCfg(double R, double rStep);

  virtual Cfg GetRandomCfg_CenterOfMass(double *boundingBox);

  // get entire dof mask
  vector<vector<bool> > MaskGet(void);

  // get the dof mask for a particular robot
  vector<bool> MaskGet(int robot_id);

  // get a single dof from a particular robot 
  bool MaskGet(int robot_id, int dof_id);
	
  // make new mask from a prebuilt set of dof flags
  bool MaskNew(vector<vector<bool> > newmask_vec);

  // make new (default all active) mask.  numdof_vec provides the dof
  // count for each robot. 
  bool MaskNew(vector<int> numdof_vec);

  // make new (default all active) mask of <robots> robots each with
  // <numdof> dof. 
  bool MaskNew(int robots, int numdof);

  // set entire dof mask
  bool MaskSet(vector<vector<bool> > newmask_vec);

  // set mask for a particular robot
  bool MaskSet(int robot_id, vector<bool> newmask);

  // set flag for a particular dof of a particular robot
  bool MaskSet(int robot_id, int dof_id, bool state);

  // add robot to mask with initial flags
  int MaskAddRobot(vector<bool> newmask);

  // add robot to mask with <numdof> dof
  int MaskAddRobot(int numdof);

  // remove a robot from the mask
  int MaskDelRobot(int robot_id);
	
  // revert dof mask to state before last change
  bool MaskPrevious(void);

  // activate all dof
  bool MaskOn(void);

  // activate dof of a particular robot
  bool MaskOn(int robot_id);

  // deactivate all dof
  bool MaskOff(void);

  //deactivate dof for a particular robot
  bool MaskOff(int robot_id);

  //temporarily unnecessary functions 
  virtual Vector3D GetRobotCenterPosition(const Cfg & c) const; // {};
  virtual Cfg GetRandomRay(double incr); // {};

  // methods for Cfg generation and collision checking.
  virtual bool ConfigEnvironment(const Cfg &c, Environment *env); // {};

  // Node Generation methods
  virtual bool GenerateOverlapCfg(Environment *env, int robot, 
    Vector3D robot_start, Vector3D robot_goal, Cfg *resultCfg); // OBPRM
  virtual vector<Cfg> GenSurfaceCfgs4ObstNORMAL(Environment * env,
    CollisionDetection *,int obstacle, int nCfgs, SID _cdsetid, 
    CDInfo& _cdInfo) {cout << "gensurf" << endl;}; // NORMAL


protected:
    vector<vector<bool> > mask;
    vector<vector<bool> > oldmask;

private:

}; 
#endif
