/**@file Cfg_free_tree.h
   A derived template class from CfgManager. It provides some
   specific implementation directly related to a multiple joints
   serial robot.
   @author Guang Song
   @date 08/31/99
*/



#ifndef Cfg_free_tree_h
#define Cfg_free_tree_h

#include "CfgManager.h"

class Cfg_free_tree : public CfgManager {
public:

  //===================================================================
  //  Constructors and Destructor
  //===================================================================

  Cfg_free_tree(int _numofJoints);
  ~Cfg_free_tree();
  
  /// create and clone functions
  CfgManager * create() const { return (new Cfg_free_tree(NumofJoints)); }
  CfgManager * clone() const;


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
     int NumofJoints;

  private:



} ;
#endif
