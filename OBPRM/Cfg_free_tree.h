// $Id$
/**@file Cfg_free_tree.h
   A derived template class from CfgManager. It provides some
   specific implementation directly related to a multiple joints
   serial robot.
   @author Guang Song
   @date 08/31/99
*/

#ifndef Cfg_free_tree_h
#define Cfg_free_tree_h

#include "Cfg_free.h"

class Cfg_free_tree : public Cfg_free {
 public:
  
  //===================================================================
  //  Constructors and Destructor
  //===================================================================

  Cfg_free_tree();
  Cfg_free_tree(int _numofjoints);
  Cfg_free_tree(const Vector6<double>& _v);
  Cfg_free_tree(const vector<double>& _v);
  Cfg_free_tree(const Cfg&c);
  Cfg_free_tree(double x, double y, double z, 
		double roll, double pitch, double yaw);
  ~Cfg_free_tree();

  static int  getNumofJoints() { return NumofJoints; }
  static void setNumofJoints(int _numofjoints) { NumofJoints = _numofjoints; }

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods : Retrive and set related information of this class
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Access Methods*/
  //@{

  virtual void equals(const Cfg&);

  ///The center position is get from param, c, configuration. (The position part of c)
  virtual Vector3D GetRobotCenterPosition() const;

  virtual const char* GetName() const;

  ///Move the (the first link of)  robot in enviroment to the given configuration.
  virtual bool ConfigEnvironment(Environment*) const;
  //===================================================================
  //  Other Methods
  //===================================================================
  virtual void GetRandomCfg(double R, double rStep);
  virtual void GetRandomCfg(Environment* env);
  virtual void GetRandomRay(double incr);

  /// Node Generation methods: OBPRM
  virtual bool GenerateOverlapCfg(Environment *env, int robot,
				  Vector3D robot_start, Vector3D robot_goal, 
				  Cfg *resultCfg);

  /// Node Generation methods: NORMAL
  virtual void GenSurfaceCfgs4ObstNORMAL(Environment * env, Stat_Class& Stats,
					 CollisionDetection *,
					 int obstacle, int nCfgs,
					 CDInfo& _cdInfo, 
					 vector<Cfg*>&surface);

  virtual Cfg* CreateNewCfg() const;
  virtual Cfg* CreateNewCfg(vector<double>&) const;


 protected:
  ///Randomly generate a Cfg whose center positon is inside a given bounding box.(rotation, don't care!)
  virtual void GetRandomCfg_CenterOfMass(Environment* env);

  static   int NumofJoints;

 private:
};

#endif
