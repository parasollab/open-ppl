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
  Cfg_free_tree(const Vector6D& _v);
  Cfg_free_tree(const vector<double>& _v);
  Cfg_free_tree(const Cfg&c);
  Cfg_free_tree(double x, double y, double z, 
		double roll, double pitch, double yaw);
  virtual ~Cfg_free_tree();
  
  #ifdef _PARALLEL
    void define_type(stapl::typer &t)  
    {
      Cfg_free::define_type(t);
      
    }
#endif

  static int  GetNumOfJoints() { return m_numOfJoints; }

  // setNumofJoints should be consistent in every class
  static void SetNumOfJoints(int _numofjoints) { m_numOfJoints = _numofjoints; }

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods : Retrive and set related information of this class
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Access Methods*/
  //@{

  ///The center position is get from param, c, configuration. (The position part of c)
  virtual Vector3D GetRobotCenterPosition() const;
  virtual Vector3D GetRobotCenterofMass(Environment* env) const;

  virtual const char* GetName() const;

  ///Move the (the first link of)  robot in enviroment to the given configuration.
  virtual bool ConfigEnvironment(Environment*) const;
  //===================================================================
  //  Other Methods
  //===================================================================
  virtual void GetRandomCfg(double R, double rStep);
  virtual void GetRandomCfg(Environment* _env);
  virtual void GetRandomCfg(Environment *_env,shared_ptr<Boundary> _bb);
 protected:
  ///Randomly generate a Cfg whose center positon is inside a given bounding box.(rotation, don't care!)
  virtual void GetRandomCfg_CenterOfMass(Environment* _env);
  virtual void GetRandomCfg_CenterOfMass(Environment* _env,shared_ptr<Boundary> _bb);
  static int m_numOfJoints;

 private:
};

#endif
