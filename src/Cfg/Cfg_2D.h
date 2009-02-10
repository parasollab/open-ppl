// $Id$
/**@file Cfg_2D.h
  *A derived class from Cfg_free. It provides some specific
  *implementation for a 3-dof rigid-body moving in a 2-D work space.
  *
  *@date 12/21/01
  *@author Jinsuck Kim
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Cfg_2D_h
#define Cfg_2D_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include obprm headers
#include "Cfg_free.h"

////////////////////////////////////////////////////////////////////////////////////////////
/**
  *A derived class from Cfg_free. It provides some specific
  *implementation for a 3-dof rigid-body moving in a 2-D work space.
  */
class Cfg_2D : public Cfg_free {
public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  
  //===================================================================
  /**@name  Constructors and Destructor*/
  //===================================================================
  //@{
  ///Degree of freedom is 6 and Degree of freedom for position part is 3.
  Cfg_2D();
  Cfg_2D(const Cfg&c);
  Cfg_2D(const Vector6<double>& _v);
  Cfg_2D(double, double, double, double, double, double);

  ///Do nothing
  virtual ~Cfg_2D();
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods : Retrive and set related information of this class
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  virtual void GetRandomCfg(double R, double rStep);
  virtual void GetRandomCfg(Environment *env);
  virtual void GetRandomRay(double incr, Environment* env, DistanceMetric* dm);

  virtual void equals(const Cfg&);

  ///The center position is get from param, c, configuration. (The position part of c)
  virtual Vector3D GetRobotCenterPosition() const;

  virtual const char* GetName() const;

  ///Move the (the first link of)  robot in enviroment to the given configuration.
  virtual bool ConfigEnvironment(Environment*) const;

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    methods for nodes generation 
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  virtual bool GenerateOverlapCfg(Environment *env, int robot,
				  Vector3D robot_start, Vector3D robot_goal, 
				  Cfg *resultCfg);

  virtual void GenSurfaceCfgs4ObstNORMAL(Environment * env, Stat_Class& Stats,
					 CollisionDetection *,
					 int obstacle, int nCfgs, 
					 CDInfo& _cdInfo, vector<Cfg*>&);

  virtual void GetCfgByOverlappingNormal(Environment * env, Stat_Class& Stats,
					 CollisionDetection* cd,
					 const GMSPolyhedron &polyRobot, const GMSPolyhedron &polyObst,
					 int robTri, int obsTri,
					 CDInfo& _cdInfo,
					 shared_ptr<MultiBody>, vector<Cfg*>);

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Helper functions
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  virtual Cfg* CreateNewCfg() const;
  virtual Cfg* CreateNewCfg(vector<double>&) const;

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    protected Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
 protected:
  ///Randomly generate a Cfg whose center positon is inside a given bounding box.(rotation, don't care!)
  virtual void GetRandomCfg_CenterOfMass(Environment* env);

  void ForceItTo2D(); 

};

#endif
