// $Id$

/**@file Cfg_free.h
  *A derived class from CfgManager. It provides some specific
  *implementation for a 6-dof rigid-body moving in a 3-D work space.
  *
  *@date 08/31/99
  *@author Guang Song
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Cfg_free_h
#define Cfg_free_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include obprm headers
#include "Cfg.h"

////////////////////////////////////////////////////////////////////////////////////////////
class GMSPolyhedron;

////////////////////////////////////////////////////////////////////////////////////////////
/**
  *A derived class from CfgManager. It provides some specific
  *implementation for a 6-dof rigid-body moving in a 3-D work space.
  */
class Cfg_free : public Cfg {
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
  Cfg_free();
  Cfg_free(double x, double y, double z, double roll, double pitch, double yaw);
  Cfg_free(const Vector6<double>& _v);
  Cfg_free(const Cfg& c);
  ///Do nothing
  ~Cfg_free();
  //@}

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

  /**Randomly generate a Cfg
    *@param R This new Cfg will have distance (position) R from origin
    *@param rStep
    *@todo what is rStep?
    */
  virtual void GetRandomCfg(double R, double rStep);
  virtual void GetRandomCfg(Environment* env);

  ///Get a random vector whose magnitude is incr (note. the orienatation of of this Cfg is 0)
  virtual void GetRandomRay(double incr, Environment* env, DistanceMetric* dm);
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    methods for nodes generation 
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Node Generation*/
  //@{

  /**Node Generation methods: OBPRM.
    *Generate a new Cfg, and put it in resultCfg.
    *The position of new cfg is from (robot_goal-robot_start)
    *The orientation of new cfg is generated randomly.
    */
  virtual bool GenerateOverlapCfg(Environment* env, int robot,
				  Vector3D robot_start, Vector3D robot_goal, 
				  Cfg* resultCfg);

  /**Node Generation methods: NORMAL
    *generate nodes by overlapping two triangles' normal.
    */
  virtual void GenSurfaceCfgs4ObstNORMAL(Environment* env, Stat_Class& Stats, 
					 CollisionDetection *,
					 int obstacle, int nCfgs,
					 CDInfo& _cdInfo,
					 vector<Cfg*>&) const;
  
  /**@todo Document this
    */
  virtual void GetCfgByOverlappingNormal(Environment* env, Stat_Class& Stats,
					 CollisionDetection* cd,
					 const GMSPolyhedron &polyRobot, 
					 const GMSPolyhedron &polyObst,
					 int robTri, int obsTri,
					 CDInfo& _cdInfo,
					 MultiBody*, vector<Cfg*>&) const;

  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Helper functions
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /*@name Helper functions*/
  //@{

  /**Check if a given configuration c is inside narrow passage.
    *This is done by moving c a little bit and check for collision.
    *return true if inside narrow passage.
    */
  virtual bool InNarrowPassage(Environment* env, Stat_Class& Stats,
			       CollisionDetection* cd, CDInfo& _cdInfo,
			       MultiBody* onflyRobot) const;
  //@}


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

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    private Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  private:

};

#endif
