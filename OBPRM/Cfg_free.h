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
#include "CfgManager.h"

////////////////////////////////////////////////////////////////////////////////////////////
class GMSPolyhedron;

////////////////////////////////////////////////////////////////////////////////////////////
/**
  *A derived class from CfgManager. It provides some specific
  *implementation for a 6-dof rigid-body moving in a 3-D work space.
  */
class Cfg_free : public CfgManager {
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

  ///The center position is get from param, c, configuration. (The position part of c)
  virtual Vector3D GetRobotCenterPosition(const Cfg & c) const;

  /**Randomly generate a Cfg
    *@param R This new Cfg will have distance (position) R from origin
    *@param rStep
    *@todo what is rStep?
    */
  virtual Cfg GetRandomCfg(double R, double rStep);

  ///Randomly generate a Cfg whose center positon is inside a given bounding box.(rotation, don't care!)
  virtual Cfg GetRandomCfg_CenterOfMass(Environment *env);

  ///Get a random vector whose magnitude is incr (note. the orienatation of of this Cfg is 0)
  virtual Cfg GetRandomRay(double incr);
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
  virtual bool GenerateOverlapCfg(Environment *env, int robot,
         Vector3D robot_start, Vector3D robot_goal, Cfg *resultCfg);

  /**Node Generation methods: NORMAL
    *generate nodes by overlapping two triangles' normal.
    */
  virtual vector<Cfg> GenSurfaceCfgs4ObstNORMAL(Environment * env,
         CollisionDetection *,int obstacle, int nCfgs,
    SID _cdsetid,CDInfo& _cdInfo);

  /**@todo Document this
    */
  virtual vector<Cfg> GetCfgByOverlappingNormal(
    Environment * env, CollisionDetection* cd,
    const GMSPolyhedron &polyRobot, const GMSPolyhedron &polyObst,
    int robTri, int obsTri,
    SID _cdsetid, CDInfo& _cdInfo,
    MultiBody *);

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

  ///Move the (the first link of)  robot in enviroment to the given configuration.
  virtual bool ConfigEnvironment(const Cfg &c, Environment *env);

  /**Check if a given configuration c is inside narrow passage.
    *This is done by moving c a little bit and check for collision.
    *return true if inside narrow passage.
    */
  virtual bool InNarrowPassage(
    const Cfg& c, Environment * env,CollisionDetection* cd,
    SID _cdsetid, CDInfo& _cdInfo,
    MultiBody * onflyRobot);
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    protected Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  protected:

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
